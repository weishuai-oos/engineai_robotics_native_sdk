#include "rl_supermimic/reference.h"

#include <cnpy.h>
#include <zlib.h>
#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <regex>
#include <stdexcept>

#include "rl_supermimic/policy_contract.h"

namespace runner::supermimic {
namespace {

uint32_t LittleEndian(const unsigned char* p, int count) {
  uint32_t value = 0;
  for (int i = 0; i < count; ++i) value |= uint32_t(p[i]) << (8 * i);
  return value;
}

// cnpy preserves word size but not dtype kind: int32 and float32 both have four
// bytes. Inspect the NPY prefix of these small exported entries before cnpy
// interprets their payload. This supports the exporter ZIP format, not general
// encrypted, streamed, or ZIP64 archives.
void ValidateFloat32Headers(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("Cannot open SuperMimic reference: " + path);
  int entries = 0;
  while (true) {
    std::array<unsigned char, 30> header{};
    file.read(reinterpret_cast<char*>(header.data()), header.size());
    if (!file) throw std::runtime_error("Truncated SuperMimic NPZ header");
    const uint32_t signature = LittleEndian(header.data(), 4);
    if (signature == 0x02014b50) break;
    if (signature != 0x04034b50) throw std::runtime_error("Invalid SuperMimic NPZ entry");
    const uint32_t flags = LittleEndian(header.data() + 6, 2);
    const uint32_t method = LittleEndian(header.data() + 8, 2);
    const uint32_t compressed = LittleEndian(header.data() + 18, 4);
    const uint32_t name_size = LittleEndian(header.data() + 26, 2);
    const uint32_t extra_size = LittleEndian(header.data() + 28, 2);
    if ((flags & 9) || (method != 0 && method != 8) || compressed == 0xffffffff ||
        compressed == 0 || compressed > 64 * 1024 * 1024 || name_size < 5)
      throw std::runtime_error("Unsupported SuperMimic NPZ entry format");
    file.seekg(name_size + extra_size, std::ios::cur);
    const auto payload_start = file.tellg();
    std::vector<unsigned char> prefix(std::min<uint32_t>(compressed, 4096));
    file.read(reinterpret_cast<char*>(prefix.data()), prefix.size());
    if (!file) throw std::runtime_error("Truncated SuperMimic NPZ payload");
    std::array<unsigned char, 512> inflated{};
    const unsigned char* npy = prefix.data();
    size_t prefix_size = prefix.size();
    if (method == 8) {
      z_stream stream{};
      stream.next_in = prefix.data();
      stream.avail_in = prefix.size();
      stream.next_out = inflated.data();
      stream.avail_out = inflated.size();
      if (inflateInit2(&stream, -MAX_WBITS) != Z_OK)
        throw std::runtime_error("Cannot inspect SuperMimic compressed NPY header");
      const int status = inflate(&stream, Z_SYNC_FLUSH);
      prefix_size = stream.total_out;
      inflateEnd(&stream);
      if (status != Z_OK && status != Z_STREAM_END)
        throw std::runtime_error("Invalid SuperMimic compressed NPY header");
      npy = inflated.data();
    }
    if (prefix_size < 10 || std::memcmp(npy, "\x93NUMPY", 6) != 0)
      throw std::runtime_error("Invalid SuperMimic NPY signature");
    const size_t header_offset = npy[6] == 1 ? 10 : 12;
    if ((npy[6] != 1 && npy[6] != 2 && npy[6] != 3) || npy[7] != 0 ||
        prefix_size < header_offset)
      throw std::runtime_error("Unsupported SuperMimic NPY version");
    const size_t header_length = LittleEndian(npy + 8, header_offset - 8);
    if (header_length > prefix_size - header_offset)
      throw std::runtime_error("SuperMimic NPY header exceeds inspection limit");
    const std::string text(reinterpret_cast<const char*>(npy + header_offset), header_length);
    static const std::regex dtype(R"(['"]descr['"]\s*:\s*['"](?:<|=|\|)f4['"])");
    if (!std::regex_search(text, dtype))
      throw std::runtime_error("SuperMimic reference arrays must be little-endian float32");
    file.seekg(payload_start + std::streamoff(compressed));
    ++entries;
  }
  if (entries != 8) throw std::runtime_error("SuperMimic reference must contain eight state/goal arrays");
}

void ValidateArray(const cnpy::NpyArray& array, const std::vector<size_t>& shape, const std::string& name) {
  if (array.shape != shape || array.word_size != sizeof(float) || array.fortran_order ||
      !array.data_holder || array.num_vals == 0 || array.num_bytes() != array.num_vals * sizeof(float))
    throw std::runtime_error("Invalid SuperMimic reference shape/dtype: " + name);
  const auto* values = array.data<float>();
  for (size_t i = 0; i < array.num_vals; ++i)
    if (!std::isfinite(values[i])) throw std::runtime_error("Nonfinite SuperMimic reference: " + name);
}

}  // namespace

Reference LoadReference(const std::string& path) {
  ValidateFloat32Headers(path);
  const auto arrays = cnpy::npz_load(path);
  const auto root = arrays.find("root_pos");
  if (root == arrays.end() || root->second.shape.size() != 2 || root->second.shape[0] < 2 ||
      root->second.shape[0] > 100000)
    throw std::runtime_error("Invalid SuperMimic reference frame count");
  const size_t frames = root->second.shape[0];
  const std::map<std::string, std::vector<size_t>> shapes = {
      {"root_pos", {frames, 3}}, {"root_quat", {frames, 4}}, {"root_rotation", {frames, 3, 3}},
      {"root_lin_vel", {frames, 3}}, {"root_ang_vel", {frames, 3}},
      {"joint_pos", {frames, kActions}}, {"joint_vel", {frames, kActions}}, {"goal", {frames, kGoal}}};
  for (const auto& [name, shape] : shapes) {
    const auto found = arrays.find(name);
    if (found == arrays.end()) throw std::runtime_error("SuperMimic reference is missing: " + name);
    ValidateArray(found->second, shape, name);
  }
  Reference reference;
  reference.initial_joint_pos = Eigen::Map<const Eigen::VectorXf>(
      arrays.at("joint_pos").data<float>(), kActions).cast<double>();
  reference.rotations.reserve(frames);
  const float* rotations = arrays.at("root_rotation").data<float>();
  const float* quaternions = arrays.at("root_quat").data<float>();
  for (size_t frame = 0; frame < frames; ++frame) {
    const Eigen::Matrix3d rotation = Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(
                                        rotations + 9 * frame).cast<double>();
    const Eigen::Quaterniond quaternion(quaternions[4 * frame], quaternions[4 * frame + 1],
                                        quaternions[4 * frame + 2], quaternions[4 * frame + 3]);
    if (std::abs(quaternion.norm() - 1.0) > 1e-4 || std::abs(rotation.determinant() - 1.0) > 1e-4 ||
        (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-4 ||
        (rotation - quaternion.normalized().toRotationMatrix()).norm() > 1e-4)
      throw std::runtime_error("Invalid SuperMimic reference root rotation");
    reference.rotations.push_back(rotation);
  }
  reference.goals = Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      arrays.at("goal").data<float>(), frames, kGoal);
  return reference;
}

}  // namespace runner::supermimic
