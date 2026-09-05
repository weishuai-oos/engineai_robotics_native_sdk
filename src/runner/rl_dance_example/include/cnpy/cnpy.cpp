// Copyright (C) 2011  Carl Rogers
// Released under MIT License
// license available in LICENSE file, or at http://www.opensource.org/licenses/mit-license.php

#include "cnpy.h"
#include <stdint.h>
#include <algorithm>
#include <complex>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <regex>
#include <stdexcept>

namespace {

constexpr uint32_t kZip32BitSentinel = 0xffffffffU;
constexpr uint16_t kZip64ExtraFieldId = 0x0001U;

uint16_t ReadLittleEndian16(const std::vector<char>& bytes, size_t offset) {
  if (offset + sizeof(uint16_t) > bytes.size()) throw std::runtime_error("ZIP extra field is truncated");
  return static_cast<uint16_t>(static_cast<uint8_t>(bytes[offset])) |
         (static_cast<uint16_t>(static_cast<uint8_t>(bytes[offset + 1])) << 8);
}

uint64_t ReadLittleEndian64(const std::vector<char>& bytes, size_t offset) {
  if (offset + sizeof(uint64_t) > bytes.size()) throw std::runtime_error("ZIP64 extra field is truncated");
  uint64_t value = 0;
  for (size_t byte = 0; byte < sizeof(uint64_t); ++byte) {
    value |= static_cast<uint64_t>(static_cast<uint8_t>(bytes[offset + byte])) << (byte * 8);
  }
  return value;
}

struct ZipEntrySizes {
  size_t compressed;
  size_t uncompressed;
};

ZipEntrySizes ReadZipEntrySizes(const std::vector<char>& local_header, const std::vector<char>& extra_field) {
  const uint32_t compressed32 = *reinterpret_cast<const uint32_t*>(&local_header[18]);
  const uint32_t uncompressed32 = *reinterpret_cast<const uint32_t*>(&local_header[22]);
  uint64_t compressed = compressed32;
  uint64_t uncompressed = uncompressed32;

  if (compressed32 == kZip32BitSentinel || uncompressed32 == kZip32BitSentinel) {
    bool found_zip64_field = false;
    size_t offset = 0;
    while (offset + 4 <= extra_field.size()) {
      const uint16_t field_id = ReadLittleEndian16(extra_field, offset);
      const uint16_t field_size = ReadLittleEndian16(extra_field, offset + 2);
      offset += 4;
      if (offset + field_size > extra_field.size()) throw std::runtime_error("ZIP extra field is truncated");

      if (field_id == kZip64ExtraFieldId) {
        found_zip64_field = true;
        const size_t field_end = offset + field_size;
        if (uncompressed32 == kZip32BitSentinel) {
          uncompressed = ReadLittleEndian64(extra_field, offset);
          offset += sizeof(uint64_t);
        }
        if (compressed32 == kZip32BitSentinel) {
          compressed = ReadLittleEndian64(extra_field, offset);
          offset += sizeof(uint64_t);
        }
        if (offset > field_end) throw std::runtime_error("ZIP64 extra field is truncated");
        break;
      }
      offset += field_size;
    }
    if (!found_zip64_field) throw std::runtime_error("ZIP64 entry is missing its ZIP64 extra field");
  }

  if (compressed > std::numeric_limits<size_t>::max() || uncompressed > std::numeric_limits<size_t>::max()) {
    throw std::runtime_error("ZIP entry is too large for this platform");
  }
  return {static_cast<size_t>(compressed), static_cast<size_t>(uncompressed)};
}

}  // namespace

char cnpy::BigEndianTest() {
  int x = 1;
  return (((char*)&x)[0]) ? '<' : '>';
}

char cnpy::map_type(const std::type_info& t) {
  if (t == typeid(float)) return 'f';
  if (t == typeid(double)) return 'f';
  if (t == typeid(long double)) return 'f';

  if (t == typeid(int)) return 'i';
  if (t == typeid(char)) return 'i';
  if (t == typeid(short)) return 'i';
  if (t == typeid(long)) return 'i';
  if (t == typeid(long long)) return 'i';

  if (t == typeid(unsigned char)) return 'u';
  if (t == typeid(unsigned short)) return 'u';
  if (t == typeid(unsigned long)) return 'u';
  if (t == typeid(unsigned long long)) return 'u';
  if (t == typeid(unsigned int)) return 'u';

  if (t == typeid(bool)) return 'b';

  if (t == typeid(std::complex<float>)) return 'c';
  if (t == typeid(std::complex<double>)) return 'c';
  if (t == typeid(std::complex<long double>))
    return 'c';

  else
    return '?';
}

template <>
std::vector<char>& cnpy::operator+=(std::vector<char>& lhs, const std::string rhs) {
  lhs.insert(lhs.end(), rhs.begin(), rhs.end());
  return lhs;
}

template <>
std::vector<char>& cnpy::operator+=(std::vector<char>& lhs, const char* rhs) {
  // write in little endian
  size_t len = strlen(rhs);
  lhs.reserve(len);
  for (size_t byte = 0; byte < len; byte++) {
    lhs.push_back(rhs[byte]);
  }
  return lhs;
}

void cnpy::parse_npy_header(unsigned char* buffer, size_t& word_size, std::vector<size_t>& shape, bool& fortran_order) {
  // std::string magic_string(buffer,6);
  uint8_t major_version = *reinterpret_cast<uint8_t*>(buffer + 6);
  uint8_t minor_version = *reinterpret_cast<uint8_t*>(buffer + 7);
  uint16_t header_len = *reinterpret_cast<uint16_t*>(buffer + 8);
  std::string header(reinterpret_cast<char*>(buffer + 9), header_len);

  size_t loc1, loc2;

  // fortran order
  loc1 = header.find("fortran_order") + 16;
  fortran_order = (header.substr(loc1, 4) == "True" ? true : false);

  // shape
  loc1 = header.find("(");
  loc2 = header.find(")");

  std::regex num_regex("[0-9][0-9]*");
  std::smatch sm;
  shape.clear();

  std::string str_shape = header.substr(loc1 + 1, loc2 - loc1 - 1);
  while (std::regex_search(str_shape, sm, num_regex)) {
    shape.push_back(std::stoi(sm[0].str()));
    str_shape = sm.suffix().str();
  }

  // endian, word size, data type
  // byte order code | stands for not applicable.
  // not sure when this applies except for byte array
  loc1 = header.find("descr") + 9;
  bool littleEndian = (header[loc1] == '<' || header[loc1] == '|' ? true : false);
  assert(littleEndian);

  // char type = header[loc1+1];
  // assert(type == map_type(T));

  std::string str_ws = header.substr(loc1 + 2);
  loc2 = str_ws.find("'");
  word_size = atoi(str_ws.substr(0, loc2).c_str());
}

void cnpy::parse_npy_header(FILE* fp, size_t& word_size, std::vector<size_t>& shape, bool& fortran_order) {
  char buffer[256];
  size_t res = fread(buffer, sizeof(char), 11, fp);
  if (res != 11) throw std::runtime_error("parse_npy_header: failed fread");
  std::string header = fgets(buffer, 256, fp);
  assert(header[header.size() - 1] == '\n');

  size_t loc1, loc2;

  // fortran order
  loc1 = header.find("fortran_order");
  if (loc1 == std::string::npos)
    throw std::runtime_error("parse_npy_header: failed to find header keyword: 'fortran_order'");
  loc1 += 16;
  fortran_order = (header.substr(loc1, 4) == "True" ? true : false);

  // shape
  loc1 = header.find("(");
  loc2 = header.find(")");
  if (loc1 == std::string::npos || loc2 == std::string::npos)
    throw std::runtime_error("parse_npy_header: failed to find header keyword: '(' or ')'");

  std::regex num_regex("[0-9][0-9]*");
  std::smatch sm;
  shape.clear();

  std::string str_shape = header.substr(loc1 + 1, loc2 - loc1 - 1);
  while (std::regex_search(str_shape, sm, num_regex)) {
    shape.push_back(std::stoi(sm[0].str()));
    str_shape = sm.suffix().str();
  }

  // endian, word size, data type
  // byte order code | stands for not applicable.
  // not sure when this applies except for byte array
  loc1 = header.find("descr");
  if (loc1 == std::string::npos) throw std::runtime_error("parse_npy_header: failed to find header keyword: 'descr'");
  loc1 += 9;
  bool littleEndian = (header[loc1] == '<' || header[loc1] == '|' ? true : false);
  assert(littleEndian);

  // char type = header[loc1+1];
  // assert(type == map_type(T));

  std::string str_ws = header.substr(loc1 + 2);
  loc2 = str_ws.find("'");
  word_size = atoi(str_ws.substr(0, loc2).c_str());
}

void cnpy::parse_zip_footer(FILE* fp, uint16_t& nrecs, size_t& global_header_size, size_t& global_header_offset) {
  std::vector<char> footer(22);
  fseek(fp, -22, SEEK_END);
  size_t res = fread(&footer[0], sizeof(char), 22, fp);
  if (res != 22) throw std::runtime_error("parse_zip_footer: failed fread");

  uint16_t disk_no, disk_start, nrecs_on_disk, comment_len;
  disk_no = *(uint16_t*)&footer[4];
  disk_start = *(uint16_t*)&footer[6];
  nrecs_on_disk = *(uint16_t*)&footer[8];
  nrecs = *(uint16_t*)&footer[10];
  global_header_size = *(uint32_t*)&footer[12];
  global_header_offset = *(uint32_t*)&footer[16];
  comment_len = *(uint16_t*)&footer[20];

  assert(disk_no == 0);
  assert(disk_start == 0);
  assert(nrecs_on_disk == nrecs);
  assert(comment_len == 0);
}

cnpy::NpyArray load_the_npy_file(FILE* fp) {
  std::vector<size_t> shape;
  size_t word_size;
  bool fortran_order;
  cnpy::parse_npy_header(fp, word_size, shape, fortran_order);

  cnpy::NpyArray arr(shape, word_size, fortran_order);
  size_t nread = fread(arr.data<char>(), 1, arr.num_bytes(), fp);
  if (nread != arr.num_bytes()) throw std::runtime_error("load_the_npy_file: failed fread");
  return arr;
}

cnpy::NpyArray load_the_npz_array(FILE* fp, size_t compr_bytes, size_t uncompr_bytes) {
  std::vector<unsigned char> buffer_compr(compr_bytes);
  std::vector<unsigned char> buffer_uncompr(uncompr_bytes);
  size_t nread = fread(&buffer_compr[0], 1, compr_bytes, fp);
  if (nread != compr_bytes) throw std::runtime_error("load_the_npy_file: failed fread");

  int err;
  z_stream d_stream;

  d_stream.zalloc = Z_NULL;
  d_stream.zfree = Z_NULL;
  d_stream.opaque = Z_NULL;
  d_stream.avail_in = 0;
  d_stream.next_in = Z_NULL;
  err = inflateInit2(&d_stream, -MAX_WBITS);

  d_stream.avail_in = compr_bytes;
  d_stream.next_in = &buffer_compr[0];
  d_stream.avail_out = uncompr_bytes;
  d_stream.next_out = &buffer_uncompr[0];

  err = inflate(&d_stream, Z_FINISH);
  err = inflateEnd(&d_stream);

  std::vector<size_t> shape;
  size_t word_size;
  bool fortran_order;
  cnpy::parse_npy_header(&buffer_uncompr[0], word_size, shape, fortran_order);

  cnpy::NpyArray array(shape, word_size, fortran_order);

  size_t offset = uncompr_bytes - array.num_bytes();
  memcpy(array.data<unsigned char>(), &buffer_uncompr[0] + offset, array.num_bytes());

  return array;
}

cnpy::npz_t cnpy::npz_load(std::string fname) {
  FILE* fp = fopen(fname.c_str(), "rb");

  if (!fp) {
    throw std::runtime_error("npz_load: Error! Unable to open file " + fname + "!");
  }

  cnpy::npz_t arrays;

  while (1) {
    std::vector<char> local_header(30);
    size_t headerres = fread(&local_header[0], sizeof(char), 30, fp);
    if (headerres != 30) throw std::runtime_error("npz_load: failed fread");

    // if we've reached the global header, stop reading
    if (local_header[2] != 0x03 || local_header[3] != 0x04) break;

    // read in the variable name
    uint16_t name_len = *(uint16_t*)&local_header[26];
    std::string varname(name_len, ' ');
    size_t vname_res = fread(&varname[0], sizeof(char), name_len, fp);
    if (vname_res != name_len) throw std::runtime_error("npz_load: failed fread");

    // erase the lagging .npy
    varname.erase(varname.end() - 4, varname.end());

    // read in the extra field
    uint16_t extra_field_len = *(uint16_t*)&local_header[28];
    std::vector<char> extra_field(extra_field_len);
    if (extra_field_len > 0) {
      size_t efield_res = fread(&extra_field[0], sizeof(char), extra_field_len, fp);
      if (efield_res != extra_field_len) throw std::runtime_error("npz_load: failed fread");
    }

    uint16_t compr_method = *reinterpret_cast<uint16_t*>(&local_header[0] + 8);
    const ZipEntrySizes sizes = ReadZipEntrySizes(local_header, extra_field);

    if (compr_method == 0) {
      arrays[varname] = load_the_npy_file(fp);
    } else {
      arrays[varname] = load_the_npz_array(fp, sizes.compressed, sizes.uncompressed);
    }
  }

  fclose(fp);
  return arrays;
}

cnpy::NpyArray cnpy::npz_load(std::string fname, std::string varname) {
  FILE* fp = fopen(fname.c_str(), "rb");

  if (!fp) throw std::runtime_error("npz_load: Unable to open file " + fname);

  while (1) {
    std::vector<char> local_header(30);
    size_t header_res = fread(&local_header[0], sizeof(char), 30, fp);
    if (header_res != 30) throw std::runtime_error("npz_load: failed fread");

    // if we've reached the global header, stop reading
    if (local_header[2] != 0x03 || local_header[3] != 0x04) break;

    // read in the variable name
    uint16_t name_len = *(uint16_t*)&local_header[26];
    std::string vname(name_len, ' ');
    size_t vname_res = fread(&vname[0], sizeof(char), name_len, fp);
    if (vname_res != name_len) throw std::runtime_error("npz_load: failed fread");
    vname.erase(vname.end() - 4, vname.end());  // erase the lagging .npy

    // read in the extra field
    uint16_t extra_field_len = *(uint16_t*)&local_header[28];
    std::vector<char> extra_field(extra_field_len);
    if (extra_field_len > 0) {
      size_t efield_res = fread(&extra_field[0], sizeof(char), extra_field_len, fp);
      if (efield_res != extra_field_len) throw std::runtime_error("npz_load: failed fread");
    }

    uint16_t compr_method = *reinterpret_cast<uint16_t*>(&local_header[0] + 8);
    const ZipEntrySizes sizes = ReadZipEntrySizes(local_header, extra_field);

    if (vname == varname) {
      NpyArray array =
          (compr_method == 0) ? load_the_npy_file(fp) : load_the_npz_array(fp, sizes.compressed, sizes.uncompressed);
      fclose(fp);
      return array;
    } else {
      // Skip the bytes stored in the archive. For compressed entries this is
      // the compressed size, not the uncompressed size.
      if (fseek(fp, static_cast<long>(sizes.compressed), SEEK_CUR) != 0) {
        fclose(fp);
        throw std::runtime_error("npz_load: failed to skip archive entry");
      }
    }
  }

  fclose(fp);

  // if we get here, we haven't found the variable in the file
  throw std::runtime_error("npz_load: Variable name " + varname + " not found in " + fname);
}

cnpy::NpyArray cnpy::npy_load(std::string fname) {
  FILE* fp = fopen(fname.c_str(), "rb");

  if (!fp) throw std::runtime_error("npy_load: Unable to open file " + fname);

  NpyArray arr = load_the_npy_file(fp);

  fclose(fp);
  return arr;
}
