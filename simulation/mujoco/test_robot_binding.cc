#include "robot_binding.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Model = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using Data = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void Near(double actual, double expected, const std::string& message) {
  Require(std::abs(actual - expected) < 1e-10, message + ": " + std::to_string(actual));
}

template <typename Operation>
void Throws(Operation operation, const std::string& message) {
  try {
    operation();
  } catch (const std::runtime_error&) {
    return;
  }
  throw std::runtime_error(message);
}

std::string RobotBody(const std::string& prefix, double x) {
  return "<body name='" + prefix + "base_body' pos='" + std::to_string(x) + " 0 1'>"
      "<freejoint/><geom type='sphere' size='.1' mass='1'/>"
      "<site name='" + prefix + "base'/><site name='" + prefix + "imu' pos='0 0 .1'/>"
      // An unactuated DOF precedes the controlled joint, defeating a +7/+6 slice.
      "<body><joint name='" + prefix + "unactuated'/><geom type='sphere' size='.03' mass='.1'/></body>"
      "<body><joint name='" + prefix + "knee'/><geom type='sphere' size='.04' mass='.2'/></body>"
      "<body pos='0 .1 -.5'><geom type='sphere' size='.04' mass='.1'/>"
      "<site name='" + prefix + "left_foot'/></body>"
      "<body pos='0 -.1 -.5'><geom type='sphere' size='.04' mass='.1'/>"
      "<site name='" + prefix + "right_foot'/></body></body>";
}

std::string RobotSensors(const std::string& p, bool omit_quaternion) {
  // Deliberately not the deployment sensor order.
  std::string sensors =
      "<gyro name='" + p + "imu_angular_velocity' site='" + p + "imu'/>"
      "<torque name='" + p + "torque_right_foot' site='" + p + "right_foot'/>"
      "<framepos name='" + p + "imu_link_position' objtype='site' objname='" + p + "imu'/>"
      "<force name='" + p + "force_left_foot' site='" + p + "left_foot'/>"
      "<frameangvel name='" + p + "base_link_angular_velocity' objtype='site' objname='" + p + "base'/>"
      "<framequat name='" + p + "base_link_quaternion' objtype='site' objname='" + p + "base'/>"
      "<accelerometer name='" + p + "imu_linear_acceleration' site='" + p + "imu'/>"
      "<framelinvel name='" + p + "imu_link_linear_velocity' objtype='site' objname='" + p + "imu'/>"
      "<force name='" + p + "force_right_foot' site='" + p + "right_foot'/>"
      "<framepos name='" + p + "base_link_position' objtype='site' objname='" + p + "base'/>"
      "<framequat name='" + p + "imu_link_quaternion' objtype='site' objname='" + p + "imu'/>"
      "<torque name='" + p + "torque_left_foot' site='" + p + "left_foot'/>"
      "<frameangvel name='" + p + "imu_link_angular_velocity' objtype='site' objname='" + p + "imu'/>"
      "<framelinvel name='" + p + "base_link_linear_velocity' objtype='site' objname='" + p + "base'/>";
  if (!omit_quaternion) {
    sensors += "<framequat name='" + p + "imu_quaternion' objtype='site' objname='" + p + "imu'/>";
  }
  return sensors;
}

Model Fixture(bool reverse_roots = false, bool omit_sensor = false) {
  const std::string first = reverse_roots ? "robot_b/" : "robot_a/";
  const std::string second = reverse_roots ? "robot_a/" : "robot_b/";
  const std::string xml = "<mujoco><option gravity='0 0 0'/><worldbody>" +
      RobotBody(first, -1) + RobotBody(second, 1) + "</worldbody><actuator>" +
      // Opposite to the body order; actuator addresses cannot be joint addresses.
      "<motor name='" + second + "motor' joint='" + second + "knee'/>" +
      "<motor name='" + first + "motor' joint='" + first + "knee'/>" +
      "</actuator><sensor>" + RobotSensors(second, omit_sensor && second == "robot_a/") +
      RobotSensors(first, omit_sensor && first == "robot_a/") + "</sensor></mujoco>";
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  Require(mj_addBufferVFS(&vfs, "fixture.xml", xml.data(), static_cast<int>(xml.size())) == 0,
          "Cannot create MuJoCo fixture");
  char error[1024] = {};
  Model model(mj_loadXML("fixture.xml", &vfs, error, sizeof(error)), &mj_deleteModel);
  mj_deleteVFS(&vfs);
  Require(model != nullptr, std::string("Invalid fixture: ") + error);
  return model;
}

int Joint(const mjModel* m, const std::string& prefix) {
  const int joint = mj_name2id(m, mjOBJ_JOINT, (prefix + "knee").c_str());
  Require(joint >= 0, "Fixture knee not found");
  return joint;
}

int Actuator(const mjModel* m, const std::string& prefix) {
  const int actuator = mj_name2id(m, mjOBJ_ACTUATOR, (prefix + "motor").c_str());
  Require(actuator >= 0, "Fixture motor not found");
  return actuator;
}

void Sensor(const mjModel* m, mjData* d, const std::string& name, std::initializer_list<double> values) {
  const int id = mj_name2id(m, mjOBJ_SENSOR, name.c_str());
  Require(id >= 0 && m->sensor_dim[id] == static_cast<int>(values.size()), "Invalid sensor fixture " + name);
  std::copy(values.begin(), values.end(), d->sensordata + m->sensor_adr[id]);
}

std::shared_ptr<LcmDataStore> Store() {
  return std::make_shared<LcmDataStore>(1, 3, 6);
}

RobotBinding Binding(const std::string& prefix, const std::shared_ptr<LcmDataStore>& store) {
  return RobotBinding(prefix, store, {"knee"}, {"LINK_FOOT_L", "LINK_FOOT_R", "LINK_WRIST_END_L"});
}

int64_t Now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

void Command(const std::shared_ptr<LcmDataStore>& store, double q, double qd, double kp, double kd, double ff) {
  SimCommand command(1);
  command.q[0] = q;
  command.qd[0] = qd;
  command.kp[0] = kp;
  command.kd[0] = kd;
  command.tau_ff[0] = ff;
  store->sim_command.Set(command);
  store->last_command_time_ns.store(Now());
}

void TestAddressAndCommandIsolation() {
  auto m = Fixture();
  Data d(mj_makeData(m.get()), &mj_deleteData);
  auto store_a = Store();
  auto store_b = Store();
  auto a = Binding("robot_a/", store_a);
  auto b = Binding("robot_b/", store_b);
  a.Resolve(m.get());
  b.Resolve(m.get());
  const int ja = Joint(m.get(), "robot_a/");
  const int jb = Joint(m.get(), "robot_b/");
  const int aa = Actuator(m.get(), "robot_a/");
  const int ab = Actuator(m.get(), "robot_b/");
  Require(m->nq == 18 && m->nv == 16, "Fixture must contain two free roots and extra DOFs");
  Require(m->jnt_qposadr[ja] != 7 && m->jnt_dofadr[ja] != 6, "Fixture must defeat fixed-offset mapping");
  Require(aa > ab, "Fixture actuator order must differ from root order");
  d->qpos[m->jnt_qposadr[ja]] = .4;
  d->qvel[m->jnt_dofadr[ja]] = .1;
  d->qpos[m->jnt_qposadr[jb]] = -.3;
  d->qvel[m->jnt_dofadr[jb]] = -.2;
  mj_forward(m.get(), d.get());
  d->actuator_force[aa] = 11;
  d->actuator_force[ab] = 22;
  Sensor(m.get(), d.get(), "robot_a/base_link_position", {1, 2, 3});
  Sensor(m.get(), d.get(), "robot_b/base_link_position", {4, 5, 6});
  Sensor(m.get(), d.get(), "robot_a/base_link_quaternion", {.5, .5, .5, .5});
  Sensor(m.get(), d.get(), "robot_b/base_link_quaternion", {0, 0, 0, 1});
  Sensor(m.get(), d.get(), "robot_a/base_link_linear_velocity", {31, 32, 33});
  Sensor(m.get(), d.get(), "robot_b/base_link_angular_velocity", {34, 35, 36});
  Sensor(m.get(), d.get(), "robot_a/imu_link_position", {7, 8, 9});
  Sensor(m.get(), d.get(), "robot_b/imu_link_position", {10, 11, 12});
  Sensor(m.get(), d.get(), "robot_a/imu_link_quaternion", {0, 1, 0, 0});
  Sensor(m.get(), d.get(), "robot_b/imu_link_quaternion", {0, 0, 1, 0});
  Sensor(m.get(), d.get(), "robot_a/imu_link_linear_velocity", {37, 38, 39});
  Sensor(m.get(), d.get(), "robot_b/imu_link_angular_velocity", {40, 41, 42});
  Sensor(m.get(), d.get(), "robot_a/imu_quaternion", {0, 0, 0, 1});
  Sensor(m.get(), d.get(), "robot_b/imu_quaternion", {.5, -.5, .5, -.5});
  Sensor(m.get(), d.get(), "robot_a/imu_linear_acceleration", {13, 14, 15});
  Sensor(m.get(), d.get(), "robot_b/imu_angular_velocity", {16, 17, 18});
  Sensor(m.get(), d.get(), "robot_a/force_left_foot", {19, 20, 21});
  Sensor(m.get(), d.get(), "robot_a/torque_left_foot", {22, 23, 24});
  Sensor(m.get(), d.get(), "robot_b/force_right_foot", {25, 26, 27});
  Sensor(m.get(), d.get(), "robot_b/torque_right_foot", {28, 29, 30});
  a.ReadState(d.get());
  b.ReadState(d.get());
  const auto sa = store_a->sim_state();
  const auto sb = store_b->sim_state();
  Near(sa.q[0], .4, "A joint position");
  Near(sb.q[0], -.3, "B joint position");
  Near(sa.qd[0], .1, "A joint velocity");
  Near(sb.qd[0], -.2, "B joint velocity");
  Near(sa.tau[0], 11, "A measured actuator force");
  Near(sb.tau[0], 22, "B measured actuator force");
  Near(sa.base_link_position[2], 3, "A named base sensor");
  Near(sb.base_link_position[2], 6, "B named base sensor");
  Near(sa.base_link_quaternion.w(), .5, "A base quaternion w-first convention");
  Near(sb.base_link_quaternion.z(), 1, "B base quaternion");
  Near(sa.base_link_linear_velocity[2], 33, "A base linear velocity");
  Near(sb.base_link_angular_velocity[2], 36, "B base angular velocity");
  Near(sa.imu_link_position[1], 8, "A named IMU link sensor");
  Near(sb.imu_link_position[1], 11, "B named IMU link sensor");
  Near(sa.imu_link_quaternion.x(), 1, "A IMU link quaternion");
  Near(sb.imu_link_quaternion.y(), 1, "B IMU link quaternion");
  Near(sa.imu_link_linear_velocity[2], 39, "A IMU link linear velocity");
  Near(sb.imu_link_angular_velocity[2], 42, "B IMU link angular velocity");
  Near(sa.imu_sensor_quaternion.z(), 1, "A IMU sensor quaternion");
  Near(sb.imu_sensor_quaternion.x(), -.5, "B IMU sensor quaternion");
  Near(sa.imu_sensor_linear_acceleration[0], 13, "A accelerometer");
  Near(sb.imu_sensor_angular_velocity[0], 16, "B gyro");
  Near(sa.contact_force[0][0], -19, "A foot force sign");
  Near(sa.contact_force[0][5], -24, "A foot torque address");
  Near(sb.contact_force[1][0], -25, "B foot force sign");
  Near(sb.contact_force[1][5], -30, "B foot torque address");
  Require(sa.contact_force[2].isZero() && sb.contact_force[2].isZero(), "Unmeasured hands must be zero");

  d->ctrl[ab] = 123;
  Command(store_a, 1, .5, 10, 2, 3);
  a.ApplyCommand(d.get());
  Near(d->ctrl[aa], 9.8, "A PD command");
  Near(d->ctrl[ab], 123, "Applying A must leave B unchanged");
  Command(store_b, -1, -.5, 20, 4, -2);
  b.ApplyCommand(d.get());
  Near(d->ctrl[ab], -17.2, "B PD command");
  Near(d->ctrl[aa], 9.8, "Applying B must leave A unchanged");

  store_a->last_command_time_ns.store(Now() - 300000000);
  a.ApplyCommand(d.get());
  Near(d->ctrl[aa], 0, "Stale A command must expire");
  Near(d->ctrl[ab], -17.2, "A timeout must leave B unchanged");
  Command(store_a, 1, 0, 10, 0, 0);
  auto malformed = store_a->sim_command();
  malformed.q.resize(2);
  store_a->sim_command.Set(malformed);
  a.ApplyCommand(d.get());
  Near(d->ctrl[aa], 0, "Wrong command dimensions must be rejected");
  Command(store_a, 1, 0, std::numeric_limits<double>::quiet_NaN(), 0, 0);
  a.ApplyCommand(d.get());
  Near(d->ctrl[aa], 0, "Non-finite command must be rejected");
  Command(store_a, 1, 0, 10, 0, 0);
  a.ClearCommand();
  a.ApplyCommand(d.get());
  Near(d->ctrl[aa], 0, "Cleared command must not be applied");
  Require(store_a->last_command_time_ns.load() == 0, "Clear must invalidate command freshness");
}

void TestMissingContractsAreRejected() {
  auto missing_sensor = Fixture(false, true);
  auto store = Store();
  auto binding = Binding("robot_a/", store);
  Throws([&] { binding.Resolve(missing_sensor.get()); }, "Missing mandatory IMU sensor was accepted");
  auto m = Fixture();
  RobotBinding missing_joint("robot_a/", store, {"missing"}, {"LINK_FOOT_L", "LINK_FOOT_R", "hand"});
  Throws([&] { missing_joint.Resolve(m.get()); }, "Missing joint was accepted");
  RobotBinding wrong_dimensions("robot_a/", store, {"knee"}, {});
  Throws([&] { wrong_dimensions.Resolve(m.get()); }, "Mismatched contact dimensions were accepted");
}

void TestReloadRebuildsAddresses() {
  auto first = Fixture();
  auto second = Fixture(true);
  auto store = Store();
  auto binding = Binding("robot_a/", store);
  binding.Resolve(first.get());
  const int old_address = first->jnt_qposadr[Joint(first.get(), "robot_a/")];
  const int new_address = second->jnt_qposadr[Joint(second.get(), "robot_a/")];
  Require(old_address != new_address, "Reload fixture must change addresses");
  Command(store, 1, 0, 10, 0, 0);
  binding.Resolve(second.get());
  Require(store->last_command_time_ns.load() == 0, "Reload must clear the old command");
  Data d(mj_makeData(second.get()), &mj_deleteData);
  d->qpos[old_address] = -.6;
  d->qpos[new_address] = .7;
  mj_forward(second.get(), d.get());
  binding.ReadState(d.get());
  Near(store->sim_state().q[0], .7, "Reload must use the new joint address");
  Command(store, 1, 0, 10, 0, 0);
  binding.ApplyCommand(d.get());
  Near(d->ctrl[Actuator(second.get(), "robot_a/")], 3, "Reload must use the new actuator address");
  Near(d->ctrl[Actuator(second.get(), "robot_b/")], 0, "Reload must not write the peer actuator");
}

void TestZeroPosePreparation(const mjModel* model, mjData* data, RobotBinding& a, RobotBinding& b,
                             const std::shared_ptr<LcmDataStore>& store_a,
                             const std::shared_ptr<LcmDataStore>& store_b) {
  mj_resetData(model, data);
  mj_forward(model, data);
  SimCommand preparation(25);
  preparation.kp << 1080, 480, 880, 1000, 800, 100,
                    1080, 480, 880, 1000, 800, 100,
                    200, 120, 120, 120, 120, 120,
                    120, 120, 120, 120, 120, 100, 100;
  preparation.kd << 25, 25, 25, 25, 2, 2,
                    25, 25, 25, 25, 2, 2,
                    1, 1, 1, 1, 1, .2,
                    1, 1, 1, 1, .2, 1, 1;
  store_a->sim_command.Set(preparation);
  store_b->sim_command.Set(preparation);
  const std::array<std::string, 2> prefixes = {"robot_a/", "robot_b/"};
  std::array<int, 2> root_addresses;
  std::array<double, 2> min_height;
  std::array<double, 2> max_tilt = {0, 0};
  for (size_t i = 0; i < prefixes.size(); ++i) {
    const int body = mj_name2id(model, mjOBJ_BODY, (prefixes[i] + "LINK_BASE").c_str());
    Require(body >= 0, "Zero-pose probe requires the generated T800 roots");
    const int root = model->body_jntadr[body];
    Require(root >= 0 && model->jnt_type[root] == mjJNT_FREE, "Zero-pose probe requires free roots");
    root_addresses[i] = model->jnt_qposadr[root];
    min_height[i] = data->qpos[root_addresses[i] + 2];
  }
  const int steps = static_cast<int>(std::ceil(3.0 / model->opt.timestep));
  for (int step = 0; step < steps; ++step) {
    store_a->last_command_time_ns.store(Now());
    store_b->last_command_time_ns.store(Now());
    a.ApplyCommand(data);
    b.ApplyCommand(data);
    mj_step(model, data);
    for (size_t i = 0; i < prefixes.size(); ++i) {
      min_height[i] = std::min(min_height[i], data->qpos[root_addresses[i] + 2]);
      mjtNum rotation[9];
      mju_quat2Mat(rotation, data->qpos + root_addresses[i] + 3);
      max_tilt[i] = std::max(max_tilt[i], std::acos(std::clamp(rotation[8], -1.0, 1.0)));
    }
  }
  mj_forward(model, data);
  a.ReadState(data);
  b.ReadState(data);
  const std::array<SimState, 2> states = {store_a->sim_state(), store_b->sim_state()};
  for (size_t i = 0; i < prefixes.size(); ++i) {
    std::cout << "Zero-pose preparation probe " << prefixes[i] << " time=" << data->time
              << " root_z=" << data->qpos[root_addresses[i] + 2]
              << " min_root_z=" << min_height[i] << " max_tilt_rad=" << max_tilt[i]
              << " max_joint_error_rad=" << states[i].q.cwiseAbs().maxCoeff() << '\n';
    Require(min_height[i] > .9, "T800 must retain root height during the three-second preparation");
    Require(max_tilt[i] < .2, "T800 must remain upright during the three-second preparation");
    Require(states[i].q.allFinite() && states[i].q.cwiseAbs().maxCoeff() < .05,
            "T800 must track the zero joint pose at the end of preparation");
  }
  for (int i = 0; i < mjNWARNING; ++i) {
    if (data->warning[i].number) {
      std::cout << "Zero-pose preparation MuJoCo warning index=" << i
                << " count=" << data->warning[i].number << '\n';
    }
    Require(data->warning[i].number == 0, "Three-second T800 preparation produced a MuJoCo warning");
  }
  // This regression checks only three seconds in the spawn preparation pose; it
  // does not establish stability under combat disturbances. Keep the collision
  // test independent of the preparation's final physical state.
  mj_resetData(model, data);
  a.ClearCommand();
  b.ClearCommand();
}

void TestGeneratedScene(const char* filename) {
  char error[1024] = {};
  Model m(mj_loadXML(filename, nullptr, error, sizeof(error)), &mj_deleteModel);
  Require(m != nullptr, std::string("Cannot load generated scene: ") + error);
  Data d(mj_makeData(m.get()), &mj_deleteData);
  std::vector<std::string> joint_names;
  std::vector<int> actuator_a;
  std::vector<int> actuator_b;
  for (int id = 0; id < m->nu; ++id) {
    const char* name = mj_id2name(m.get(), mjOBJ_ACTUATOR, id);
    if (!name) continue;
    const std::string actuator_name(name);
    if (actuator_name.starts_with("robot_a/")) {
      Require(m->actuator_trntype[id] == mjTRN_JOINT, "Real scene must use joint actuators");
      const int joint = m->actuator_trnid[2 * id];
      const std::string joint_name(mj_id2name(m.get(), mjOBJ_JOINT, joint));
      Require(joint_name.starts_with("robot_a/"), "A actuator targets another robot");
      joint_names.push_back(joint_name.substr(std::string("robot_a/").size()));
      actuator_a.push_back(id);
    } else if (actuator_name.starts_with("robot_b/")) {
      actuator_b.push_back(id);
    }
  }
  Require(joint_names.size() == 25 && actuator_b.size() == 25, "Generated scene must have 25 motors per T800");
  const std::vector<std::string> contact_frames = {
      "LINK_FOOT_L", "LINK_FOOT_R", "LINK_WRIST_END_L", "LINK_WRIST_END_R"};
  auto store_a = std::make_shared<LcmDataStore>(25, 4, 6);
  auto store_b = std::make_shared<LcmDataStore>(25, 4, 6);
  RobotBinding a("robot_a/", store_a, joint_names, contact_frames);
  RobotBinding b("robot_b/", store_b, joint_names, contact_frames);
  a.Resolve(m.get());
  b.Resolve(m.get());
  mj_forward(m.get(), d.get());
  a.ReadState(d.get());
  b.ReadState(d.get());
  const auto state_a = store_a->sim_state();
  const auto state_b = store_b->sim_state();
  Require(state_a.q.size() == 25 && state_b.q.size() == 25, "Real T800 state dimensions");
  Require(state_a.base_link_position.x() < 0 && state_b.base_link_position.x() > 0,
          "Real T800s must spawn on opposite sides");
  Require(std::abs(state_a.base_link_quaternion.dot(state_b.base_link_quaternion)) < 1e-8,
          "Real T800s must face opposite directions");
  Require(state_a.contact_force[2].isZero() && state_a.contact_force[3].isZero() &&
              state_b.contact_force[2].isZero() && state_b.contact_force[3].isZero(),
          "Real T800 unmeasured hand contacts must be zero");

  SimCommand command_a(25);
  SimCommand command_b(25);
  command_a.q = state_a.q.array() + .1;
  command_b.q = state_b.q.array() - .2;
  command_a.kp.setConstant(10);
  command_b.kp.setConstant(10);
  for (int i = 0; i < 25; ++i) {
    command_a.tau_ff[i] = .1 * i;
    command_b.tau_ff[i] = -.2 * i;
    d->ctrl[actuator_b[i]] = 123;
  }
  store_a->sim_command.Set(command_a);
  store_b->sim_command.Set(command_b);
  store_a->last_command_time_ns.store(Now());
  store_b->last_command_time_ns.store(Now());
  a.ApplyCommand(d.get());
  for (int i = 0; i < 25; ++i) {
    Near(d->ctrl[actuator_a[i]], 1 + .1 * i, "Real T800 A PD and feed-forward command");
    Near(d->ctrl[actuator_b[i]], 123, "Real T800 A must not write B");
  }
  b.ApplyCommand(d.get());
  for (int i = 0; i < 25; ++i) {
    Near(d->ctrl[actuator_a[i]], 1 + .1 * i, "Real T800 B must not write A");
    Near(d->ctrl[actuator_b[i]], -2 - .2 * i, "Real T800 B PD and feed-forward command");
  }

  TestZeroPosePreparation(m.get(), d.get(), a, b, store_a, store_b);

  // Bring the free roots together without stepping dynamics: this tests collision
  // filtering between instances, not policy stability under a punch.
  for (const auto& prefix : {std::string("robot_a/"), std::string("robot_b/")}) {
    const int body = mj_name2id(m.get(), mjOBJ_BODY, (prefix + "LINK_BASE").c_str());
    Require(body >= 0, "Generated T800 root body missing");
    const int joint = m->body_jntadr[body];
    Require(joint >= 0 && m->jnt_type[joint] == mjJNT_FREE, "Generated T800 requires its own free root");
    const int address = m->jnt_qposadr[joint];
    d->qpos[address] = prefix == "robot_a/" ? -.05 : .05;
    d->qpos[address + 1] = 0;
  }
  mj_forward(m.get(), d.get());
  bool cross_robot_contact = false;
  for (int i = 0; i < d->ncon; ++i) {
    const mjContact& contact = d->contact[i];
    const char* first = mj_id2name(m.get(), mjOBJ_BODY, m->geom_bodyid[contact.geom1]);
    const char* second = mj_id2name(m.get(), mjOBJ_BODY, m->geom_bodyid[contact.geom2]);
    if (!first || !second) continue;
    const std::string body1(first);
    const std::string body2(second);
    cross_robot_contact |= (body1.starts_with("robot_a/") && body2.starts_with("robot_b/")) ||
                           (body1.starts_with("robot_b/") && body2.starts_with("robot_a/"));
  }
  Require(cross_robot_contact, "Generated scene must permit physical contact between the T800s");
  std::cout << "Generated T800 duel scene passed: 2 free robots, 50 isolated motors, cross-robot contact\n";
}
}  // namespace

int main(int argc, char** argv) {
  try {
    TestAddressAndCommandIsolation();
    TestMissingContractsAreRejected();
    TestReloadRebuildsAddresses();
    if (argc > 1) TestGeneratedScene(argv[1]);
    std::cout << "Robot binding tests passed (MuJoCo " << mj_versionString() << ")\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Robot binding test failed: " << error.what() << '\n';
    return 1;
  }
}
