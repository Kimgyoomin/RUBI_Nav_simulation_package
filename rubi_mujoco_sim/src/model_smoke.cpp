#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

const std::vector<std::string> kExpectedActuators = {
    "L_HR_motor", "L_HP_motor", "L_KN_motor",
    "R_HR_motor", "R_HP_motor", "R_KN_motor"};
const std::vector<std::string> kExpectedJoints = {
    "L_HR_JOINT", "L_HP_JOINT", "L_KN_JOINT",
    "R_HR_JOINT", "R_HP_JOINT", "R_KN_JOINT"};

bool finite_state(const mjModel* model, const mjData* data) {
  for (int i = 0; i < model->nq; ++i) {
    if (!std::isfinite(data->qpos[i])) {
      return false;
    }
  }
  for (int i = 0; i < model->nv; ++i) {
    if (!std::isfinite(data->qvel[i])) {
      return false;
    }
  }
  return std::isfinite(data->time);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) {
    std::cerr << "usage: rubi_mujoco_model_smoke MODEL_XML [STEPS]\n";
    return 64;
  }

  const int steps = argc == 3 ? std::atoi(argv[2]) : 100;
  if (steps < 0) {
    std::cerr << "model_status=FAIL reason=negative_step_count\n";
    return 64;
  }

  char error[1024] = {};
  mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (model == nullptr) {
    std::cerr << "model_status=FAIL load_error=" << error << '\n';
    return 2;
  }
  mjData* data = mj_makeData(model);
  if (data == nullptr) {
    std::cerr << "model_status=FAIL reason=mj_makeData_failed\n";
    mj_deleteModel(model);
    return 3;
  }

  bool valid = true;
  std::cout << "model=" << argv[1] << '\n';
  std::cout << "mujoco_header_version=" << mjVERSION_HEADER << '\n';
  std::cout << "mujoco_runtime_version=" << mj_version() << '\n';
  std::cout << "mujoco_runtime_string=" << mj_versionString() << '\n';
  std::cout << "nq=" << model->nq << " nv=" << model->nv
            << " nu=" << model->nu << " nsensor=" << model->nsensor << '\n';
  std::cout << "timestep=" << model->opt.timestep << '\n';
  valid = valid && mjVERSION_HEADER == mj_version();
  valid = valid && std::abs(model->opt.timestep - 0.002) < 1e-12;
  valid = valid && model->nu == static_cast<int>(kExpectedActuators.size());

  for (int i = 0; i < model->nu; ++i) {
    const char* actuator_name = mj_id2name(model, mjOBJ_ACTUATOR, i);
    const int joint_id = model->actuator_trnid[2 * i];
    const char* joint_name = joint_id >= 0 ? mj_id2name(model, mjOBJ_JOINT, joint_id) : nullptr;
    std::cout << "actuator[" << i << "]=" << (actuator_name ? actuator_name : "<unnamed>")
              << " joint=" << (joint_name ? joint_name : "<none>")
              << " ctrlrange=[" << model->actuator_ctrlrange[2 * i] << ','
              << model->actuator_ctrlrange[2 * i + 1] << "]\n";
    if (i < static_cast<int>(kExpectedActuators.size())) {
      valid = valid && actuator_name && kExpectedActuators[i] == actuator_name;
      valid = valid && joint_name && kExpectedJoints[i] == joint_name;
      valid = valid && model->actuator_ctrllimited[i];
      valid = valid && model->actuator_ctrlrange[2 * i] == -90.0;
      valid = valid && model->actuator_ctrlrange[2 * i + 1] == 90.0;
    }
  }

  for (int i = 0; i < model->nsensor; ++i) {
    const char* name = mj_id2name(model, mjOBJ_SENSOR, i);
    std::cout << "sensor[" << i << "]=" << (name ? name : "<unnamed>")
              << " dim=" << model->sensor_dim[i] << '\n';
  }
  for (const char* sensor : {"orientation", "angular_velocity", "imu_quat"}) {
    valid = valid && mj_name2id(model, mjOBJ_SENSOR, sensor) >= 0;
  }
  std::cout << "accelerometer_present="
            << (mj_name2id(model, mjOBJ_SENSOR, "accelerometer") >= 0 ? "true" : "false")
            << '\n';

  mj_forward(model, data);
  for (int i = 0; i < steps && finite_state(model, data); ++i) {
    for (int actuator = 0; actuator < model->nu; ++actuator) {
      data->ctrl[actuator] = 0.0;
    }
    mj_step(model, data);
  }
  valid = valid && finite_state(model, data);
  std::cout << "steps=" << steps << " final_time=" << data->time << '\n';
  std::cout << "actuator_write=ZERO_ONLY\n";
  std::cout << "model_status=" << (valid ? "PASS" : "FAIL") << '\n';

  mj_deleteData(data);
  mj_deleteModel(model);
  return valid ? 0 : 4;
}
