#include <mujoco/mujoco.h>

#ifdef RUBI_MUJOCO_HAS_GLFW
#include <GLFW/glfw3.h>
#endif
#ifdef RUBI_MUJOCO_HAS_PNG
#include <png.h>
#endif

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rubi_control_core/controller.hpp"
#include "rubi_control_core/policy_contract.hpp"
#include "rubi_control_core/policy_runner.hpp"

namespace {

namespace rc = rubi_control_core;
using Trigger = std_srvs::srv::Trigger;

constexpr std::array<const char*, rc::kJointCount> kActuatorNames{
    "L_HR_motor", "L_HP_motor", "L_KN_motor",
    "R_HR_motor", "R_HP_motor", "R_KN_motor"};

template <typename Container>
bool all_finite(const Container& values) {
  return std::all_of(values.begin(), values.end(),
                     [](const auto value) { return std::isfinite(value); });
}

template <typename Container>
bool exact_zero(const Container& values) {
  return std::all_of(values.begin(), values.end(),
                     [](const auto value) { return value == 0; });
}

class PlantAdapter {
 public:
  PlantAdapter(const std::string& model_path, const rclcpp::Logger& logger)
      : logger_(logger) {
    char error[1024] = {};
    model_.reset(mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error)));
    if (!model_) {
      throw std::runtime_error("mj_loadXML failed: " + std::string(error));
    }
    data_.reset(mj_makeData(model_.get()));
    if (!data_) {
      throw std::runtime_error("mj_makeData failed");
    }
    if (std::abs(model_->opt.timestep - 0.002) > 1.0e-12) {
      throw std::runtime_error("MuJoCo timestep is not canonical 0.002 s");
    }
    map_and_validate();
    mj_forward(model_.get(), data_.get());
  }

  rc::RobotState read_state() const {
    rc::RobotState state;
    state.sim_time_sec = data_->time;
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      state.joint_position[i] = data_->qpos[qpos_address_[i]];
      state.joint_velocity[i] = data_->qvel[dof_address_[i]];
      state.measured_effort[i] = data_->actuator_force[actuator_id_[i]];
    }

    const int quaternion_address = model_->sensor_adr[quaternion_sensor_id_];
    const int gyro_address = model_->sensor_adr[gyro_sensor_id_];
    // MuJoCo framequat sensor is wxyz; the core contract stores xyzw.
    state.base_orientation_xyzw = {
        data_->sensordata[quaternion_address + 1],
        data_->sensordata[quaternion_address + 2],
        data_->sensordata[quaternion_address + 3],
        data_->sensordata[quaternion_address]};
    for (std::size_t i = 0; i < 3; ++i) {
      state.base_angular_velocity[i] = data_->sensordata[gyro_address + i];
    }
    return state;
  }

  void write_effort(const rc::JointArray& effort) {
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      data_->ctrl[actuator_id_[i]] = effort[i];
    }
  }

  std::array<double, rc::kJointCount> control_snapshot() const {
    std::array<double, rc::kJointCount> result{};
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      result[i] = data_->ctrl[actuator_id_[i]];
    }
    return result;
  }

  void step() { mj_step(model_.get(), data_.get()); }

  void reset() {
    mj_resetData(model_.get(), data_.get());
    mj_forward(model_.get(), data_.get());
    rc::JointArray zero{};
    write_effort(zero);
  }

  double time() const { return data_->time; }
  mjModel* model() const { return model_.get(); }
  mjData* data() const { return data_.get(); }

 private:
  struct ModelDeleter {
    void operator()(mjModel* model) const { mj_deleteModel(model); }
  };
  struct DataDeleter {
    void operator()(mjData* data) const { mj_deleteData(data); }
  };

  void map_and_validate() {
    const auto& joint_names = rc::canonical_joint_order();
    rc::validate_joint_order(
        std::vector<std::string>(joint_names.begin(), joint_names.end()));
    if (model_->nu != static_cast<int>(rc::kJointCount)) {
      throw std::runtime_error("MuJoCo model does not expose exactly six actuators");
    }

    RCLCPP_INFO(logger_, "controller_index expected_joint actuator_id actuator_name "
                         "joint_id qpos_adr dof_adr ctrl_min ctrl_max");
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      const int actuator = mj_name2id(model_.get(), mjOBJ_ACTUATOR, kActuatorNames[i]);
      if (actuator < 0) {
        throw std::runtime_error("missing actuator " + std::string(kActuatorNames[i]));
      }
      const int joint = model_->actuator_trnid[2 * actuator];
      const char* actual_joint = joint >= 0 ? mj_id2name(model_.get(), mjOBJ_JOINT, joint) : nullptr;
      if (!actual_joint || joint_names[i] != actual_joint) {
        throw std::runtime_error("actuator-to-joint mapping mismatch at index " +
                                 std::to_string(i));
      }
      if (!model_->actuator_ctrllimited[actuator] ||
          model_->actuator_ctrlrange[2 * actuator] != -90.0 ||
          model_->actuator_ctrlrange[2 * actuator + 1] != 90.0) {
        throw std::runtime_error("actuator ctrl range mismatch at index " +
                                 std::to_string(i));
      }
      actuator_id_[i] = actuator;
      joint_id_[i] = joint;
      qpos_address_[i] = model_->jnt_qposadr[joint];
      dof_address_[i] = model_->jnt_dofadr[joint];
      RCLCPP_INFO(logger_, "%zu %s %d %s %d %d %d %.1f %.1f", i,
                  joint_names[i].c_str(), actuator, kActuatorNames[i], joint,
                  qpos_address_[i], dof_address_[i],
                  model_->actuator_ctrlrange[2 * actuator],
                  model_->actuator_ctrlrange[2 * actuator + 1]);
    }

    quaternion_sensor_id_ = mj_name2id(model_.get(), mjOBJ_SENSOR, "imu_quat");
    gyro_sensor_id_ = mj_name2id(model_.get(), mjOBJ_SENSOR, "angular_velocity");
    if (quaternion_sensor_id_ < 0 || gyro_sensor_id_ < 0 ||
        model_->sensor_dim[quaternion_sensor_id_] != 4 ||
        model_->sensor_dim[gyro_sensor_id_] != 3) {
      throw std::runtime_error("canonical IMU sensor mapping failed");
    }
    RCLCPP_INFO(logger_, "sensor_mapping quaternion=imu_quat[%d] angular_velocity[%d]",
                model_->sensor_adr[quaternion_sensor_id_],
                model_->sensor_adr[gyro_sensor_id_]);
    RCLCPP_INFO(logger_, "mapping_status=PASS timestep=%.3f", model_->opt.timestep);
  }

  rclcpp::Logger logger_;
  std::unique_ptr<mjModel, ModelDeleter> model_;
  std::unique_ptr<mjData, DataDeleter> data_;
  std::array<int, rc::kJointCount> actuator_id_{};
  std::array<int, rc::kJointCount> joint_id_{};
  std::array<int, rc::kJointCount> qpos_address_{};
  std::array<int, rc::kJointCount> dof_address_{};
  int quaternion_sensor_id_{-1};
  int gyro_sensor_id_{-1};
};

#ifdef RUBI_MUJOCO_HAS_GLFW
class MujocoRenderer {
 public:
  MujocoRenderer(mjModel* model, mjData* data, const rclcpp::Logger& logger)
      : model_(model), data_(data), logger_(logger) {
    if (!model_ || !data_) {
      throw std::invalid_argument("MuJoCo renderer received null model/data");
    }
    if (!glfwInit()) {
      throw std::runtime_error("glfwInit failed");
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
    window_ = glfwCreateWindow(1280, 720, "RUBI MuJoCo ROS 2", nullptr, nullptr);
    if (!window_) {
      glfwTerminate();
      throw std::runtime_error("glfwCreateWindow failed");
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);

    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    mjv_makeScene(model_, &scene_, 2000);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);
    camera_.type = mjCAMERA_FREE;
    camera_.lookat[0] = 0.0;
    camera_.lookat[1] = 0.0;
    camera_.lookat[2] = 0.55;
    camera_.distance = 2.2;
    camera_.azimuth = 135.0;
    camera_.elevation = -18.0;

    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, &MujocoRenderer::key_callback);
    glfwSetMouseButtonCallback(window_, &MujocoRenderer::mouse_button_callback);
    glfwSetCursorPosCallback(window_, &MujocoRenderer::cursor_position_callback);
    glfwSetScrollCallback(window_, &MujocoRenderer::scroll_callback);
    RCLCPP_INFO(logger_,
                "MUJOCO_GUI_READY=PASS window=1280x720 renderer=MuJoCo/GLFW "
                "mjdata_owner=physics_thread camera=orbit_pan_zoom");
  }

  ~MujocoRenderer() {
    if (window_) {
      glfwMakeContextCurrent(window_);
      mjr_freeContext(&context_);
      mjv_freeScene(&scene_);
      glfwDestroyWindow(window_);
      window_ = nullptr;
    }
    glfwTerminate();
  }

  MujocoRenderer(const MujocoRenderer&) = delete;
  MujocoRenderer& operator=(const MujocoRenderer&) = delete;

  void render() {
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    const mjrRect viewport{0, 0, width, height};
    mjv_updateScene(model_, data_, &option_, nullptr, &camera_, mjCAT_ALL,
                    &scene_);
    mjr_render(viewport, &scene_, &context_);
    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  void capture_png(const std::string& path) {
#ifdef RUBI_MUJOCO_HAS_PNG
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    if (width <= 0 || height <= 0) {
      throw std::runtime_error("cannot capture an empty GLFW framebuffer");
    }
    const mjrRect viewport{0, 0, width, height};
    std::vector<unsigned char> bottom_up(
        static_cast<std::size_t>(width * height * 3));
    std::vector<unsigned char> top_down(bottom_up.size());
    mjr_readPixels(bottom_up.data(), nullptr, viewport, &context_);
    const std::size_t row_bytes = static_cast<std::size_t>(width * 3);
    for (int row = 0; row < height; ++row) {
      std::copy_n(bottom_up.data() +
                      static_cast<std::size_t>(height - 1 - row) * row_bytes,
                  row_bytes,
                  top_down.data() + static_cast<std::size_t>(row) * row_bytes);
    }
    png_image image{};
    image.version = PNG_IMAGE_VERSION;
    image.width = static_cast<png_uint_32>(width);
    image.height = static_cast<png_uint_32>(height);
    image.format = PNG_FORMAT_RGB;
    if (!png_image_write_to_file(&image, path.c_str(), 0, top_down.data(), 0,
                                 nullptr)) {
      throw std::runtime_error("PNG framebuffer write failed: " +
                               std::string(image.message));
    }
    RCLCPP_INFO(logger_,
                "MUJOCO_GUI_SCREENSHOT=PASS path=%s width=%d height=%d",
                path.c_str(), width, height);
#else
    (void)path;
    throw std::runtime_error(
        "screenshot_path requested but libpng was unavailable at build time");
#endif
  }

  bool should_close() const { return glfwWindowShouldClose(window_); }
  bool paused() const { return paused_; }

  bool consume_reset() { return consume(reset_requested_); }
  bool consume_walk_ready() { return consume(walk_ready_requested_); }
  bool consume_policy_on() { return consume(policy_on_requested_); }
  bool consume_torque_off() { return consume(torque_off_requested_); }
  bool consume_emergency_stop() { return consume(emergency_stop_requested_); }

 private:
  static bool consume(bool& value) {
    const bool result = value;
    value = false;
    return result;
  }

  static MujocoRenderer* instance(GLFWwindow* window) {
    return static_cast<MujocoRenderer*>(glfwGetWindowUserPointer(window));
  }

  static void key_callback(GLFWwindow* window, int key, int, int action, int) {
    if (action != GLFW_PRESS) {
      return;
    }
    auto* self = instance(window);
    if (!self) {
      return;
    }
    switch (key) {
      case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(window, GLFW_TRUE);
        break;
      case GLFW_KEY_SPACE:
        self->paused_ = !self->paused_;
        break;
      case GLFW_KEY_R:
        self->reset_requested_ = true;
        break;
      case GLFW_KEY_W:
        self->walk_ready_requested_ = true;
        break;
      case GLFW_KEY_P:
        self->policy_on_requested_ = true;
        break;
      case GLFW_KEY_T:
        self->torque_off_requested_ = true;
        break;
      case GLFW_KEY_E:
        self->emergency_stop_requested_ = true;
        break;
      default:
        break;
    }
  }

  static void mouse_button_callback(GLFWwindow* window, int, int, int) {
    auto* self = instance(window);
    if (self) {
      glfwGetCursorPos(window, &self->last_x_, &self->last_y_);
    }
  }

  static void cursor_position_callback(GLFWwindow* window, double x, double y) {
    auto* self = instance(window);
    if (!self) {
      return;
    }
    if (!glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) &&
        !glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) &&
        !glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)) {
      self->last_x_ = x;
      self->last_y_ = y;
      return;
    }
    int width = 0;
    int height = 0;
    glfwGetWindowSize(window, &width, &height);
    const double dx = (x - self->last_x_) / std::max(1, height);
    const double dy = (y - self->last_y_) / std::max(1, height);
    self->last_x_ = x;
    self->last_y_ = y;
    const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                       glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    mjtMouse action = mjMOUSE_ZOOM;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)) {
      action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)) {
      action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)) {
      action = mjMOUSE_MOVE_H;
    }
    mjv_moveCamera(self->model_, action, dx, dy, &self->scene_,
                   &self->camera_);
  }

  static void scroll_callback(GLFWwindow* window, double, double y_offset) {
    auto* self = instance(window);
    if (self) {
      mjv_moveCamera(self->model_, mjMOUSE_ZOOM, 0.0, -0.05 * y_offset,
                     &self->scene_, &self->camera_);
    }
  }

  mjModel* model_{nullptr};
  mjData* data_{nullptr};
  rclcpp::Logger logger_;
  GLFWwindow* window_{nullptr};
  mjvCamera camera_{};
  mjvOption option_{};
  mjvScene scene_{};
  mjrContext context_{};
  double last_x_{0.0};
  double last_y_{0.0};
  bool paused_{false};
  bool reset_requested_{false};
  bool walk_ready_requested_{false};
  bool policy_on_requested_{false};
  bool torque_off_requested_{false};
  bool emergency_stop_requested_{false};
};
#endif

class RubiMujocoNode final : public rclcpp::Node {
 public:
  RubiMujocoNode() : Node("controller") {
    const auto sim_share = ament_index_cpp::get_package_share_directory("rubi_mujoco_sim");
    const auto core_share = ament_index_cpp::get_package_share_directory("rubi_control_core");
    model_path_ = declare_parameter("model", sim_share + "/models/rubi.xml");
    encoder_path_ = declare_parameter("encoder", core_share + "/models/encoder.onnx");
    policy_path_ = declare_parameter("policy", core_share + "/models/policy.onnx");
    run_fast_ = declare_parameter("run_as_fast_as_possible", false);
    real_time_factor_ = declare_parameter("real_time_factor", 1.0);
    self_test_ = declare_parameter("self_test", false);
    gui_ = declare_parameter("gui", false);
    auto_demo_ = declare_parameter("auto_demo", false);
    screenshot_path_ = declare_parameter("screenshot_path", "");
    command_timeout_ = declare_parameter("command_timeout", 0.5);
#ifndef RUBI_MUJOCO_HAS_GLFW
    if (gui_) {
      throw std::runtime_error(
          "gui=true is unavailable: this build did not find GLFW3");
    }
#endif
    if (real_time_factor_ <= 0.0 || command_timeout_ < 0.0) {
      throw std::invalid_argument("real_time_factor and command_timeout parameters are invalid");
    }

    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr message) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command_.linear_x = message->linear.x;
          command_.linear_y = message->linear.y;
          command_.angular_z = message->angular.z;
          ++command_.command_sequence;
        });
    clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    action_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("action", 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>("status", 10);

    walk_ready_service_ = mode_service("~/walk_ready", rc::ControllerMode::kWalkReady, false);
    policy_on_service_ = mode_service("~/policy_on", rc::ControllerMode::kPolicyOn, false);
    torque_off_service_ = mode_service("~/torque_off", rc::ControllerMode::kTorqueOff, false);
    emergency_service_ = mode_service("~/emergency_stop",
                                      rc::ControllerMode::kEmergencyStop, true);
    reset_service_ = create_service<Trigger>(
        "~/reset", [this](const Trigger::Request::SharedPtr,
                           Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command_.reset = true;
          command_.emergency_stop = false;
          ++command_.reset_sequence;
          response->success = true;
          response->message = "reset queued for next physics tick";
        });

    worker_ = std::thread([this] { physics_main(); });
  }

  ~RubiMujocoNode() override {
    stop_.store(true);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  rclcpp::Service<Trigger>::SharedPtr mode_service(
      const std::string& name, rc::ControllerMode mode, bool emergency) {
    return create_service<Trigger>(
        name, [this, mode, emergency](const Trigger::Request::SharedPtr,
                                     Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command_.requested_mode = mode;
          command_.emergency_stop = emergency;
          ++command_.mode_sequence;
          response->success = true;
          response->message = "mode queued for next physics tick";
        });
  }

  rc::UserCommand command_snapshot() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return command_;
  }

  void clear_consumed_reset(std::uint64_t reset_sequence) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_.reset_sequence == reset_sequence) {
      command_.reset = false;
    }
  }

#ifdef RUBI_MUJOCO_HAS_GLFW
  void apply_gui_requests(MujocoRenderer& renderer) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (renderer.consume_reset()) {
      command_.reset = true;
      command_.emergency_stop = false;
      ++command_.reset_sequence;
    }
    if (renderer.consume_walk_ready()) {
      command_.requested_mode = rc::ControllerMode::kWalkReady;
      command_.emergency_stop = false;
      ++command_.mode_sequence;
    }
    if (renderer.consume_policy_on()) {
      command_.requested_mode = rc::ControllerMode::kPolicyOn;
      command_.emergency_stop = false;
      ++command_.mode_sequence;
    }
    if (renderer.consume_torque_off()) {
      command_.requested_mode = rc::ControllerMode::kTorqueOff;
      command_.emergency_stop = false;
      ++command_.mode_sequence;
    }
    if (renderer.consume_emergency_stop()) {
      command_.requested_mode = rc::ControllerMode::kEmergencyStop;
      command_.emergency_stop = true;
      ++command_.mode_sequence;
    }
  }
#endif

  void publish_state(const rc::RobotState& state, const rc::ControllerOutput& output) {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = rclcpp::Time(static_cast<std::int64_t>(state.sim_time_sec * 1.0e9),
                               RCL_ROS_TIME);
    clock_publisher_->publish(clock);

    sensor_msgs::msg::JointState joints;
    joints.header.stamp = clock.clock;
    const auto& names = rc::canonical_joint_order();
    joints.name.assign(names.begin(), names.end());
    joints.position.assign(state.joint_position.begin(), state.joint_position.end());
    joints.velocity.assign(state.joint_velocity.begin(), state.joint_velocity.end());
    joints.effort.assign(state.measured_effort.begin(), state.measured_effort.end());
    joint_state_publisher_->publish(joints);

    std_msgs::msg::Float64MultiArray action;
    action.data.assign(output.action.begin(), output.action.end());
    action_publisher_->publish(action);

    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << "mode=" << static_cast<int>(output.mode)
           << " inference=" << (output.inference_ran ? 1 : 0)
           << " safe_zero=" << (output.safe_zero ? 1 : 0)
           << " fault=" << (output.faulted ? output.fault_reason : "none");
    status.data = stream.str();
    status_publisher_->publish(status);
  }

  rc::ControllerOutput run_tick(PlantAdapter& plant, rc::Controller& controller,
                                const rc::UserCommand& command, bool publish = true) {
    const bool reset_tick = command.reset;
    if (reset_tick) {
      plant.reset();
    }
    const auto state = plant.read_state();
    const auto output = controller.update(state, command);
    plant.write_effort(output.effort);
    if (publish && controller.physics_tick_count() % 5 == 0) {
      publish_state(state, output);
    }
    if (!reset_tick) {
      plant.step();
    }
    return output;
  }

  void require(bool condition, const std::string& message) {
    if (!condition) {
      throw std::runtime_error("self-test: " + message);
    }
  }

  void run_self_test(PlantAdapter& plant, rc::Controller& controller) {
    rc::UserCommand command;
    rc::ControllerOutput output;

    for (int i = 0; i < 250; ++i) {
      output = run_tick(plant, controller, command);
    }
    require(exact_zero(plant.control_snapshot()), "M1 ctrl is not exact zero");
    require(controller.inference_count() == 0, "M1 inference ran in torque-off");
    require(all_finite(plant.read_state().joint_position), "M1 state is nonfinite");
    RCLCPP_INFO(get_logger(), "M1 PASS startup_torque_off ticks=250 inference=0 ctrl_zero=true");

    command.requested_mode = rc::ControllerMode::kWalkReady;
    ++command.mode_sequence;
    for (int i = 0; i < 300; ++i) {
      output = run_tick(plant, controller, command);
    }
    const auto default_pose = rc::PolicyContract{}.default_pose;
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      require(std::abs(output.target_position[i] - default_pose[i]) < 1.0e-9,
              "M2 walk-ready target mismatch");
      require(std::isfinite(output.effort[i]) && std::abs(output.effort[i]) <= 90.0,
              "M2 effort invalid");
    }
    RCLCPP_INFO(get_logger(), "M2 PASS walk_ready duration=0.6 target_default=true effort_clamped=true");

    command.requested_mode = rc::ControllerMode::kPolicyOn;
    ++command.mode_sequence;
    const auto inference_before = controller.inference_count();
    for (int i = 0; i < 500; ++i) {
      output = run_tick(plant, controller, command);
      require(all_finite(output.action) && all_finite(output.effort),
              "M3 nonfinite action or effort");
    }
    require(controller.inference_count() - inference_before == 100,
            "M3 decimation is not nominal 100 Hz");
    require(controller.last_observation().size() == 30 && controller.history().size() == 300 &&
                controller.last_latent().size() == 3 &&
                controller.last_policy_input().size() == 36 && output.action.size() == 6,
            "M3 controller dimensions mismatch");
    RCLCPP_INFO(get_logger(), "M3 PASS policy_zero_command ticks=500 inference=100 dims=30/300/3/36/6 finite=true");

    command.linear_x = 0.1;
    for (int i = 0; i < 500; ++i) {
      if (i % 50 == 0) {
        ++command.command_sequence;
      }
      output = run_tick(plant, controller, command);
    }
    require(std::abs(controller.last_policy_input()[33] - 0.4F) < 1.0e-6,
            "M4 forward command scale is not 0.4");
    command.linear_x = 0.0;
    ++command.command_sequence;
    for (int i = 0; i < 5; ++i) {
      output = run_tick(plant, controller, command);
    }
    require(controller.last_policy_input()[33] == 0.0F,
            "M4 zero command was not applied");
    RCLCPP_INFO(get_logger(), "M4 PASS cmd_x=0.1 publish_rate=10Hz scaled=0.4 then_zero=true");

    const auto before_emergency = plant.control_snapshot();
    require(!exact_zero(before_emergency), "M5 pre-E-stop ctrl is zero");
    command.emergency_stop = true;
    const auto estop_tick = controller.physics_tick_count();
    output = run_tick(plant, controller, command);
    require(exact_zero(plant.control_snapshot()) && exact_zero(output.effort),
            "M5 E-stop did not same-tick overwrite six ctrl slots");
    for (int i = 0; i < 20; ++i) {
      output = run_tick(plant, controller, command);
      require(exact_zero(plant.control_snapshot()), "M5 E-stop zero was not held");
    }
    RCLCPP_INFO(get_logger(), "M5 PASS estop_tick=%llu pre_ctrl_nonzero=true same_tick_zero=true held_zero=true",
                static_cast<unsigned long long>(estop_tick));

    command.emergency_stop = false;
    command.reset = true;
    ++command.reset_sequence;
    output = run_tick(plant, controller, command);
    require(plant.time() == 0.0, "M6 simulation clock did not reset to zero");
    require(exact_zero(controller.history()) && exact_zero(controller.held_action()) &&
                controller.gait_phase() == 0.0 && exact_zero(plant.control_snapshot()),
            "M6 controller reset state mismatch");
    require(controller.mode() == rc::ControllerMode::kTorqueOff,
            "M6 mode did not reset to torque-off");
    RCLCPP_INFO(get_logger(), "M6 PASS sim_time=0 history_zero=true action_zero=true phase=0 command_cleared=true");
    RCLCPP_INFO(get_logger(), "MUJOCO_CONTROLLER_SELF_TEST=PASS call_order=read_state/core_update/write_six_ctrl/mj_step");
  }

  void physics_main() {
    try {
      PlantAdapter plant(model_path_, get_logger());
      auto policy = std::make_shared<rc::PolicyRunner>(encoder_path_, policy_path_);
      rc::PolicyContract contract;
      contract.command_timeout = command_timeout_;
      rc::Controller controller(policy, contract);
      RCLCPP_INFO(get_logger(), "policy_load=PASS encoder=%s policy=%s startup_mode=torque_off",
                  encoder_path_.c_str(), policy_path_.c_str());

#ifdef RUBI_MUJOCO_HAS_GLFW
      std::unique_ptr<MujocoRenderer> renderer;
      if (gui_) {
        renderer = std::make_unique<MujocoRenderer>(
            plant.model(), plant.data(), get_logger());
        renderer->render();
      }
#endif

      if (self_test_) {
        run_self_test(plant, controller);
        rclcpp::shutdown();
        return;
      }

      const auto wall_period = std::chrono::duration<double>(
          contract.physics_dt / real_time_factor_);
      auto next_tick = std::chrono::steady_clock::now();
      bool auto_policy_requested = false;
      bool screenshot_captured = false;
      if (auto_demo_) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_.requested_mode = rc::ControllerMode::kWalkReady;
        ++command_.mode_sequence;
        RCLCPP_INFO(get_logger(),
                    "MUJOCO_AUTO_DEMO walk_ready_requested=true");
      }
      while (rclcpp::ok() && !stop_.load()) {
#ifdef RUBI_MUJOCO_HAS_GLFW
        if (renderer) {
          apply_gui_requests(*renderer);
          if (renderer->should_close()) {
            RCLCPP_INFO(get_logger(), "MUJOCO_GUI_CLOSE clean_shutdown=true");
            break;
          }
        }
#endif
        if (auto_demo_ && !auto_policy_requested &&
            controller.physics_tick_count() >= 300) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command_.requested_mode = rc::ControllerMode::kPolicyOn;
          ++command_.mode_sequence;
          auto_policy_requested = true;
          RCLCPP_INFO(get_logger(),
                      "MUJOCO_AUTO_DEMO policy_on_requested=true");
        }
        const auto command = command_snapshot();
        bool advance_physics = true;
#ifdef RUBI_MUJOCO_HAS_GLFW
        advance_physics = !renderer || !renderer->paused();
#endif
        if (advance_physics) {
          run_tick(plant, controller, command);
          if (command.reset) {
            clear_consumed_reset(command.reset_sequence);
          }
          if (gui_ && controller.physics_tick_count() % 500 == 0) {
            const auto state = plant.read_state();
            RCLCPP_INFO(get_logger(),
                        "MUJOCO_GUI_RUNTIME sim_time=%.3f physics_ticks=%llu "
                        "inference_count=%llu mode=%d finite=%s",
                        state.sim_time_sec,
                        static_cast<unsigned long long>(
                            controller.physics_tick_count()),
                        static_cast<unsigned long long>(
                            controller.inference_count()),
                        static_cast<int>(controller.mode()),
                        all_finite(state.joint_position) &&
                                all_finite(controller.held_action())
                            ? "true"
                            : "false");
          }
        }
#ifdef RUBI_MUJOCO_HAS_GLFW
        if (renderer &&
            (!advance_physics || controller.physics_tick_count() % 8 == 0)) {
          renderer->render();
        }
        if (renderer && !screenshot_captured && !screenshot_path_.empty() &&
            controller.inference_count() >= 50) {
          renderer->render();
          renderer->capture_png(screenshot_path_);
          screenshot_captured = true;
        }
#endif
        if (!run_fast_) {
          next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(wall_period);
          std::this_thread::sleep_until(next_tick);
        }
      }
      rc::JointArray zero{};
      plant.write_effort(zero);
#ifdef RUBI_MUJOCO_HAS_GLFW
      if (renderer && renderer->should_close() && rclcpp::ok()) {
        rclcpp::shutdown();
      }
#endif
    } catch (const std::exception& error) {
      RCLCPP_FATAL(get_logger(), "MUJOCO_CONTROLLER_RUNTIME=FAIL reason=%s", error.what());
      rclcpp::shutdown();
    }
  }

  std::string model_path_;
  std::string encoder_path_;
  std::string policy_path_;
  std::string screenshot_path_;
  bool run_fast_{false};
  bool self_test_{false};
  bool gui_{false};
  bool auto_demo_{false};
  double real_time_factor_{1.0};
  double command_timeout_{0.5};
  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::mutex command_mutex_;
  rc::UserCommand command_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr action_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<Trigger>::SharedPtr walk_ready_service_;
  rclcpp::Service<Trigger>::SharedPtr policy_on_service_;
  rclcpp::Service<Trigger>::SharedPtr torque_off_service_;
  rclcpp::Service<Trigger>::SharedPtr emergency_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RubiMujocoNode>();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    std::cerr << "MUJOCO_CONTROLLER_STARTUP=FAIL reason=" << error.what() << '\n';
    rclcpp::shutdown();
    return 2;
  }
  return 0;
}
