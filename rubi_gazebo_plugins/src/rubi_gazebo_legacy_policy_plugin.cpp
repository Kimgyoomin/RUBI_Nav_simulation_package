#include "rubi_gazebo_plugins/rubi_gazebo_legacy_policy_plugin.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rubi_control_core/gazebo_legacy_policy_adapter.hpp"
#include "rubi_control_core/gazebo_legacy_policy_runner.hpp"
#include "rubi_control_core/policy_contract.hpp"
#include "rubi_gazebo_plugins/velocity_input_arbitration.hpp"

namespace rubi_gazebo_plugins {
namespace {

namespace rc = rubi_control_core;
using Trigger = std_srvs::srv::Trigger;

constexpr std::size_t kPolicyOnButton = 8;
constexpr std::size_t kTorqueOffButton = 9;
constexpr std::size_t kWalkReadyButton = 10;
constexpr std::size_t kVelocitySourceToggleButton = 3;
constexpr std::size_t kSerialButtonCount = 13;
constexpr double kJoystickLinearXLimit = 2.0;
constexpr double kJoystickLinearYLimit = 0.75;
constexpr double kJoystickAngularZLimit = 1.5;

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

template <typename Container>
double maximum_absolute(const Container& values) {
  double maximum = 0.0;
  for (const auto value : values) {
    maximum = std::max(maximum, std::abs(static_cast<double>(value)));
  }
  return maximum;
}

std::string environment_or(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  return value && value[0] != '\0' ? value : fallback;
}

bool environment_flag(const char* name, bool fallback) {
  const auto value = environment_or(name, fallback ? "true" : "false");
  return value == "1" || value == "true" || value == "TRUE";
}

const char* velocity_input_source_name(VelocityInputSource source) {
  return source == VelocityInputSource::kJoystick ? "JOYSTICK" : "NAV2";
}

VelocityCommand clamp_nav_command(const VelocityCommand& command) {
  return {
      std::clamp(command.linear_x, -kJoystickLinearXLimit,
                 kJoystickLinearXLimit),
      std::clamp(command.linear_y, -kJoystickLinearYLimit,
                 kJoystickLinearYLimit),
      std::clamp(command.angular_z, -kJoystickAngularZLimit,
                 kJoystickAngularZLimit),
  };
}

}  // namespace

class RubiGazeboLegacyPolicyPlugin::Impl {
 public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
    model_ = std::move(model);
    world_ = model_ ? model_->GetWorld() : nullptr;
    if (!model_ || !world_) {
      throw std::runtime_error("Gazebo legacy model/world pointer is null");
    }
    configure_ros_namespace(sdf);
    node_ = gazebo_ros::Node::Get(sdf);
    if (!node_) {
      throw std::runtime_error("Gazebo legacy ROS node creation failed");
    }
    const auto core_share =
        ament_index_cpp::get_package_share_directory("rubi_control_core");
    policy_path_ = environment_or(
        "RUBI_GAZEBO_LEGACY_POLICY",
        core_share + "/models/rubi_gazebo_legacy_policy.onnx");
    self_test_ = environment_flag("RUBI_GAZEBO_LEGACY_SELF_TEST", false);
    policy_test_ticks_ = std::stoi(environment_or(
        "RUBI_GAZEBO_LEGACY_POLICY_TEST_TICKS", "500"));
    if (policy_test_ticks_ < 500 || policy_test_ticks_ % 5 != 0) {
      throw std::runtime_error(
          "legacy policy test ticks must be a multiple of 5 and at least 500");
    }
    if (std::abs(world_->Physics()->GetMaxStepSize() - 0.002) > 1.0e-12) {
      throw std::runtime_error("Gazebo max_step_size is not legacy 0.002 s");
    }
    map_joints();
    if (!self_test_) {
      command_.requested_mode = rc::ControllerMode::kWalkReady;
      ++command_.mode_sequence;
    }
    configure_ros_interfaces();
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&Impl::OnUpdate, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(),
                "LEGACY_PLUGIN_LOAD=PASS policy_contract=32x10_to_7 "
                "controller_thread=world_update state_order=held_previous_state "
                "startup_mode=%s joy_topic=/joy input_source=JOYSTICK self_test=%s",
                self_test_ ? "torque_off" : "walk_ready",
                self_test_ ? "true" : "false");
  }

 private:
  void configure_ros_namespace(const sdf::ElementPtr& sdf) {
    const auto requested = environment_or(
        "RUBI_GAZEBO_LEGACY_NAMESPACE", "/rubi_gazebo_legacy");
    if (!sdf->HasElement("ros")) {
      sdf->AddElement("ros");
    }
    auto ros = sdf->GetElement("ros");
    if (!ros->HasElement("namespace")) {
      ros->AddElement("namespace");
    }
    ros->GetElement("namespace")->Set(requested);
  }

  void map_joints() {
    const auto& names = rc::canonical_joint_order();
    rc::validate_joint_order(
        std::vector<std::string>(names.begin(), names.end()));
    for (std::size_t i = 0; i < names.size(); ++i) {
      joints_[i] = model_->GetJoint(names[i]);
      if (!joints_[i] || joints_[i]->DOF() != 1) {
        throw std::runtime_error(
            "missing or non-single-DoF legacy joint " + names[i]);
      }
      const auto axis = joints_[i]->LocalAxis(0);
      const ignition::math::Vector3d expected =
          (i == 0 || i == 3) ? ignition::math::Vector3d::UnitX
                             : ignition::math::Vector3d::UnitY;
      if (!axis.Equal(expected, 1.0e-12) ||
          std::abs(joints_[i]->GetEffortLimit(0) - 90.0) > 1.0e-12) {
        throw std::runtime_error(
            "legacy joint axis/effort mismatch for " + names[i]);
      }
      RCLCPP_INFO(node_->get_logger(),
                  "legacy_mapping index=%zu joint=%s dof=1 axis=[%.0f,%.0f,%.0f] "
                  "effort_limit=90 api_axis=0",
                  i, names[i].c_str(), axis.X(), axis.Y(), axis.Z());
    }
    RCLCPP_INFO(node_->get_logger(),
                "LEGACY_MAPPING=PASS correction=LEGACY_AXIS_1_BUG_CORRECTION "
                "read_write_axis=0");
  }

  bool resolve_imu() {
    if (imu_) {
      return true;
    }
    auto body = model_->GetLink("BODY");
    if (!body || body->GetSensorCount() == 0) {
      return false;
    }
    auto sensor = gazebo::sensors::SensorManager::Instance()->GetSensor(
        body->GetSensorName(0));
    imu_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensor);
    if (!imu_ ||
        imu_->ParentName().find(model_->GetScopedName() + "::BODY") ==
            std::string::npos) {
      throw std::runtime_error("legacy IMU mapping failed");
    }
    imu_->SetActive(true);
    RCLCPP_INFO(node_->get_logger(),
                "LEGACY_IMU_MAPPING=PASS name=%s parent=%s "
                "angular_velocity_api=false",
                imu_->ScopedName().c_str(), imu_->ParentName().c_str());
    return true;
  }

  void configure_ros_interfaces() {
    nav_cmd_timeout_sec_ = node_->declare_parameter<double>(
        "nav_cmd_timeout_sec", 0.5);
    if (!std::isfinite(nav_cmd_timeout_sec_) || nav_cmd_timeout_sec_ <= 0.0) {
      throw std::runtime_error("nav_cmd_timeout_sec must be finite and positive");
    }
    command_subscription_ =
        node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr message) {
              std::lock_guard<std::mutex> lock(command_mutex_);
              const bool valid = velocity_input_.cache_nav_command(
                  {message->linear.x, message->linear.y, message->angular.z},
                  VelocityInputArbitrator::SteadyClock::now());
              if (velocity_input_.source() == VelocityInputSource::kNav2) {
                ++velocity_command_sequence_;
              }
              if (!valid) {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 5000,
                    "NAV2_CMD_VEL_INVALID=zero_selected fields=linear.x,linear.y,angular.z");
              }
            });
    joy_subscription_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr message) {
          if (message->axes.size() <= 3) {
            RCLCPP_ERROR(node_->get_logger(),
                         "legacy joy rejected: at least four axes required");
            return;
          }
          std::lock_guard<std::mutex> lock(command_mutex_);
          bool torque_off_edge = false;
          // The ROS 1 source enables mode buttons only for its 13-button
          // serial joystick layout. Other layouts still provide motion axes.
          if (message->buttons.size() == kSerialButtonCount) {
            const auto rising_edge = [this, &message](std::size_t index) {
              return message->buttons[index] != 0 &&
                     !previous_serial_buttons_[index];
            };
            if (rising_edge(kPolicyOnButton)) {
              command_.requested_mode = rc::ControllerMode::kPolicyOn;
              command_.emergency_stop = false;
              ++command_.mode_sequence;
              RCLCPP_INFO(node_->get_logger(),
                          "LEGACY_JOY_EDGE button=8 mode=policy_on");
            } else if (rising_edge(kTorqueOffButton)) {
              command_.requested_mode = rc::ControllerMode::kTorqueOff;
              command_.emergency_stop = false;
              ++command_.mode_sequence;
              torque_off_edge = true;
              RCLCPP_INFO(node_->get_logger(),
                          "LEGACY_JOY_EDGE button=9 mode=torque_off");
            } else if (rising_edge(kWalkReadyButton)) {
              command_.requested_mode = rc::ControllerMode::kWalkReady;
              command_.emergency_stop = false;
              ++command_.mode_sequence;
              RCLCPP_INFO(node_->get_logger(),
                          "LEGACY_JOY_EDGE button=10 mode=walk_ready");
            }
            for (std::size_t i = 0; i < kSerialButtonCount; ++i) {
              previous_serial_buttons_[i] = message->buttons[i] != 0;
            }
          } else {
            previous_serial_buttons_.fill(false);
          }
          const bool source_toggle_pressed =
              button_is_pressed(message->buttons, kVelocitySourceToggleButton);
          if (torque_off_edge) {
            reset_velocity_input_locked();
            velocity_input_.synchronize_source_toggle_button(
                source_toggle_pressed);
          } else if (velocity_input_.update_source_toggle_button(
                         source_toggle_pressed)) {
            ++velocity_command_sequence_;
            RCLCPP_INFO(node_->get_logger(), "INPUT_SOURCE: %s -> %s",
                        velocity_input_source_name(
                            velocity_input_.source() ==
                                    VelocityInputSource::kNav2
                                ? VelocityInputSource::kJoystick
                                : VelocityInputSource::kNav2),
                        velocity_input_source_name(velocity_input_.source()));
          }
          if (velocity_input_.source() == VelocityInputSource::kJoystick) {
            command_.linear_x = kJoystickLinearXLimit * message->axes[1];
            command_.linear_y = kJoystickLinearYLimit * message->axes[0];
            command_.angular_z = kJoystickAngularZLimit * message->axes[3];
            ++command_.command_sequence;
            ++velocity_command_sequence_;
          }
        });
    joint_state_publisher_ =
        node_->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);
    action_publisher_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "action", 10);
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        "status", 10);
    walk_ready_service_ =
        mode_service("~/walk_ready", rc::ControllerMode::kWalkReady, false);
    policy_on_service_ =
        mode_service("~/policy_on", rc::ControllerMode::kPolicyOn, false);
    torque_off_service_ =
        mode_service("~/torque_off", rc::ControllerMode::kTorqueOff, false);
    emergency_service_ = mode_service(
        "~/emergency_stop", rc::ControllerMode::kEmergencyStop, true);
    reset_service_ = node_->create_service<Trigger>(
        "~/reset", [this](const Trigger::Request::SharedPtr,
                           Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command_.reset = true;
          command_.emergency_stop = false;
          ++command_.reset_sequence;
          controller_fault_active_ = false;
          reset_velocity_input_locked();
          response->success = true;
          response->message = "legacy reset queued";
        });
  }

  rclcpp::Service<Trigger>::SharedPtr mode_service(
      const std::string& name, rc::ControllerMode mode, bool emergency) {
    return node_->create_service<Trigger>(
        name, [this, mode, emergency](const Trigger::Request::SharedPtr,
                                     Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command_.requested_mode = mode;
          command_.emergency_stop = emergency;
          ++command_.mode_sequence;
          if (mode == rc::ControllerMode::kTorqueOff || emergency) {
            reset_velocity_input_locked();
          }
          response->success = true;
          response->message = "legacy mode queued";
        });
  }

  void reset_velocity_input_locked() {
    velocity_input_.reset_to_joystick();
    ++velocity_command_sequence_;
  }

  rc::UserCommand snapshot_command(bool* nav_timeout_started) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    const bool controller_disabled =
        controller_fault_active_ || command_.reset || command_.emergency_stop ||
        command_.requested_mode == rc::ControllerMode::kTorqueOff ||
        command_.requested_mode == rc::ControllerMode::kEmergencyStop ||
        controller_->mode() == rc::ControllerMode::kTorqueOff ||
        controller_->mode() == rc::ControllerMode::kEmergencyStop;
    const auto decision = velocity_input_.select(
        {command_.linear_x, command_.linear_y, command_.angular_z},
        controller_disabled, VelocityInputArbitrator::SteadyClock::now(),
        std::chrono::duration<double>(nav_cmd_timeout_sec_));
    if (decision.nav_timeout_started) {
      ++velocity_command_sequence_;
    }
    const auto selected = velocity_input_.source() == VelocityInputSource::kNav2
                              ? clamp_nav_command(decision.command)
                              : decision.command;
    auto command = command_;
    command.linear_x = selected.linear_x;
    command.linear_y = selected.linear_y;
    command.angular_z = selected.angular_z;
    command.command_sequence = velocity_command_sequence_;
    *nav_timeout_started = decision.nav_timeout_started;
    return command;
  }

  void queue_source_policy_transition(const rc::UserCommand& consumed_command) {
    if (!controller_->walk_ready_complete() ||
        controller_->mode() != rc::ControllerMode::kWalkReady) {
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_.mode_sequence != consumed_command.mode_sequence ||
        command_.requested_mode != rc::ControllerMode::kWalkReady) {
      return;
    }
    command_.requested_mode = rc::ControllerMode::kPolicyOn;
    ++command_.mode_sequence;
    RCLCPP_INFO(node_->get_logger(),
                "LEGACY_SOURCE_TRANSITION walk_ready_complete=true "
                "next_mode=policy_on");
  }

  void read_joints_into_held_state(double sim_time) {
    held_state_.sim_time_sec = sim_time;
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      held_state_.joint_position[i] = joints_[i]->Position(0);
      held_state_.joint_velocity[i] = joints_[i]->GetVelocity(0);
      held_state_.measured_effort[i] = joints_[i]->GetForce(0);
    }
  }

  void read_imu_into_held_state() {
    const auto orientation = imu_->Orientation();
    const auto angular_velocity = imu_->AngularVelocity(false);
    held_state_.base_orientation_xyzw = {
        orientation.X(), orientation.Y(), orientation.Z(), orientation.W()};
    held_state_.base_angular_velocity = {
        angular_velocity.X(), angular_velocity.Y(), angular_velocity.Z()};
  }

  void write_effort(const rc::JointArray& effort) {
    commanded_effort_ = effort;
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      joints_[i]->SetForce(0, effort[i]);
    }
  }

  void initialize_controller() {
    auto policy = std::make_shared<rc::GazeboLegacyPolicyRunner>(policy_path_);
    controller_ =
        std::make_unique<rc::GazeboLegacyPolicyAdapter>(std::move(policy));
    RCLCPP_INFO(node_->get_logger(),
                "LEGACY_POLICY_LOAD=PASS path=%s shape=320_to_7 "
                "names=initial_obs/action_output dtype=float32",
                policy_path_.c_str());
  }

  void publish(const rc::RobotState& state,
               const rc::ControllerOutput& output) {
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = rclcpp::Time(
        static_cast<std::int64_t>(state.sim_time_sec * 1.0e9), RCL_ROS_TIME);
    const auto& names = rc::canonical_joint_order();
    joints.name.assign(names.begin(), names.end());
    joints.position.assign(state.joint_position.begin(),
                           state.joint_position.end());
    joints.velocity.assign(state.joint_velocity.begin(),
                           state.joint_velocity.end());
    joints.effort.assign(state.measured_effort.begin(),
                         state.measured_effort.end());
    joint_state_publisher_->publish(joints);
    std_msgs::msg::Float64MultiArray action;
    action.data.assign(output.action.begin(), output.action.end());
    action_publisher_->publish(action);
    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << "variant=legacy_policy mode=" << static_cast<int>(output.mode)
           << " inference_count=" << controller_->inference_count()
           << " cycle_period=" << controller_->cycle_period()
           << " fault=" << (output.faulted ? output.fault_reason : "none");
    status.data = stream.str();
    status_publisher_->publish(status);
  }

  void require(bool condition, const std::string& message) {
    if (!condition) {
      throw std::runtime_error("Gazebo legacy self-test: " + message);
    }
  }

  void advance_self_test(const rc::RobotState& state,
                         const rc::ControllerOutput& output) {
    ++stage_tick_;
    if (stage_ == 4 && output.faulted) {
      require(exact_zero(commanded_effort_),
              "nonfinite fault did not issue same-update six-zero");
      RCLCPP_ERROR(node_->get_logger(),
                   "LEGACY_G4_UNSUPPORTED=BLOCKED tick=%d inference=%llu "
                   "reason=%s safe_zero=true",
                   stage_tick_, static_cast<unsigned long long>(
                       controller_->inference_count()),
                   output.fault_reason.c_str());
      stage_ = 60;
      stage_tick_ = 0;
      test_command_.reset = true;
      ++test_command_.reset_sequence;
      return;
    }
    if (stage_ == 1 && stage_tick_ == 250) {
      require(exact_zero(commanded_effort_), "startup torque-off not zero");
      require(controller_->inference_count() == 0,
              "inference ran in startup torque-off");
      RCLCPP_INFO(node_->get_logger(),
                  "LEGACY_G1 PASS torque_off=0.5s six_SetForce0_zero=true");
      stage_ = 3;
      stage_tick_ = 0;
      test_command_.requested_mode = rc::ControllerMode::kWalkReady;
      ++test_command_.mode_sequence;
    } else if (stage_ == 3 && stage_tick_ == 300) {
      require(controller_->walk_ready_complete(),
              "walk-ready did not complete");
      require(all_finite(output.effort), "walk-ready effort nonfinite");
      RCLCPP_INFO(node_->get_logger(),
                  "LEGACY_G3 PASS walk_ready=0.6s source_pose=[0,.65,-1.3] "
                  "explicit_policy_enable=true");
      stage_ = 4;
      stage_tick_ = 0;
      inference_stage_start_ = controller_->inference_count();
      test_command_.requested_mode = rc::ControllerMode::kPolicyOn;
      ++test_command_.mode_sequence;
    } else if (stage_ == 4 && stage_tick_ == policy_test_ticks_) {
      const auto inference_delta =
          controller_->inference_count() - inference_stage_start_;
      require(inference_delta ==
                  static_cast<std::uint64_t>(policy_test_ticks_ / 5),
              "legacy inference decimation mismatch");
      require(all_finite(output.action) && all_finite(output.effort) &&
                  all_finite(state.joint_position) &&
                  all_finite(controller_->history()) &&
                  all_finite(controller_->last_network_output()),
              "legacy policy closed loop nonfinite");
      RCLCPP_INFO(node_->get_logger(),
                  "LEGACY_G4 PASS ticks=%d inference=%llu rate=100Hz "
                  "dims=32/320/7 action_max=%.9g history_max=%.9g "
                  "cycle_period=%.9g",
                  policy_test_ticks_, static_cast<unsigned long long>(
                      inference_delta),
                  maximum_absolute(output.action),
                  maximum_absolute(controller_->history()),
                  controller_->cycle_period());
      stage_ = 5;
      stage_tick_ = 0;
      test_command_.linear_x = 0.1;
      ++test_command_.command_sequence;
    } else if (stage_ == 5 && stage_tick_ == 500) {
      require(all_finite(output.action) && all_finite(output.effort),
              "legacy low command nonfinite");
      RCLCPP_INFO(node_->get_logger(),
                  "LEGACY_G5 PASS cmd_x=0.1 duration=1.0s finite=true");
      stage_ = 6;
      stage_tick_ = 0;
      test_command_.linear_x = 0.0;
      ++test_command_.command_sequence;
    } else if (stage_ == 6 && stage_tick_ == 5) {
      require(!exact_zero(commanded_effort_),
              "legacy pre-E-stop effort is zero");
      test_command_.emergency_stop = true;
    } else if (stage_ == 6 && stage_tick_ == 6) {
      require(exact_zero(commanded_effort_) && exact_zero(output.effort),
              "legacy same-update E-stop zero failed");
      RCLCPP_INFO(node_->get_logger(),
                  "LEGACY_G6 PASS same_update_six_SetForce_axis0_zero=true");
      stage_ = 7;
      stage_tick_ = 0;
      test_command_.emergency_stop = false;
      test_command_.reset = true;
      ++test_command_.reset_sequence;
    } else if (stage_ == 7 && stage_tick_ == 1) {
      require(exact_zero(controller_->history()) &&
                  exact_zero(controller_->held_action()) &&
                  controller_->cycle_time() == 0.0 &&
                  controller_->cycle_period() == 0.4 &&
                  controller_->mode() == rc::ControllerMode::kTorqueOff &&
                  exact_zero(commanded_effort_),
              "legacy reset state mismatch");
      RCLCPP_INFO(node_->get_logger(),
                  "LEGACY_G7 PASS reset_history_action_phase=true "
                  "mode=torque_off");
      RCLCPP_INFO(node_->get_logger(),
                  "RUBI_GAZEBO_LEGACY_SELF_TEST=PASS "
                  "stable_locomotion=NOT_TESTED");
      self_test_complete_ = true;
      world_->SetPaused(true);
    } else if (stage_ == 60 && stage_tick_ == 1) {
      require(exact_zero(controller_->history()) &&
                  exact_zero(controller_->held_action()) &&
                  exact_zero(commanded_effort_),
              "legacy blocked-path reset failed");
      stage_ = 61;
      stage_tick_ = 0;
      test_command_.reset = false;
      test_command_.requested_mode = rc::ControllerMode::kWalkReady;
      ++test_command_.mode_sequence;
    } else if (stage_ == 61 && stage_tick_ == 252) {
      test_command_.requested_mode = rc::ControllerMode::kPolicyOn;
      ++test_command_.mode_sequence;
      stage_ = 62;
      stage_tick_ = 0;
    } else if (stage_ == 62 && stage_tick_ == 5) {
      test_command_.emergency_stop = true;
    } else if (stage_ == 62 && stage_tick_ == 6) {
      require(exact_zero(commanded_effort_) && exact_zero(output.effort),
              "legacy blocked-path E-stop zero failed");
      RCLCPP_INFO(node_->get_logger(),
                  "LEGACY_G6 PASS after_blocked_G4 same_update_six_zero=true");
      test_command_.emergency_stop = false;
      test_command_.reset = true;
      ++test_command_.reset_sequence;
      stage_ = 63;
      stage_tick_ = 0;
    } else if (stage_ == 63 && stage_tick_ == 1) {
      require(exact_zero(controller_->history()) &&
                  exact_zero(controller_->held_action()) &&
                  exact_zero(commanded_effort_),
              "legacy blocked-path final reset failed");
      RCLCPP_ERROR(node_->get_logger(),
                   "RUBI_GAZEBO_LEGACY_SELF_TEST=BLOCKED "
                   "controller_nonfinite safety_G6_G7=PASS");
      self_test_complete_ = true;
      world_->SetPaused(true);
    }
  }

  void OnUpdate(const gazebo::common::UpdateInfo& info) {
    try {
      if (self_test_complete_) {
        return;
      }
      if (!resolve_imu()) {
        write_effort(rc::JointArray{});
        return;
      }
      if (!controller_) {
        initialize_controller();
      }
      const double sim_time = info.simTime.Double();
      bool nav_timeout_started = false;
      rc::UserCommand command = self_test_ ? test_command_
                                            : snapshot_command(&nav_timeout_started);
      if (nav_timeout_started) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "NAV2_CMD_VEL_TIMEOUT=zero_selected timeout_sec=%.3f",
            nav_cmd_timeout_sec_);
      }
      if (command.reset) {
        model_->Reset();
      }
      held_state_.sim_time_sec = sim_time;

      // Active ROS 1 order: inference/PD/write use held state, then encoders and
      // decimated IMU are sampled for the next update.
      const auto output = controller_->update(held_state_, command);
      write_effort(output.effort);
      if (!self_test_) {
        queue_source_policy_transition(command);
      }
      read_joints_into_held_state(sim_time);
      if (adapter_tick_ % 5 == 0) {
        read_imu_into_held_state();
      }
      ++adapter_tick_;

      if (controller_->physics_tick_count() % 5 == 0) {
        publish(held_state_, output);
      }
      if (output.faulted && !fault_logged_) {
        fault_logged_ = true;
        RCLCPP_ERROR(node_->get_logger(),
                     "LEGACY_FIRST_FAULT sim_time=%.9f inference=%llu "
                     "reason=%s history_max=%.9g safe_zero=true",
                     sim_time, static_cast<unsigned long long>(
                         controller_->inference_count()),
                     output.fault_reason.c_str(),
                     maximum_absolute(controller_->history()));
      }
      if (output.faulted) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (!controller_fault_active_) {
          controller_fault_active_ = true;
          reset_velocity_input_locked();
        }
      }
      if (self_test_) {
        const bool reset_consumed = command.reset;
        const auto reset_sequence = command.reset_sequence;
        advance_self_test(held_state_, output);
        if (reset_consumed &&
            test_command_.reset_sequence == reset_sequence) {
          test_command_.reset = false;
        }
      } else if (command.reset) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (command_.reset_sequence == command.reset_sequence) {
          command_.reset = false;
        }
      }
    } catch (const std::exception& error) {
      write_effort(rc::JointArray{});
      RCLCPP_FATAL(node_->get_logger(),
                   "RUBI_GAZEBO_LEGACY_RUNTIME=FAIL reason=%s",
                   error.what());
      self_test_complete_ = true;
      world_->SetPaused(true);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_connection_;
  std::array<gazebo::physics::JointPtr, rc::kJointCount> joints_{};
  gazebo::sensors::ImuSensorPtr imu_;
  std::unique_ptr<rc::GazeboLegacyPolicyAdapter> controller_;
  std::string policy_path_;
  bool self_test_{false};
  bool self_test_complete_{false};
  bool fault_logged_{false};
  bool controller_fault_active_{false};
  int policy_test_ticks_{500};
  int stage_{1};
  int stage_tick_{0};
  std::uint64_t inference_stage_start_{0};
  std::uint64_t adapter_tick_{0};
  rc::RobotState held_state_;
  rc::JointArray commanded_effort_{};
  std::mutex command_mutex_;
  rc::UserCommand command_;
  rc::UserCommand test_command_;
  std::array<bool, kSerialButtonCount> previous_serial_buttons_{};
  VelocityInputArbitrator velocity_input_;
  std::uint64_t velocity_command_sequence_{0};
  double nav_cmd_timeout_sec_{0.5};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      action_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<Trigger>::SharedPtr walk_ready_service_;
  rclcpp::Service<Trigger>::SharedPtr policy_on_service_;
  rclcpp::Service<Trigger>::SharedPtr torque_off_service_;
  rclcpp::Service<Trigger>::SharedPtr emergency_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;
};

RubiGazeboLegacyPolicyPlugin::RubiGazeboLegacyPolicyPlugin()
    : impl_(std::make_unique<Impl>()) {}
RubiGazeboLegacyPolicyPlugin::~RubiGazeboLegacyPolicyPlugin() = default;

void RubiGazeboLegacyPolicyPlugin::Load(
    gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  try {
    impl_->Load(std::move(model), std::move(sdf));
  } catch (const std::exception& error) {
    gzerr << "RUBI Gazebo legacy policy plugin load failed: "
          << error.what() << '\n';
    throw;
  }
}

GZ_REGISTER_MODEL_PLUGIN(RubiGazeboLegacyPolicyPlugin)

}  // namespace rubi_gazebo_plugins
