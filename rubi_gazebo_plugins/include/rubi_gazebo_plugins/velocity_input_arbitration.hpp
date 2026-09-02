#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <vector>

namespace rubi_gazebo_plugins {

enum class VelocityInputSource {
  kJoystick,
  kNav2,
};

struct VelocityCommand {
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

struct VelocityInputDecision {
  VelocityCommand command{};
  bool switch_zero_applied{false};
  bool nav_timeout_started{false};
  bool nav_command_available{false};
};

inline VelocityCommand terrain_navigation_command(
    const VelocityCommand& command) noexcept {
  constexpr double kLinearXLimit = 0.5;
  constexpr double kLinearYLimit = 0.5;
  constexpr double kAngularZLimit = 1.0;
  constexpr double kTerrainLinearXScale = 1.0;
  constexpr double kTerrainLinearYScale = 0.75;
  constexpr double kTerrainAngularZScale = 1.5;
  return {
      std::clamp(command.linear_x / kLinearXLimit, -1.0, 1.0) *
          kTerrainLinearXScale,
      std::clamp(command.linear_y / kLinearYLimit, -1.0, 1.0) *
          kTerrainLinearYScale,
      std::clamp(command.angular_z / kAngularZLimit, -1.0, 1.0) *
          kTerrainAngularZScale,
  };
}

inline VelocityCommand terrain_joystick_command(
    const VelocityCommand& command) noexcept {
  return {0.5 * command.linear_x, command.linear_y, command.angular_z};
}

inline bool button_is_pressed(const std::vector<int32_t>& buttons,
                              std::size_t index) noexcept {
  return buttons.size() > index && buttons[index] != 0;
}

// Keeps source selection and Nav2 freshness separate from the controller core.
class VelocityInputArbitrator {
 public:
  using SteadyClock = std::chrono::steady_clock;

  VelocityInputSource source() const noexcept { return source_; }

  bool update_source_toggle_button(bool pressed) {
    const bool toggled = pressed && !previous_toggle_button_;
    previous_toggle_button_ = pressed;
    if (!toggled) {
      return false;
    }
    source_ = source_ == VelocityInputSource::kJoystick
                  ? VelocityInputSource::kNav2
                  : VelocityInputSource::kJoystick;
    switch_zero_pending_ = true;
    if (source_ == VelocityInputSource::kNav2) {
      invalidate_nav_command();
    }
    return true;
  }

  void synchronize_source_toggle_button(bool pressed) noexcept {
    previous_toggle_button_ = pressed;
  }

  void reset_to_joystick() noexcept {
    source_ = VelocityInputSource::kJoystick;
    previous_toggle_button_ = false;
    switch_zero_pending_ = true;
    invalidate_nav_command();
  }

  bool cache_nav_command(const VelocityCommand& command,
                         SteadyClock::time_point received_at) noexcept {
    nav_command_ = command;
    nav_received_at_ = received_at;
    nav_command_valid_ = std::isfinite(command.linear_x) &&
                         std::isfinite(command.linear_y) &&
                         std::isfinite(command.angular_z);
    nav_timeout_active_ = false;
    return nav_command_valid_;
  }

  VelocityInputDecision select(const VelocityCommand& joystick_command,
                               bool force_zero,
                               SteadyClock::time_point now,
                               std::chrono::duration<double> timeout) {
    VelocityInputDecision decision;
    if (force_zero) {
      return decision;
    }
    if (switch_zero_pending_) {
      switch_zero_pending_ = false;
      decision.switch_zero_applied = true;
      return decision;
    }
    if (source_ == VelocityInputSource::kJoystick) {
      decision.command = joystick_command;
      return decision;
    }
    if (nav_command_valid_ && now - nav_received_at_ <= timeout) {
      decision.command = nav_command_;
      decision.nav_command_available = true;
      return decision;
    }
    if (!nav_timeout_active_) {
      nav_timeout_active_ = true;
      decision.nav_timeout_started = true;
    }
    return decision;
  }

 private:
  void invalidate_nav_command() noexcept {
    nav_command_ = VelocityCommand{};
    nav_command_valid_ = false;
    nav_timeout_active_ = false;
  }

  VelocityInputSource source_{VelocityInputSource::kJoystick};
  bool previous_toggle_button_{false};
  bool switch_zero_pending_{false};
  bool nav_command_valid_{false};
  bool nav_timeout_active_{false};
  VelocityCommand nav_command_{};
  SteadyClock::time_point nav_received_at_{};
};

}  // namespace rubi_gazebo_plugins
