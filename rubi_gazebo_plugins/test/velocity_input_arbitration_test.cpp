#include "rubi_gazebo_plugins/velocity_input_arbitration.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>

namespace rgp = rubi_gazebo_plugins;

namespace {

using Clock = rgp::VelocityInputArbitrator::SteadyClock;
using namespace std::chrono_literals;

TEST(VelocityInputArbitrator, StartsInJoystickMode) {
  rgp::VelocityInputArbitrator arbitrator;
  EXPECT_EQ(arbitrator.source(), rgp::VelocityInputSource::kJoystick);
}

TEST(VelocityInputArbitrator, ShortButtonArraysDoNotAccessButtonThree) {
  EXPECT_FALSE(rgp::button_is_pressed({}, 3));
  EXPECT_FALSE(rgp::button_is_pressed({0}, 3));
  EXPECT_FALSE(rgp::button_is_pressed({0, 0, 0}, 3));
  EXPECT_FALSE(rgp::button_is_pressed({0, 0, 0, 0}, 3));
  EXPECT_TRUE(rgp::button_is_pressed({0, 0, 0, 1}, 3));
}

TEST(VelocityInputArbitrator, ToggleUsesOnlyRisingEdges) {
  rgp::VelocityInputArbitrator arbitrator;
  EXPECT_FALSE(arbitrator.update_source_toggle_button(false));
  EXPECT_TRUE(arbitrator.update_source_toggle_button(true));
  EXPECT_EQ(arbitrator.source(), rgp::VelocityInputSource::kNav2);
  EXPECT_FALSE(arbitrator.update_source_toggle_button(true));
  EXPECT_EQ(arbitrator.source(), rgp::VelocityInputSource::kNav2);
  EXPECT_FALSE(arbitrator.update_source_toggle_button(false));
  EXPECT_TRUE(arbitrator.update_source_toggle_button(true));
  EXPECT_EQ(arbitrator.source(), rgp::VelocityInputSource::kJoystick);
}

TEST(VelocityInputArbitrator, SwitchAppliesOneZeroBeforeJoystickCommand) {
  rgp::VelocityInputArbitrator arbitrator;
  const auto now = Clock::now();
  const rgp::VelocityCommand joystick{0.2, -0.1, 0.3};
  arbitrator.update_source_toggle_button(true);
  arbitrator.update_source_toggle_button(false);
  arbitrator.update_source_toggle_button(true);
  const auto zero = arbitrator.select(joystick, false, now, 500ms);
  EXPECT_TRUE(zero.switch_zero_applied);
  EXPECT_DOUBLE_EQ(zero.command.linear_x, 0.0);
  const auto joy = arbitrator.select(joystick, false, now, 500ms);
  EXPECT_DOUBLE_EQ(joy.command.linear_x, joystick.linear_x);
  EXPECT_DOUBLE_EQ(joy.command.linear_y, joystick.linear_y);
  EXPECT_DOUBLE_EQ(joy.command.angular_z, joystick.angular_z);
}

TEST(VelocityInputArbitrator, Nav2IgnoresJoystickAxesAndUsesFluComponents) {
  rgp::VelocityInputArbitrator arbitrator;
  const auto now = Clock::now();
  arbitrator.update_source_toggle_button(true);
  const rgp::VelocityCommand nav{0.05, 0.03, 0.1};
  EXPECT_TRUE(arbitrator.cache_nav_command(nav, now));
  const auto switch_zero = arbitrator.select({2.0, 0.75, 1.5}, false, now, 500ms);
  EXPECT_TRUE(switch_zero.switch_zero_applied);
  const auto selected = arbitrator.select({-2.0, -0.75, -1.5}, false, now, 500ms);
  EXPECT_TRUE(selected.nav_command_available);
  EXPECT_DOUBLE_EQ(selected.command.linear_x, 0.05);
  EXPECT_DOUBLE_EQ(selected.command.linear_y, 0.03);
  EXPECT_DOUBLE_EQ(selected.command.angular_z, 0.1);
}

TEST(VelocityInputArbitrator, EntryInvalidatesPreviousNavCommand) {
  rgp::VelocityInputArbitrator arbitrator;
  const auto now = Clock::now();
  EXPECT_TRUE(arbitrator.cache_nav_command({0.2, 0.1, 0.1}, now));
  arbitrator.update_source_toggle_button(true);
  const auto switch_zero = arbitrator.select({}, false, now, 500ms);
  EXPECT_TRUE(switch_zero.switch_zero_applied);
  const auto unavailable = arbitrator.select({}, false, now, 500ms);
  EXPECT_FALSE(unavailable.nav_command_available);
  EXPECT_DOUBLE_EQ(unavailable.command.linear_x, 0.0);
}

TEST(VelocityInputArbitrator, TimeoutAndInvalidCommandsStayZeroWithoutFallback) {
  rgp::VelocityInputArbitrator arbitrator;
  const auto now = Clock::now();
  arbitrator.update_source_toggle_button(true);
  EXPECT_TRUE(arbitrator.cache_nav_command({0.05, 0.03, 0.1}, now));
  arbitrator.select({}, false, now, 500ms);
  const auto timed_out = arbitrator.select({2.0, 0.75, 1.5}, false, now + 501ms, 500ms);
  EXPECT_TRUE(timed_out.nav_timeout_started);
  EXPECT_DOUBLE_EQ(timed_out.command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(timed_out.command.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(timed_out.command.angular_z, 0.0);
  EXPECT_FALSE(arbitrator.cache_nav_command(
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, now + 502ms));
  const auto invalid = arbitrator.select({2.0, 0.75, 1.5}, false, now + 502ms, 500ms);
  EXPECT_DOUBLE_EQ(invalid.command.linear_x, 0.0);
}

TEST(VelocityInputArbitrator, SafetyZeroOverridesAllSources) {
  rgp::VelocityInputArbitrator arbitrator;
  const auto now = Clock::now();
  arbitrator.update_source_toggle_button(true);
  ASSERT_TRUE(arbitrator.cache_nav_command({0.05, 0.03, 0.1}, now));
  const auto decision = arbitrator.select({2.0, 0.75, 1.5}, true, now, 500ms);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular_z, 0.0);
}

}  // namespace
