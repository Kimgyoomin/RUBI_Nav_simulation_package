# rubi_gazebo_plugins

Gazebo Classic world-update adapter for `rubi_control_core`. The plugin validates
the canonical six-joint order, single-DoF joints, axis vectors, effort limits, and
the scoped BODY IMU before enabling policy inference.

All joint state and effort operations use axis index 0. This intentional porting
correction is logged as `LEGACY_AXIS_1_BUG_CORRECTION`. ROS callbacks only update
a mutex-protected command snapshot; the Gazebo update thread owns controller state.
