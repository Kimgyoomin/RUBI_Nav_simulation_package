# RUBI Gazebo simulation

This package contains the Ubuntu 22.04 / ROS 2 Humble Gazebo Classic paths for
the base six-DoF RUBI robot. The active daily entry point is:

```bash
ros2 launch rubi_gazebo_sim rubi_gazebo_lidar.launch.py
```

That wrapper starts the legacy ONNX policy integration with a MID360 LiDAR and
IMU. It defaults to a paused, torque-off-safe startup and leaves `/joy` global.

The terrain controller has a separate entry point and does not replace the
legacy policy:

```bash
ros2 launch rubi_gazebo_sim rubi_gazebo_terrain_lidar.launch.py
```

It loads `rubi_control_core/models/encoder.onnx` (330 to 32) followed by
`rubi_control_core/models/policy.onnx` (65 to 6). The robot model, world,
MID360/IMU mounting, corrected Livox topics, joystick buttons, command-source
toggle, namespace, and safe-start guard are shared with the legacy entry point.

The physical roll-180 sensor output is available on `/livox/lidar_raw` and
`/livox/imu_raw` in `livox_frame_raw`. `mid360_frame_adapter_node` publishes the
canonical FLU data on `/livox/lidar`, `/livox/lidar_PointCloud2`, and
`/livox/imu` in `livox_frame`.

See [the canonical runbook](../../docs/RUBI_SIMULATION_COMMANDS.md) and
[the sensor-frame decision](../../docs/SENSOR_FRAME_DECISION.md).
