# RUBI Gazebo simulation

This package contains the Ubuntu 22.04 / ROS 2 Humble Gazebo Classic paths for
the base six-DoF RUBI robot. The active daily entry point is:

```bash
ros2 launch rubi_gazebo_sim rubi_gazebo_lidar.launch.py
```

That wrapper starts the legacy ONNX policy integration with a MID360 LiDAR and
IMU. It defaults to a paused, torque-off-safe startup and leaves `/joy` global.

The physical roll-180 sensor output is available on `/livox/lidar_raw` and
`/livox/imu_raw` in `livox_frame_raw`. `mid360_frame_adapter_node` publishes the
canonical FLU data on `/livox/lidar`, `/livox/lidar_PointCloud2`, and
`/livox/imu` in `livox_frame`.

See [the canonical runbook](../../docs/RUBI_SIMULATION_COMMANDS.md) and
[the sensor-frame decision](../../docs/SENSOR_FRAME_DECISION.md).
