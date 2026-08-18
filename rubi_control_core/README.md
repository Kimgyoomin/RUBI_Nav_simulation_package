# rubi_control_core

Simulator-independent controller for the approved RUBI six-DoF encoder and policy.
The public API owns no ROS 2, MuJoCo, Gazebo, filesystem path, or simulator index.
Adapters must validate canonical joint names and resolve installed model paths before
constructing `PolicyRunner`.

The controller starts torque-off, runs physics at a caller-owned 0.002 s cadence,
updates inference every five physics ticks, clamps effort to +/-90, and latches
emergency-stop or nonfinite faults until reset.
