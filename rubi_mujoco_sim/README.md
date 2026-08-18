# RUBI MuJoCo model-only port

This package validates the six-actuator RUBI MuJoCo plant without loading a
policy controller. The CMake search order is `MUJOCO_ROOT`, system paths, then
the active Python interpreter's `mujoco` wheel.

When the Python wheel is selected, the installed executable keeps an RPATH to
that wheel. This is reproducible on the configured host, but it is not a
portable binary distribution. MuJoCo headers and libraries are not copied into
this source package.
