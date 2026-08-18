# rubi_description

ROS 2 resource package for the six-DoF RUBI robot.

## R1 preservation contract

- Geometry source: `/home/kim/rubi_package/RUBI/urdf/RUBI.urdf`
- Root frame: `BODY`
- Active joint order:
  `L_HR_JOINT`, `L_HP_JOINT`, `L_KN_JOINT`,
  `R_HR_JOINT`, `R_HP_JOINT`, `R_KN_JOINT`
- The only intentional URDF text change is replacing the legacy package URI
  authority with `package://rubi_description/`.
- The legacy leading empty item in `joint_names_RUBI.yaml` is not copied.

## Preserved issues

These findings are intentionally not corrected in R1:

1. The commented `R_THIGH` collision mesh references `L_THIGH.STL` at
   `urdf/rubi.urdf:646`.
2. A stray `s` follows the `R_KN_JOINT` limit element at
   `urdf/rubi.urdf:766`.
3. Every STL `<mesh>` element is commented out in the legacy URDF. R1 keeps
   this unchanged, so RViz2 renders the active cylinder, box, and sphere
   visuals rather than STL meshes.
4. Joint axes, limits, inertial values, and the `BODY` frame name are
   unchanged pending simulator parity tests.
5. The two fixed tip joints remain in the URDF but are not included in the
   six-joint controller manifest.
6. `BODY` has inertia in the legacy URDF. KDL warns about inertia on a root
   link, but R1 does not add a dummy root link because `BODY` must be
   preserved.

Non-R1 variants, controllers, policy runtimes, and simulator plugins are
outside this package scope.
