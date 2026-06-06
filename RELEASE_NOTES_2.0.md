# 2.0 Release Notes

## Breaking Changes

- Public pose APIs now use crate-owned glam-backed `Pose` and `Pose32` types.
- nalgebra public geometry types were replaced with glam types. nalgebra has
  been removed from the public API and from direct dependencies.
- `KinematicsWithShape` base and tool transforms now use `Pose`; collision and
  visualization placement use `Pose32`.
- Jacobian velocity and torque APIs now use `Twist` and `Wrench`; six-component
  helper APIs use `Joints`.
- `parry3d` is upgraded to 0.26, and `rs-read-trimesh` uses its default Parry
  0.26 support.
- The `rrt` feature was replaced by `stroke_planning`.
- Compatibility feature stubs `ply-rs-bw` and `stl_io` were removed.
- Direct `Cartesian` construction requires the new planner configuration fields.
- `RRTPlanner` gained `smooth`.
- `AnnotatedJoints` gained `move_into`.
- `PathFlags::ALTERED` and `PathFlags::CARTESIAN` were removed.
- Reconfiguration is represented by `PathFlags::RECONFIGURING` plus
  `MoveKind::Joint`.

## Migration Notes

- Replace nalgebra isometry construction with `Pose::from_translation` or
  `Pose::from_parts(DVec3, DQuat)`.
- Import glam types from the crate re-export:
  `use rs_opw_kinematics::glam::{DQuat, DVec3, Quat, Vec3};`.
- If importing `glam` directly, depend on the exact compatible version:
  `glam = "=0.30.10"`.
- Use `Pose32::from_translation` or `Pose32::from_parts(Vec3, Quat)` for
  collision and visualization placement.
- Read glam fields directly through `pose.translation` and `pose.rotation`.
- Add nalgebra as an application dependency if downstream code still needs it.
- Replace `features = ["rrt"]` with `features = ["stroke_planning"]`.
- Remove `ply-rs-bw` and `stl_io` from feature lists. Use
  `rs-read-trimesh` or `allow_filesystem` for mesh/file loading support.
- When constructing `RRTPlanner`, add `smooth`; use `0` to preserve raw RRT
  paths without shortcut smoothing.
- When constructing `Cartesian`, add:
  - `allow_reconfigure`,
  - `max_reconfiguration_prefix_candidates`,
  - `max_onboarding_suffix_candidates`,
  - `max_cartesian_layer_states`,
  - `max_solutions_await`.
  Use the exported `DEFAULT_*` constants or `0` where documented to select the
  crate defaults.
- When consuming `AnnotatedJoints`, read `move_into` to distinguish Cartesian
  motion from joint-space motion into a waypoint.
- Replace checks for removed path flags:
  - `PathFlags::CARTESIAN` -> `AnnotatedJoints::move_into == MoveKind::Cartesian`,
  - `PathFlags::ALTERED` -> `PathFlags::RECONFIGURING` for reconfiguration
    state, with `move_into == MoveKind::Joint` for the joint-space move.
