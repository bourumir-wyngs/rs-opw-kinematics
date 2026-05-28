# 2.0 Release Notes

## Breaking Changes

- Public pose APIs now use crate-owned glam-backed `Pose` and `Pose32` types.
- nalgebra has been removed from the public API and from direct dependencies.
- `KinematicsWithShape` base and tool transforms now use `Pose`; collision and
  visualization placement use `Pose32`.
- Jacobian velocity and torque APIs now use `Twist` and `Wrench`; six-component
  helper APIs use `Joints`.
- `parry3d` is upgraded to 0.26, and `rs-read-trimesh` uses its default Parry
  0.26 support.

## Migration Notes

- Replace nalgebra isometry construction with `Pose::from_translation` or
  `Pose::from_parts(DVec3, DQuat)`.
- Use `Pose32::from_translation` or `Pose32::from_parts(Vec3, Quat)` for
  collision and visualization placement.
- Read glam fields directly through `pose.translation` and `pose.rotation`.
- Add nalgebra as an application dependency if downstream code still needs it.
