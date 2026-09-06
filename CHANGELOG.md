# Changelog

All notable changes to this project will be documented in this file.

## [3.0.0] - 2026-09-6

- This release contains substantial rewrite of the singularity recovering algorithm.
  Instead of "shifted pose" approach we now recover when calculating wrist joint
  (J4, J5, J6) positions, while base (J1, J2, J3) is never affected by J5 = 0 ± 180 singularity.
  This eliminates specific edge cases when robotic arm is too extended to recover by shift.
- Visualization is no longer a default feature. Being it default was intended
  to encourage onboarding, but now very long build times look annoying. Examples
  that were previously displaying visualization now print the hint how to activate
  if if running on default features.

### Fixed

- Reject non-finite joint offsets and offsets outside ±360° when constructing
  a solver, matching the YAML limits and preventing malformed configurations
  from hanging inverse normalization.

- Handle singularities in the positioning arm, separate from the J5 wrist poles
  at 0° or ±180°. Supported cases are:

  - Free J1 when the wrist center lies on the J1 axis.
  - Free J2 when an arm with equal effective link lengths is fully folded.
  - Free J1 and J2 together when that fully folded wrist center lies on the J1 axis.

  Recovery preserves previous joint choices and searches coupled arm/wrist limits.

- Recognize folded-arm singularities with zero or short flange offsets, including
  zero base height, by accounting for roundoff in cancelling arm-link contributions.
- Use the resolved constraint-center J6 during five-axis continuation and choose
  free J4 at five-axis poles independently of ignored tool roll.
- Recover reachable shoulder and elbow workspace boundaries when floating-point
  roundoff moves square-root or cosine arguments just outside their domains,
  while still rejecting targets beyond the roundoff allowance.
- Preserve small, resolvable nonzero J5 bends and their individual J4/J6 angles
  near both wrist poles instead of rounding them to a singularity.
- Prevent large J4/J6 jumps at exact wrist poles when J1–J3 recovery amplifies
  roundoff. Overlapping pole and regular candidates are FK-validated and
  deduplicated before continuation ranks them against the previous joints.
- Recover reachable wrist-pole poses whose equal J4/J6 correction violates a
  joint limit by choosing the nearest feasible pair along the pole phase line,
  including wrapped limits and reversed joint directions.
- Rank feasible J4/J6 wrist-pole pairs using the same weighted L1 distance as
  continuation sorting, including constraint-center priorities and mixed weights.
- Apply the same ranking to free J4 at five-axis wrist poles, including wrapped
  limits and previous turns. Tied scores prefer the previous J4.
- Reject non-finite continuation references and fixed J6 values. Revalidate
  returned poses after restoring previous turns so large finite references
  cannot produce solutions whose joint rounding invalidates the original FK check.

## [2.0.3] - 2026-09-01

### Fixed

- Reduced docs.rs dependency-compilation memory usage while retaining the
  complete default-feature API documentation, including visualization.

## [2.0.2] - 2026-09-01

### Fixed

- Corrected wrist-joint redistribution at the J5 = 0 singularity.
  `inverse_continuing` now moves J4 and J6 in the same direction to preserve
  J4 + J6, while retaining opposite-direction remapping at J5 = ±π.
- Scaled inverse-kinematics position tolerances and singularity-recovery
  offsets to one part per million of the robot's total absolute OPW geometry,
  preserving reliable 5-DOF and 6-DOF solutions for very large robots.

## [2.0.1] - 2026-08-15

### Changed

- Updated optional `serde-saphyr` support from `0.0.27` to `>=0.1, <2` and
  refreshed locked dependencies.

### Fixed

- Rejected non-finite URDF origin, axis, and joint-limit values. Malformed
  limits now return parsing errors instead of being printed and silently
  treated as unconstrained.
- Prevented constraint construction from hanging on non-finite or extremely
  large wrapped bounds by using constant-time modular normalization. Invalid
  bounds now fail closed.
- Fixed URDF extraction for models containing exactly five joints and no sixth
  joint. They are now recognized as 5-DOF with `c4 = 0`; extracting a nonzero
  TCP offset for this fallback remains unsupported.
