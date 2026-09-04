# Changelog

All notable changes to this project will be documented in this file.

## [2.0.4]

### Changed

- Changed `inverse_continuing` singularity-recovery position shifts to follow
  the target pose's local TCP axes instead of the global coordinate axes.

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
  preserving reliable 5-DOF and 6-DOF solutions for huge (crane-size) robots.

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
