# Changelog

All notable changes to this project will be documented in this file.

## [2.0.2] - 2026-09-01

### Fixed

- Corrected wrist-joint redistribution at the J5 = 0 singularity.
  `inverse_continuing` now moves J4 and J6 in the same direction to preserve
  J4 + J6, while retaining opposite-direction remapping at J5 = ±π.
- Scaled inverse-kinematics position tolerances and singularity-recovery
  offsets to one part per million of the robot's total absolute OPW geometry,
  preserving reliable 5-DOF and 6-DOF solutions for very large robots.


