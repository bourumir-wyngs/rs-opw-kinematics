# Glam Migration Tracker

Tracker rule: completed checkboxes must be checked. If a task is finished, update
this file in the same change so the tracker reflects reality.

## Migration Goals

- [x] Make the crate API and main implementation glam-native, not a nalgebra API
  wrapped in conversions.
- [x] Migrate exact OPW forward and inverse kinematics first.
- [x] Keep the no-default feature slice passing as the first hard checkpoint.
- [x] Port path planning after the core solver and collision-aware shape layer.
- [x] Port visualization last.
- [x] Remove `nalgebra` as a normal direct dependency.
- [x] Keep dependency-boundary conversions private.

## Baseline

Observed on 2026-05-28 before implementation work:

- [x] `cargo check --no-default-features` passes.
- [x] `cargo test --no-default-features` passes: 30 unit tests and 6 doctests.
- [x] `cargo check --no-default-features --features collisions` passes.
- [x] `cargo check --no-default-features --features rrt` passes.
- [x] `cargo test --no-default-features --features rrt` passes: 34 unit tests
  and 7 doctests.
- [x] `cargo test --no-default-features --features allow_filesystem,collisions`
  passes: 86 unit tests and 9 doctests.
- [x] `cargo check` with default features passes.
- [x] `cargo check --no-default-features --features allow_filesystem` passes.
  Verified after fixing `allow_filesystem` to enable `parry3d`.

The no-default feature slice currently includes `kinematic_traits`,
`kinematics_impl`, `tool`, `frame`, `parallelogram`, `jacobian`, `constraints`,
parameters, and utilities. It excludes file loading, collisions, RRT/cartesian
planning, and visualization.

## Scope Decisions

- [x] Treat the migration as a breaking API change.
- [x] Target this migration for a 2.0 release. It may live on an independent
  migration branch until ready.
- [x] Do not provide a temporary `nalgebra-compat` feature in the initial
  migration. Revisit compatibility later only if there is a concrete need.
- [x] Use direct `glam` dependency version `0.30.10`, matching the version used
  by Bevy 0.18.
- [x] Use a crate-owned `Pose` type backed by `glam::DVec3` and `glam::DQuat`.
  This keeps the main solver API stable inside the crate while reducing the
  amount of code that must be refactored at once.
- [x] Do not use `DAffine3` as the main public pose type because it can represent
  scale/shear, while robot poses must remain rigid transforms.
- [x] Use glam-native f64 geometry for solver-facing public APIs. Exact IK is
  expected to work properly only with f64 precision.
- [x] Use glam-native f32 geometry for visualization and collision placement.
- [x] Remove nalgebra from the crate API, core implementation, and direct normal
  dependencies before trying to remove every transitive nalgebra occurrence from
  `cargo tree`.

## Parry Decision

Before Phase 6, the repo resolved `parry3d` 0.25.x, whose math API was
nalgebra-based. The migration now resolves `parry3d` 0.26.x, whose
`parry3d::math::Pose` is a glamx/glam-style f32 pose type.

- [x] Use `parry3d` 0.26.x for the glam migration.
- [x] Use `rs-read-trimesh` in its default configuration for the glam migration.
  `rs-read-trimesh` 2.0.9 advertises `default = [parry_26]`.
- [x] Prefer the Parry 0.26.x glam-style API over adding long-lived nalgebra
  conversions around collision queries.
- [x] `parry3d` 0.26.x and default `rs-read-trimesh` work in this repo with
  `cargo test --no-default-features --features allow_filesystem,collisions`.
- [x] After the Parry decision,
  `cargo tree --no-default-features --features allow_filesystem,collisions -i nalgebra`
  shows only this crate's remaining direct dependency; Parry no longer pulls
  nalgebra transitively.

## Phase 0: Feature Hygiene and Baseline

- [x] Fix `allow_filesystem` so it builds without default features.
- [x] Decide whether `allow_filesystem` should enable `parry3d`.
- [x] Keep `read_trimesh` under `allow_filesystem` instead of moving it behind a
  separate mesh/collision feature.
- [x] Add direct `glam = "=0.30.10"` dependency.
- [x] Do not add an initial direct `nalgebra` compatibility feature.
- [x] Record updated baseline results after the feature leak is fixed.

Phase 0 checkpoints:

- [x] `cargo test --no-default-features`
- [x] `cargo test --no-default-features --features allow_filesystem`
- [x] `cargo test --no-default-features --features collisions`
- [x] `cargo test --no-default-features --features rrt`
- [x] `cargo check`

## Phase 1: Pose Foundation

- [x] Create crate-owned `Pose`.
- [x] Store `Pose::translation` as `glam::DVec3`.
- [x] Store `Pose::rotation` as `glam::DQuat`.
- [x] Derive or implement `Copy`, `Clone`, `Debug`, and `PartialEq` as
  appropriate.
- [x] Add `Pose::identity`.
- [x] Add translation-only constructor.
- [x] Add constructor from translation plus rotation.
- [x] Normalize or validate quaternions at construction boundaries.
- [x] Add `Pose::inverse`.
- [x] Add pose composition.
- [x] Add vector transform.
- [x] Add point transform.
- [x] Add f64 to f32 conversion for visualization/collision layers.
- [x] Add angular distance helper.
- [x] Expose public `translation` and `rotation` fields, so separate
  debug-friendly accessors are not needed.
- [x] Add `Twist` for linear plus angular velocity.
- [x] Add `Wrench` for force plus torque.
- [x] Add tests for pose identity, inverse, composition, transform, and angular
  distance.

Phase 1 checkpoints:

- [x] New pose tests pass.
- [x] No solver behavior has changed yet, or changes are isolated behind unused
  helpers.

## Phase 2: Core OPW Kinematics

- [x] Redefine `kinematic_traits::Pose` to the new crate-owned type.
- [x] Update `Kinematics` trait docs.
- [x] Port `OPWKinematics::forward`.
- [x] Port `OPWKinematics::forward_with_joint_poses`.
- [x] Port `OPWKinematics::inverse_intern`.
- [x] Port `OPWKinematics::inverse_intern_5_dof`.
- [x] Port `OPWKinematics::inverse_continuing`.
- [x] Port `compare_poses`.
- [x] Remove cached nalgebra `unit_z` storage.
- [x] Replace nalgebra `Matrix3` usage with `glam::DMat3` or focused helpers.
- [x] Add helper for matrix construction from rows.
- [x] Add helper for explicit `(row, col)` rotation matrix access.
- [x] Verify quaternion component ordering: nalgebra used `w/i/j/k`, glam uses
  `x/y/z/w`.
- [x] Keep inverse solutions cross-checked with forward kinematics.

Phase 2 checkpoints:

- [x] Forward kinematics tests match previous translation expectations.
- [x] Forward kinematics tests match previous orientation expectations.
- [x] Inverse kinematics tests still return valid cross-checked solutions.
- [x] Singularity tests still pass.
- [x] 5-DOF tests still pass.

## Phase 3: Always-Compiled Core Modules

These modules are exported without feature gates, so the no-default crate cannot
compile until they are ported or temporarily gated.

- [x] Port `src/tool.rs`.
- [x] Port `Tool`.
- [x] Port `Base`.
- [x] Port `LinearAxis`.
- [x] Port `Gantry`.
- [x] Port `src/frame.rs`.
- [x] Replace `Point3` API with glam-native point/vector usage.
- [x] Keep frame transforms rigid.
- [x] Port `src/parallelogram.rs`.
- [x] Port `src/utils/utils.rs`.
- [x] Port pose dumping helpers.
- [x] Port pose assertion helpers.
- [x] Move nalgebra `Vector6` helpers out of shared utils and into the Jacobian
  Phase 4 scope.
- [x] Keep `src/jacobian.rs` building, either by porting it in Phase 4 before
  running the no-default checkpoint or by temporarily feature-gating it.

Phase 3 checkpoints:

- [x] `cargo check --no-default-features`
- [x] `cargo test --no-default-features`
- [x] `rg -n "nalgebra|Isometry3|Translation3|UnitQuaternion" src/kinematic_traits.rs src/kinematics_impl.rs src/tool.rs src/frame.rs src/parallelogram.rs src/utils`
  shows no unintended core usage.

## Phase 4: Jacobian

`src/jacobian.rs` is core-adjacent and currently always compiled. Its public
geometry API is now glam-native; nalgebra is no longer used in this module.

API migration:

- [x] Replace `velocities(&Isometry3<f64>)` with `velocities(&Twist)`.
- [x] Replace `torques(&Isometry3<f64>)` with `torques(&Wrench)`.
- [x] Replace public `Vector6<f64>` usage.
- [x] Six-element vectors are still represented as `Joints`, `[f64; 6]`
- [x] Replace public `Matrix6<f64>` usage.
- [x] Jacobian matrix remains publicly inspectable.
- [x] Add a crate-owned `Matrix6` type.

Implementation migration:

- [x] Compute linear deltas with `DVec3`.
- [x] Compute angular deltas with quaternion delta scaled axis.
- [x] Replace direct matrix inverse.
- [x] Replace SVD/pseudoinverse fallback.
- [x] Decide Jacobian singular fallback strategy.
- [x] Option A: implement small dense 6x6 inverse/solve plus damped least
  squares.

Phase 4 checkpoints:

- [x] `src/jacobian.rs` unit tests cover forward, inverse, velocity, torque, and
  matrix computation.
- [x] `cargo test --no-default-features`
- [x] Public Jacobian docs no longer describe velocity or force as an isometry.

## Phase 5: Core Tests, Examples, and Docs

- [x] Port `src/tests/test_utils.rs`.
- [x] Port YAML pose conversion helpers.
- [x] Port integration tests that read `pose.translation`.
- [x] Port integration tests that read or mutate `pose.rotation`.
- [x] Port integration tests that previously constructed nalgebra isometries.
- [x] Port `examples/basic.rs`.
- [x] Port `examples/basic_readme.rs`.
- [x] Port `examples/constraints.rs`.
- [x] Port `examples/frame.rs`.
- [x] Port `examples/jacobian.rs`.
- [x] Port `examples/parallelogram.rs`.
- [x] Port `examples/tool_and_base.rs`.
- [x] Update README `Pose` section.
- [x] Update README Jacobian velocity/torque vector section.
- [x] Update README tool/base examples.
- [x] Update README frame examples.
- [x] Update doctests in `kinematic_traits`, `tool`, `frame`, and
  `parallelogram`.

Phase 5 checkpoints:

- [x] `cargo test --no-default-features`
- [x] `cargo test --no-default-features --features allow_filesystem`
- [x] `cargo check --no-default-features --example basic`
- [x] `cargo check --no-default-features --example constraints`
- [x] `cargo check --no-default-features --example frame`
- [x] `cargo check --no-default-features --example jacobian`
- [x] `cargo check --no-default-features --example parallelogram`
- [x] `cargo check --no-default-features --example tool_and_base`

## Phase 6: Collision and Shape Layer

- [x] Complete the Parry 0.26.x upgrade evaluation.
- [x] Port `CollisionBody::pose`.
- [x] Port `BaseBody::base_pose`.
- [x] Port `PositionedJoint::transform`.
- [x] Port public collision placement APIs to glam-native f32 pose data.
- [x] Port `KinematicsWithShape::new` base transform parameter.
- [x] Port `KinematicsWithShape::new` tool transform parameter.
- [x] Port `KinematicsWithShape::with_safety` base transform parameter.
- [x] Port `KinematicsWithShape::with_safety` tool transform parameter.
- [x] Keep `TriMesh` as-is unless replacing Parry is explicitly added to scope.
- [x] If on Parry 0.26.x, map crate f32 poses to `parry3d::math::Pose`.
- [x] Not applicable: Parry 0.26.x is in use, so `collisions.rs` does not need
  temporary nalgebra conversions.
- [x] Port `transform_mesh`.
- [x] Port collision tests.

Phase 6 checkpoints:

- [x] `cargo test --no-default-features --features collisions`
- [x] `cargo test --no-default-features --features allow_filesystem,collisions`
- [x] `cargo tree -i nalgebra` inspected for remaining Parry-related paths.

## Phase 7: Path Planning

- [x] Port `src/path_plan/rrt.rs` if any type fallout reaches it.
- [x] Port `src/path_plan/cartesian.rs`.
- [x] Port `AnnotatedPose`.
- [x] Port pose interpolation to `DVec3::lerp` and `DQuat::slerp`.
- [x] Replace nalgebra translation construction in intermediate poses.
- [x] Keep collision-aware IK behavior unchanged.
- [x] Port RRT/path planning examples.

Phase 7 checkpoints:

- [x] `cargo test --no-default-features --features rrt`
- [x] `cargo check --no-default-features --features rrt --example path_planning_rrt`
- [x] `cargo check --no-default-features --features rrt,allow_filesystem --example cartesian_stroke`
- [x] Extra check: `cargo check --no-default-features --features rrt,allow_filesystem --example path_planning_rrt`

## Phase 8: Visualization

- [x] Port `src/visualize/visualization.rs`.
- [x] Remove nalgebra imports from visualization.
- [x] Convert robot poses directly into Bevy `Transform` values.
- [x] Use glam vectors for mesh normal computation.
- [x] Port TCP pose construction from UI values.
- [x] Port TCP display extraction from pose.
- [x] Keep UI numeric arrays if they remain convenient.
- [x] Port `examples/complete_visible_robot.rs`.
- [x] Port any visualization-related README snippets.

Phase 8 automated checkpoints:

- [x] `cargo check --no-default-features --features visualization`
- [x] `cargo check --example complete_visible_robot`

Phase 8 manual smoke tests:

- [x] Manual visualization smoke test: app starts.
- [x] Manual visualization smoke test: joint sliders update geometry.
- [x] Manual visualization smoke test: TCP sliders run inverse kinematics.
- [x] Manual visualization smoke test: collision highlighting still works.

## Phase 9: Cleanup and Release Prep

- [x] Remove direct normal `nalgebra` dependency from `Cargo.toml`.
- [x] Do not keep nalgebra behind a compatibility feature in the initial 2.0
  migration; no `nalgebra-compat` feature exists.
- [x] Run `rg -n "nalgebra|Isometry3|Translation3|UnitQuaternion|Matrix6|Vector6" src examples README.md Cargo.toml`.
- [x] Resolve or document every remaining grep result: README has intentional
  2.0 migration notes, and `src/jacobian.rs` has the crate-owned `Matrix6`.
- [x] Run `cargo tree -i nalgebra`.
- [x] Resolve or document every remaining nalgebra dependency path:
  `cargo tree -i nalgebra` reports that no package ID `nalgebra` exists.
- [x] Update README to say `Pose` is crate-owned and glam-backed.
- [x] Update generated or checked-in documentation if maintained manually:
  no generated API docs are checked in; README and release notes were updated.
- [x] Add downstream migration notes.
- [x] Add release notes for the breaking API change.
- [x] Check examples with default features.

Final checkpoint matrix:

- [x] `cargo test --no-default-features`
- [x] `cargo test --no-default-features --features allow_filesystem`
- [x] `cargo test --no-default-features --features collisions`
- [x] `cargo test --no-default-features --features rrt`
- [x] `cargo test --no-default-features --features allow_filesystem,collisions`
- [x] `cargo check --no-default-features --features visualization`
- [x] `cargo check`

## PR Tracker

- [x] PR 1: feature hygiene, Parry decision, and baseline refresh.
- [x] PR 2: pose foundation.
- [x] PR 3: OPW solver and always-compiled wrappers.
- [x] PR 4: Jacobian.
- [x] PR 5: core tests, examples, and README.
- [x] PR 6: collision and shape layer.
- [x] PR 7: RRT/cartesian planning.
- [x] PR 8: visualization.
- [x] PR 9: cleanup and release notes.

## Risk Tracker

- [x] Rigid transform safety: avoid representing robot poses with scale/shear.
- [x] Matrix convention safety: audit row/column assumptions when moving from
  nalgebra matrices to glam matrices.
- [x] Quaternion convention safety: audit `w/i/j/k` versus `x/y/z/w` ordering.
- [x] Jacobian numerical safety: choose a replacement for SVD/pseudoinverse.
- [x] f64/f32 boundary safety: make solver-to-collision and solver-to-visual
  conversions explicit.
- [x] Parry version safety: verify 0.26.x plus `rs-read-trimesh` works before
  relying on Parry's glam-style API.
