# Glam Migration Tracker

Tracker rule: completed checkboxes must be checked. If a task is finished, update
this file in the same change so the tracker reflects reality.

## Migration Goals

- [ ] Make the crate API and main implementation glam-native, not a nalgebra API
  wrapped in conversions.
- [ ] Migrate exact OPW forward and inverse kinematics first.
- [ ] Keep the no-default feature slice passing as the first hard checkpoint.
- [ ] Port path planning after the core solver and collision-aware shape layer.
- [ ] Port visualization last.
- [ ] Remove `nalgebra` as a normal direct dependency.
- [ ] Keep unavoidable dependency-boundary conversions private and temporary.

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

The repo currently resolves `parry3d` 0.25.x, whose math API is nalgebra-based.
Current `parry3d` 0.26.x exposes `parry3d::math::Pose` as a glamx/glam-style
pose type.

- [x] Use `parry3d` 0.26.x for the glam migration.
- [x] Use `rs-read-trimesh` in its default configuration for the glam migration.
  `rs-read-trimesh` 2.0.9 advertises `default = [parry_26]`.
- [x] Prefer the Parry 0.26.x glam-style API over adding long-lived nalgebra
  conversions around collision queries.
- [ ] During implementation, if `parry3d` 0.26.x or default `rs-read-trimesh`
  does not work in this repo, stop and describe the incompatibility before
  adding a workaround.
- [ ] After the Parry decision, run `cargo tree -i nalgebra` and record why any
  remaining nalgebra path still exists.

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

- [ ] Create crate-owned `Pose`.
- [ ] Store `Pose::translation` as `glam::DVec3`.
- [ ] Store `Pose::rotation` as `glam::DQuat`.
- [ ] Derive or implement `Copy`, `Clone`, `Debug`, and `PartialEq` as
  appropriate.
- [ ] Add `Pose::identity`.
- [ ] Add translation-only constructor.
- [ ] Add constructor from translation plus rotation.
- [ ] Normalize or validate quaternions at construction boundaries.
- [ ] Add `Pose::inverse`.
- [ ] Add pose composition.
- [ ] Add vector transform.
- [ ] Add point transform.
- [ ] Add f64 to f32 conversion for visualization/collision layers.
- [ ] Add angular distance helper.
- [ ] Add debug-friendly translation and rotation accessors if direct fields are
  not public.
- [ ] Add `Twist` for linear plus angular velocity.
- [ ] Add `Wrench` for force plus torque.
- [ ] Add tests for pose identity, inverse, composition, transform, and angular
  distance.

Phase 1 checkpoints:

- [ ] New pose tests pass.
- [ ] No solver behavior has changed yet, or changes are isolated behind unused
  helpers.

## Phase 2: Core OPW Kinematics

- [ ] Redefine `kinematic_traits::Pose` to the new crate-owned type.
- [ ] Update `Kinematics` trait docs.
- [ ] Port `OPWKinematics::forward`.
- [ ] Port `OPWKinematics::forward_with_joint_poses`.
- [ ] Port `OPWKinematics::inverse_intern`.
- [ ] Port `OPWKinematics::inverse_intern_5_dof`.
- [ ] Port `OPWKinematics::inverse_continuing`.
- [ ] Port `compare_poses`.
- [ ] Remove cached nalgebra `unit_z` storage.
- [ ] Replace nalgebra `Matrix3` usage with `glam::DMat3` or focused helpers.
- [ ] Add helper for matrix construction from rows.
- [ ] Add helper for explicit `(row, col)` rotation matrix access.
- [ ] Verify quaternion component ordering: nalgebra used `w/i/j/k`, glam uses
  `x/y/z/w`.
- [ ] Keep inverse solutions cross-checked with forward kinematics.

Phase 2 checkpoints:

- [ ] Forward kinematics tests match previous translation expectations.
- [ ] Forward kinematics tests match previous orientation expectations.
- [ ] Inverse kinematics tests still return valid cross-checked solutions.
- [ ] Singularity tests still pass.
- [ ] 5-DOF tests still pass.

## Phase 3: Always-Compiled Core Modules

These modules are exported without feature gates, so the no-default crate cannot
compile until they are ported or temporarily gated.

- [ ] Port `src/tool.rs`.
- [ ] Port `Tool`.
- [ ] Port `Base`.
- [ ] Port `LinearAxis`.
- [ ] Port `Gantry`.
- [ ] Port `src/frame.rs`.
- [ ] Replace `Point3` API with glam-native point/vector usage.
- [ ] Keep frame transforms rigid.
- [ ] Port `src/parallelogram.rs`.
- [ ] Port `src/utils/utils.rs`.
- [ ] Port pose dumping helpers.
- [ ] Port pose assertion helpers.
- [ ] Replace nalgebra vector helpers or move them behind compatibility feature.
- [ ] Keep `src/jacobian.rs` building, either by porting it in Phase 4 before
  running the no-default checkpoint or by temporarily feature-gating it.

Phase 3 checkpoints:

- [ ] `cargo check --no-default-features`
- [ ] `cargo test --no-default-features`
- [ ] `rg -n "nalgebra|Isometry3|Translation3|UnitQuaternion" src/kinematic_traits.rs src/kinematics_impl.rs src/tool.rs src/frame.rs src/parallelogram.rs src/utils`
  shows no unintended core usage.

## Phase 4: Jacobian

`src/jacobian.rs` is core-adjacent and currently always compiled. It cannot stay
nalgebra-based in the final glam-native core.

API migration:

- [ ] Replace `velocities(&Isometry3<f64>)` with `velocities(&Twist)`.
- [ ] Replace `torques(&Isometry3<f64>)` with `torques(&Wrench)`.
- [ ] Replace public `Vector6<f64>` usage.
- [ ] Decide whether six-element vectors are represented as `Joints`, `[f64; 6]`,
  or a crate-owned `Vector6` newtype.
- [ ] Replace public `Matrix6<f64>` usage.
- [ ] Decide whether the Jacobian matrix remains publicly inspectable.
- [ ] If inspectable, add a crate-owned `Matrix6` type.

Implementation migration:

- [ ] Compute linear deltas with `DVec3`.
- [ ] Compute angular deltas with quaternion delta scaled axis.
- [ ] Replace direct matrix inverse.
- [ ] Replace SVD/pseudoinverse fallback.
- [ ] Decide Jacobian singular fallback strategy.
- [ ] Option A: implement small dense 6x6 inverse/solve plus damped least
  squares.
- [ ] Option B: add a focused linear algebra dependency for SVD/solves while
  keeping geometry glam-native.
- [ ] Option C: temporarily gate `jacobian` only as a short-lived branch tactic.

Phase 4 checkpoints:

- [ ] `src/jacobian.rs` unit tests cover forward, inverse, velocity, torque, and
  matrix computation.
- [ ] `cargo test --no-default-features`
- [ ] Public Jacobian docs no longer describe velocity or force as an isometry.

## Phase 5: Core Tests, Examples, and Docs

- [ ] Port `src/tests/test_utils.rs`.
- [ ] Port YAML pose conversion helpers.
- [ ] Port integration tests that read `pose.translation`.
- [ ] Port integration tests that read or mutate `pose.rotation`.
- [ ] Port integration tests that construct nalgebra isometries.
- [ ] Port `examples/basic.rs`.
- [ ] Port `examples/basic_readme.rs`.
- [ ] Port `examples/constraints.rs`.
- [ ] Port `examples/frame.rs`.
- [ ] Port `examples/jacobian.rs`.
- [ ] Port `examples/parallelogram.rs`.
- [ ] Port `examples/tool_and_base.rs`.
- [ ] Update README `Pose` section.
- [ ] Update README Jacobian velocity/torque vector section.
- [ ] Update README tool/base examples.
- [ ] Update README frame examples.
- [ ] Update doctests in `kinematic_traits`, `tool`, `frame`, and
  `parallelogram`.

Phase 5 checkpoints:

- [ ] `cargo test --no-default-features`
- [ ] `cargo test --no-default-features --features allow_filesystem`
- [ ] `cargo check --example basic`
- [ ] `cargo check --example constraints`
- [ ] `cargo check --example frame`
- [ ] `cargo check --example jacobian`
- [ ] `cargo check --example parallelogram`
- [ ] `cargo check --example tool_and_base`

## Phase 6: Collision and Shape Layer

- [ ] Complete the Parry 0.26.x upgrade evaluation.
- [ ] Port `CollisionBody::pose`.
- [ ] Port `BaseBody::base_pose`.
- [ ] Port `PositionedJoint::transform`.
- [ ] Port public collision placement APIs to glam-native f32 pose data.
- [ ] Port `KinematicsWithShape::new` base transform parameter.
- [ ] Port `KinematicsWithShape::new` tool transform parameter.
- [ ] Port `KinematicsWithShape::with_safety` base transform parameter.
- [ ] Port `KinematicsWithShape::with_safety` tool transform parameter.
- [ ] Keep `TriMesh` as-is unless replacing Parry is explicitly added to scope.
- [ ] If on Parry 0.26.x, map crate f32 poses to `parry3d::math::Pose`.
- [ ] If still on Parry 0.25.x, contain temporary nalgebra conversions inside
  `collisions.rs`.
- [ ] Port `transform_mesh`.
- [ ] Port collision tests.

Phase 6 checkpoints:

- [ ] `cargo test --no-default-features --features collisions`
- [ ] `cargo test --no-default-features --features allow_filesystem,collisions`
- [ ] `cargo tree -i nalgebra` inspected for remaining Parry-related paths.

## Phase 7: Path Planning

- [ ] Port `src/path_plan/rrt.rs` if any type fallout reaches it.
- [ ] Port `src/path_plan/cartesian.rs`.
- [ ] Port `AnnotatedPose`.
- [ ] Port pose interpolation to `DVec3::lerp` and `DQuat::slerp`.
- [ ] Replace nalgebra translation construction in intermediate poses.
- [ ] Keep collision-aware IK behavior unchanged.
- [ ] Port RRT/path planning examples.

Phase 7 checkpoints:

- [ ] `cargo test --no-default-features --features rrt`
- [ ] `cargo check --no-default-features --features rrt --example path_planning_rrt`
- [ ] `cargo check --no-default-features --features rrt,allow_filesystem --example cartesian_stroke`

## Phase 8: Visualization

- [ ] Port `src/visualize/visualization.rs`.
- [ ] Remove nalgebra imports from visualization.
- [ ] Convert robot poses directly into Bevy `Transform` values.
- [ ] Use glam vectors for mesh normal computation.
- [ ] Port TCP pose construction from UI values.
- [ ] Port TCP display extraction from pose.
- [ ] Keep UI numeric arrays if they remain convenient.
- [ ] Port `examples/complete_visible_robot.rs`.
- [ ] Port any visualization-related README snippets.

Phase 8 checkpoints:

- [ ] `cargo check --no-default-features --features visualization`
- [ ] `cargo check --example complete_visible_robot`
- [ ] Manual visualization smoke test: app starts.
- [ ] Manual visualization smoke test: joint sliders update geometry.
- [ ] Manual visualization smoke test: TCP sliders run inverse kinematics.
- [ ] Manual visualization smoke test: collision highlighting still works.

## Phase 9: Cleanup and Release Prep

- [ ] Remove direct normal `nalgebra` dependency from `Cargo.toml`.
- [ ] Keep nalgebra only behind `nalgebra-compat`, if that feature exists.
- [ ] Run `rg -n "nalgebra|Isometry3|Translation3|UnitQuaternion|Matrix6|Vector6" src examples README.md`.
- [ ] Resolve or document every remaining grep result.
- [ ] Run `cargo tree -i nalgebra`.
- [ ] Resolve or document every remaining nalgebra dependency path.
- [ ] Update README to say `Pose` is crate-owned and glam-backed.
- [ ] Update generated or checked-in documentation if maintained manually.
- [ ] Add downstream migration notes.
- [ ] Add release notes for the breaking API change.
- [ ] Check examples with default features.

Final checkpoint matrix:

- [ ] `cargo test --no-default-features`
- [ ] `cargo test --no-default-features --features allow_filesystem`
- [ ] `cargo test --no-default-features --features collisions`
- [ ] `cargo test --no-default-features --features rrt`
- [ ] `cargo test --no-default-features --features allow_filesystem,collisions`
- [ ] `cargo check --no-default-features --features visualization`
- [ ] `cargo check`

## PR Tracker

- [ ] PR 1: feature hygiene, Parry decision, and baseline refresh.
- [ ] PR 2: pose foundation.
- [ ] PR 3: OPW solver and always-compiled wrappers.
- [ ] PR 4: Jacobian.
- [ ] PR 5: core tests, examples, and README.
- [ ] PR 6: collision and shape layer.
- [ ] PR 7: RRT/cartesian planning.
- [ ] PR 8: visualization.
- [ ] PR 9: cleanup and release notes.

## Risk Tracker

- [ ] Rigid transform safety: avoid representing robot poses with scale/shear.
- [ ] Matrix convention safety: audit row/column assumptions when moving from
  nalgebra matrices to glam matrices.
- [ ] Quaternion convention safety: audit `w/i/j/k` versus `x/y/z/w` ordering.
- [ ] Jacobian numerical safety: choose a replacement for SVD/pseudoinverse.
- [ ] f64/f32 boundary safety: make solver-to-collision and solver-to-visual
  conversions explicit.
- [ ] Parry version safety: verify 0.26.x plus `rs-read-trimesh` works before
  relying on Parry's glam-style API.
