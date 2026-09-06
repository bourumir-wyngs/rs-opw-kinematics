# Second kinematics review

Reviewed the working tree on 2026-09-06, after completing the original review
coverage, formatting, and Clippy checks. All three findings below are resolved.

## Resolved: [P1] Folded-arm singularity detection incorrectly depends on flange length

Location: `src/kinematics_impl.rs:434–451` (`arm_branches`).

For a fully folded arm with equal effective link lengths, forward kinematics can
leave a tiny residual at the base axis because the link contributions cancel
inexactly. The free-J1/free-J2 classification previously derived its radial error
allowance from the resulting TCP position and flange length. With a zero or very
short flange, that allowance missed the arm-link cancellation error, so neither
free family was searched and reachable constrained targets had no solution.

Classification now includes a linear error allowance based on the arm-link
lengths. Its radial component includes `a1`; its vertical component uses only
the two effective links. The workspace square-root/cosine bounds, equal-link
guard, nonzero-`b` exclusion for free J1, and final FK checks remain in effect.

Reproduction, with joint values and limits in radians:

```rust
use rs_opw_kinematics::{
    constraints::{Constraints, BY_PREV},
    kinematic_traits::Kinematics,
    kinematics_impl::OPWKinematics,
    parameters::opw_kinematics::Parameters,
};
use std::f64::consts::PI;

let mut parameters = Parameters::staubli_tx2_140();
parameters.a1 = 0.0;
parameters.c4 = 0.0; // Also covered with 0.0001.
parameters.offsets = [0.0; 6];
let q = [0.4, 0.2, PI, 0.3, 0.5, 0.6];
let limits = Constraints::new(
    [-1.0, -1.0, PI - 0.01, 0.29, 0.49, 0.59],
    [1.0, 1.0, PI + 0.01, 0.31, 0.51, 0.61],
    BY_PREV,
);
let robot = OPWKinematics::new_with_constraints(parameters, limits);
assert!(limits.compliant(&q));
let pose = robot.forward(&q);
let solutions = robot.inverse_continuing(&pose, &q);
assert!(!solutions.is_empty());
```

The resulting geometry is `a1 = a2 = b = c4 = 0`, `c1 = 0.55`, and
`c2 = c3 = 0.625`, with unit sign corrections. J5 is an ordinary 0.5 radians;
the witness is 0.01 radians inside the J3–J6 limits. This is distinct from the
accepted exact-limit rounding behavior.

Before the fix, the debugger measured a radial residual of
`4.163336342344337e-17` against a radial allowance of `1.4791141972894106e-30`.
The focused regression and scale/geometry matrix both failed before the change
and pass afterward. They cover all four inverse APIs, exact-reference
continuity, zero/short flange offsets, zero base height, equal effective links
with nonzero `a2`, and transformed axes. A nearby ±1e-7-radian bend regression
also verifies that genuinely determined J1/J2 angles are not replaced by free
choices. The 48-case matrix spans robot scales 0.001, 1, and 1000.

## Resolved: [P2] Five-axis free J4 ignores constraint-based ranking

Locations: `src/kinematics_impl.rs:589` (`wrist_to_joints`) and
`src/kinematics_impl.rs:944` (`nearest_feasible_j4`).

At a five-axis wrist pole, J4 is independent of the ignored tool roll. Candidate
selection previously always minimized movement from previous, ignoring
`sorting_weight`. It now minimizes the same weighted L1 objective as the final
sorter. The search checks the previous/center choices and endpoints in each
feasible circular interval, including points just inside the normalization cut.
It scores the actual normalized angle against the original reference and raw
constraint center, checks both bounded and normalized representations against
the limits, and uses proximity to previous to break tied scores.

Reproduction, with all angles below in degrees:

- Start with `Parameters::irb2400_10()`, set `offsets = [0.0; 6]`,
  `sign_corrections = [1; 6]`, and `dof = 5`.
- Use `Constraints::from_degrees` with ranges
  `[9..=11, 19..=21, 29..=31, 0..=100, -1..=1, -1..=1]`
  and weight `BY_CONSTRAINS`.
- Target: `forward([10, 20, 30, 40, 0, 0])`.
- Previous: `[10, 20, 30, 10, 0, 0]`.

Both `inverse_continuing_5dof` and generic `inverse_continuing` now select
J4 = 50° under `BY_CONSTRAINS`, instead of 10°. This removes 40° of unnecessary
distance from the constraint centers while preserving position, tool-axis
direction, and fixed J6.

Three new public-API regressions failed before the fix and pass afterward.
They cover previous/center priorities, mixed weights and ties, all wrist poles,
offsets and reversed axes, wrapped limits, extra previous turns, and the +π
normalization cut. Returned solutions must be finite, compliant, preserve fixed
J6, and reproduce the requested position and tool axis.

## Resolved: [P2] Huge finite offsets can make inverse normalization loop forever

Validation: `src/kinematics_impl.rs:44–51` (`from_parameters`).

Both public solver constructors now require all six offsets to be finite and
within inclusive `[-2π, 2π]` radians (±360°), matching the existing YAML limit.
Malformed offsets trigger an assertion identifying the joint and supplied value
at construction, before inverse normalization can run. URDF-to-robot conversion
uses the same checked constructor. The constructors still return `Self`; their
panic behavior and the supported offset range are documented.

Previously, an offset such as `parameters.offsets[3] = 1e20` could make every
inverse path loop forever: repeated subtraction of `2π` rounded back to the same
large value. Such offsets are treated as invalid configuration rather than
supported model inputs.

Regression tests cover both constructors, all six offset positions, and five-
and six-axis models. They reject values one ULP outside each boundary, ±1e20,
NaN, and either infinity. Exact ±360° offsets remain unchanged and permit
successful forward/inverse recovery.

## Original review and coverage audit

The original concrete reproductions for items 1, 2, 3, 5, 6, 7 (six-axis), and 8
are addressed and have dedicated regressions. Item 4 retains the explicitly
accepted strict boundary comparisons, with the requested explanation on
`Constraints`.

This completion pass added the focused one-ULP reference-reconstruction
regression. It passes with the original user reference and fails with an isolated
mutation restoring the old reconstructed reference.

Both fixture inverse tests now check all 15,406 stored rows across 2,048 cases:
15,374 ordinary branches must match all six joints, and 32 pole rows must match
their arm/J5 families while returned full poses are independently checked.
Of those pole rows, 26 across 13 cases contain invalid stored wrist splits;
literal wrist-angle matching would incorrectly require an invalid pose.
Fixtures were preserved. Joint and pose comparison helpers now reject nonfinite
values and invalid tolerances, with five focused helper regressions.

## Verification

All commands completed successfully after resolving the folded-arm finding
and adding its three regressions in `src/tests`:

| Command | Result |
| --- | --- |
| `cargo fmt --all` and `cargo fmt --all -- --check` | Clean |
| `cargo clippy --offline --all-targets --all-features -- -D warnings` | No warnings |
| `cargo clippy --offline --no-default-features --all-targets -- -D warnings` | No warnings |
| `cargo test --offline` | 221 unit tests, 9 doctests passed |
| `cargo test --offline --no-default-features` | 117 unit tests, 6 doctests passed |
| `cargo test --offline --all-features` | 224 unit tests, 9 doctests passed; 1 existing visualization doctest ignored |
| `git diff --check` | Clean |

This review also used independent public-API probes for six-axis wrist
ranking, constrained free-arm recovery, and five-axis output validity. No further
actionable defect was confirmed in those probes. Sampling does not establish
exhaustive completeness, especially for severely ill-conditioned combinations of
almost-singular wrist bends and extremely narrow joint limits.

The subsequent review after resolving all three findings is recorded in
[`REVIEW_B.md`](REVIEW_B.md).
