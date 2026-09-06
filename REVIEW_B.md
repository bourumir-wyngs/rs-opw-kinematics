# Third kinematics review

Reviewed the working tree on 2026-09-06 after resolving all three findings in
[`REVIEW_A.md`](REVIEW_A.md). One new P2 finding remains open. No new reachability
or invalid-output defect was confirmed in this review.

## [P2] Free-arm sampling can miss better-ranked continuum solutions

Locations: `src/kinematics_impl/arm_continuum.rs:259–266` (`search`) and
`src/kinematics_impl.rs:719–725` (`singular_arm_candidates`).

When an arm angle is free, the search samples constraint crossings, previous and
center crossings, interval endpoints, points just inside endpoints, and
midpoints. These samples establish feasibility, but wrist angles vary
nonlinearly with the free arm angle. The requested weighted L1 distance can be
lower at an interior point that is absent from the generated candidates.
`sort_by_closeness` correctly sorts the candidates it receives, so changing the
sorter alone cannot recover the omitted choice.

For a fully folded Staubli TX2-140 with ordinary wrist angles, continuation under
`BY_PREV` returns total joint movement of `2.0539955302932063` radians while a
valid interior solution requires only `2.013412507031263` radians. This adds
approximately **2.325°** of total movement. Both solutions have at least
0.1 radians of joint-limit margin, position error below `1e-12`, and orientation
error below `1e-12` radians. Both already use the nearest turns to the original
reference. Exact-limit rounding and wrist-pole ambiguity do not explain the gap.

Public-API reproduction, with all joint values in radians:

```rust
use rs_opw_kinematics::{
    constraints::{Constraints, BY_PREV},
    kinematic_traits::{Joints, Kinematics},
    kinematics_impl::OPWKinematics,
    parameters::opw_kinematics::Parameters,
};
use std::f64::consts::PI;

let mut parameters = Parameters::staubli_tx2_140();
parameters.offsets = [0.0; 6];
let limits = Constraints::new(
    [0.3, -2.8, PI - 0.1, -2.8, 0.15, -2.8],
    [0.5,  2.8, PI + 0.1,  2.8, 2.9,   2.8],
    BY_PREV,
);
let robot = OPWKinematics::new_with_constraints(parameters, limits);
let pose = robot.forward(&[0.4, 1.8, PI, 2.3, 0.6, -1.73]);
let previous = [0.4, -1.46, PI, -0.015, 2.57, 0.35];
let solutions = robot.inverse_continuing(&pose, &previous);
assert!(!solutions.is_empty());

let better = [
    0.4, -1.19, PI,
    0.7033316106207576, 2.43266182890736, 1.2377427253178657,
];
let movement = |q: &Joints| {
    q.iter().zip(previous).map(|(a, b)| (a - b).abs()).sum::<f64>()
};
for candidate in [solutions[0], better] {
    assert!(candidate.iter().all(|q| q.is_finite()));
    assert!(limits.compliant(&candidate));
    let actual = robot.forward(&candidate);
    assert!((actual.translation - pose.translation).length() < 1e-12);
    let relative = pose.rotation.conjugate() * actual.rotation;
    let angle = 2.0 * relative.x.hypot(relative.y).hypot(relative.z)
        .atan2(relative.w.abs());
    assert!(angle < 1e-12);
}
assert!(movement(&better) + 0.04 < movement(&solutions[0]));
```

The better candidate was found by another public continuation call with only
the reference J2 changed to `-1.19`, then scored against the original reference.
An independent check using the explicit candidate above reproduced the gap.
A separate `BY_CONSTRAINS` case also confirmed the same cause: changing the
previous J2 changed the best center-distance score from `2.1135292199886444` to
`2.0961494163550536`, without changing the target, limits, or selected turns.

The remaining work is to optimize the requested distance within the feasible
arm intervals, including interior stationary points, and add regressions for
previous-based and constraint-based preferences. The current implementation
provides ranked feasible representatives, with no guarantee of the best point
on the continuous family. Returned poses remain valid.

## Folded-arm fix verification

The resolved flange-length defect now uses the arm-link cancellation scale when
classifying free J1/J2. Radial and vertical bounds remain separate, and workspace
domain bounds are unchanged. The focused reproduction and scale/geometry matrix
failed before the fix and pass afterward.

Three regressions in `src/tests/arm_singularity_tests.rs` cover the reported
zero/short-flange pose, 48 combinations of scale and geometry, transformed axes,
zero base height with an actual vertical cancellation residual, all four inverse
APIs, and exact-reference continuity. Nearby determined poses at ±1e-7 radians
remain determined.

## Review scope and checks

Reviewed arm-domain calculations, one- and two-dimensional free-arm recovery,
wrist phase and ranking, five-axis roll invariance and fixed J6 handling,
constraint filtering, and final normalization/validation. The review used
supported offsets within ±360° and retained the accepted strict comparisons at
exact joint-limit boundaries.

Independent public-API probes checked:

- 5,000 exact singular and 1,500 nearby determined arm cases, including zero
  flange/base height, varied geometry, reversed axes, and wrapped limits.
- 30,000 six-axis pole ranking cases against sampled feasible alternatives;
  another 90,000 inverse calls checked 113,786 returned solutions for finite
  values, constraints, position, and independently computed orientation error.
- 15,000 five-axis configurations and 136,575 returned solutions, including
  changed tool roll, all wrist poles, mixed ranking weights, extra turns,
  sentinel equivalence, and generic/explicit continuation equivalence.

All repository checks passed after the final regression refinement:

| Command | Result |
| --- | --- |
| `cargo fmt --all` and `cargo fmt --all -- --check` | Clean |
| `cargo clippy --offline --all-targets --all-features -- -D warnings` | No warnings |
| `cargo clippy --offline --no-default-features --all-targets -- -D warnings` | No warnings |
| `cargo test --offline` | 221 unit tests, 9 doctests passed |
| `cargo test --offline --no-default-features` | 117 unit tests, 6 doctests passed |
| `cargo test --offline --all-features` | 224 unit tests, 9 doctests passed; 1 existing visualization doctest ignored |
| `git diff --check` | Clean |

Sampling does not establish exhaustive branch completeness or numerical
robustness for every configuration. Severely ill-conditioned combinations of
nanoradian wrist bends and nanoradian joint-limit intervals were outside the
reachability sweeps. The ranking finding above uses ordinary wrist angles and
comfortable joint-limit margins.
