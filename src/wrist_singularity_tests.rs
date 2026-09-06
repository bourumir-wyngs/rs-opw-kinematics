use super::*;
use crate::kinematic_traits::{CONSTRAINT_CENTERED, J5};

fn model_parameters() -> Parameters {
    let mut parameters = Parameters::irb2400_10();
    parameters.offsets = [0.0; 6];
    parameters.sign_corrections = [1; 6];
    parameters
}

fn from_model(model: Joints, parameters: &Parameters) -> Joints {
    std::array::from_fn(|i| {
        (model[i] + parameters.offsets[i]) * parameters.sign_corrections[i] as f64
    })
}

fn angle_error(actual: f64, expected: f64) -> f64 {
    ((actual - expected + PI).rem_euclid(2.0 * PI) - PI).abs()
}

fn wrist_matrix_from_model(q4: f64, q5: f64, q6: f64) -> RotationMatrix {
    RotationMatrix::from_quat(
        DQuat::from_rotation_z(q4) * DQuat::from_rotation_y(q5) * DQuat::from_rotation_z(q6),
    )
}

fn assert_pose(robot: &OPWKinematics, joints: &Joints, target: &Pose) {
    assert!(joints.iter().all(|joint| joint.is_finite()));
    let actual = robot.forward(joints);
    assert!(
        (actual.translation - target.translation).length() <= robot.distance_tolerance,
        "translation mismatch for {joints:?}"
    );
    assert!(
        actual.angular_distance(*target) <= ANGULAR_TOLERANCE,
        "orientation mismatch for {joints:?}"
    );
}

fn assert_contains_joints(solutions: &[Joints], expected: &Joints) {
    assert!(
        solutions.iter().any(|solution| {
            solution
                .iter()
                .zip(expected.iter())
                .all(|(&actual, &expected)| angle_error(actual, expected) < 1e-7)
        }),
        "expected original arm branch with wrist near {expected:?}, got {solutions:?}"
    );
}

#[test]
fn singular_wrist_splits_phase_correction_from_independent_rotation() {
    // A zero-angle arm leaves the independently composed wrist rotation unchanged.
    let arm = ArmBranch {
        q1: 0.0,
        q2: 0.0,
        q3: 0.0,
    };
    let near = J4J6Near {
        j4: 10.0_f64.to_radians(),
        j6: 20.0_f64.to_radians(),
    };
    let cases: [([f64; 3], [f64; 3]); 3] = [
        // At zero, the sum changes from 30° to 90°: add 30° to both joints.
        ([60.0, 0.0, 30.0], [40.0, 0.0, 50.0]),
        // At either pi pole, the difference changes from -10° to 30°:
        // add 20° to J4 and subtract 20° from J6.
        ([60.0, 180.0, 30.0], [30.0, 180.0, 0.0]),
        ([60.0, -180.0, 30.0], [30.0, -180.0, 0.0]),
    ];

    for (target_degrees, expected_degrees) in cases {
        let target = target_degrees.map(f64::to_radians);
        let expected = expected_degrees.map(f64::to_radians);
        let matrix = wrist_matrix_from_model(target[0], target[1], target[2]);
        let branches = OPWKinematics::wrist_branch(&matrix, arm, &near).collect::<Vec<_>>();

        assert!(
            branches.iter().any(|wrist| {
                [wrist.q4, wrist.q5, wrist.q6]
                    .iter()
                    .zip(expected.iter())
                    .all(|(&actual, &expected)| angle_error(actual, expected) <= 1e-12)
            }),
            "target {target_degrees:?}°: expected wrist {expected_degrees:?}°, got {branches:?}"
        );

        // Both wrist branches must reproduce the target without using robot.forward.
        for wrist in branches {
            let actual = wrist_matrix_from_model(wrist.q4, wrist.q5, wrist.q6);
            assert!(
                actual.matrix.abs_diff_eq(matrix.matrix, 1e-12),
                "target {target_degrees:?}°: wrist {wrist:?} changed the orientation"
            );
        }
    }
}

#[test]
fn inverse_continuing_recovers_wrist_near_previous_in_both_directions() {
    let robot = OPWKinematics::new(model_parameters());
    // Each wrist triple is [J4, J5, J6] in degrees. The target's wrist
    // distribution differs from the expected nearest solution in every case.
    let cases = [
        (
            "J5=0: retain previous when its sum already matches",
            [10.0, 0.0, 20.0],
            [40.0, 0.0, -10.0],
            [10.0, 0.0, 20.0],
        ),
        (
            "J5=0: both joints increase 20 degrees through zero",
            [-10.0, 0.0, -5.0],
            [40.0, 0.0, -15.0],
            [10.0, 0.0, 15.0],
        ),
        (
            "J5=0: both joints decrease 20 degrees through zero",
            [10.0, 0.0, 5.0],
            [-40.0, 0.0, 15.0],
            [-10.0, 0.0, -15.0],
        ),
        (
            "J5=180: retain previous when its difference already matches",
            [10.0, 180.0, 20.0],
            [40.0, 180.0, 50.0],
            [10.0, 180.0, 20.0],
        ),
        (
            "J5=180: J4 increases and J6 decreases 20 degrees through zero",
            [-10.0, 180.0, 5.0],
            [40.0, 180.0, 15.0],
            [10.0, 180.0, -15.0],
        ),
        (
            "J5=180: J4 decreases and J6 increases 20 degrees through zero",
            [10.0, 180.0, -5.0],
            [-40.0, 180.0, -15.0],
            [-10.0, 180.0, 15.0],
        ),
    ];
    // Keep the arm on the same nonsingular branch throughout.
    let with_wrist = |wrist: [f64; 3]| [20.0, 35.0, -25.0, wrist[0], wrist[1], wrist[2]];
    let tolerance_degrees = 1e-7;

    for (context, previous_wrist, target_wrist, expected_wrist) in cases {
        let previous_degrees = with_wrist(previous_wrist);
        let previous = previous_degrees.map(f64::to_radians);
        let target = with_wrist(target_wrist).map(f64::to_radians);
        let expected_degrees = with_wrist(expected_wrist);
        let pose = robot.forward(&target);
        let solutions = robot.inverse_continuing(&pose, &previous);
        let best = solutions
            .first()
            .unwrap_or_else(|| panic!("{context}: no solutions"));
        let actual_degrees = best.map(f64::to_degrees);

        // Check the first (closest) solution without modular comparison or
        // renormalizing it: copying previous or jumping a full turn must fail.
        for joint in 0..6 {
            assert!(
                (actual_degrees[joint] - expected_degrees[joint]).abs() <= tolerance_degrees,
                "{context}: J{} was {}°, expected {}° (previous {}°)",
                joint + 1,
                actual_degrees[joint],
                expected_degrees[joint],
                previous_degrees[joint],
            );
        }
        for joint in [J4, J6] {
            assert!(
                (actual_degrees[joint] - previous_degrees[joint]).abs() <= 20.0 + tolerance_degrees,
                "{context}: J{} moved more than 20° from previous",
                joint + 1,
            );
        }
        for solution in &solutions {
            assert_pose(&robot, solution, &pose);
        }
    }
}

#[test]
fn exact_wrist_poles_recover_nearest_pair_on_original_arm_branch() {
    let robot = OPWKinematics::new(model_parameters());
    for q5 in [0.0, PI, -PI] {
        let target = [0.4, 0.6, -0.3, 0.7, q5, -0.2];
        let previous = [0.4, 0.6, -0.3, 0.2, q5, -0.4];
        let pose = robot.forward(&target);
        let near = J4J6Near::from_joints(&previous, &robot.parameters);
        let arm = ArmBranch {
            q1: target[0],
            q2: target[1],
            q3: target[2],
        };
        let branches =
            OPWKinematics::wrist_branch(&RotationMatrix::from_quat(pose.rotation), arm, &near);
        let mut recovered = Vec::new();
        for wrist in branches {
            let joints = [arm.q1, arm.q2, arm.q3, wrist.q4, wrist.q5, wrist.q6];
            assert_pose(&robot, &joints, &pose);
            recovered.push(joints);
        }
        let expected = if q5 == 0.0 {
            [0.4, 0.6, -0.3, 0.55, q5, -0.05]
        } else {
            [0.4, 0.6, -0.3, 0.35, q5, -0.55]
        };
        assert_contains_joints(&recovered, &expected);
        assert_contains_joints(&robot.inverse_continuing(&pose, &previous), &expected);
    }
}

#[test]
fn exact_wrist_poles_preserve_previous_despite_arm_recovery_error() {
    let robot = OPWKinematics::new(Parameters::irb2400_10());
    // Recovering this nearly degenerate arm branch amplifies roundoff enough
    // that the recovered wrist has a nonzero sine even at an exact pole.
    for j5_degrees in [0.0, 180.0, -180.0] {
        let previous = [
            152.29350524023175,
            -88.2055330183357,
            -79.86287500709295,
            22.963288826867934,
            j5_degrees,
            53.3190125785768,
        ]
        .map(f64::to_radians);
        let pose = robot.forward(&previous);
        let solutions = robot.inverse_continuing(&pose, &previous);
        let best = solutions
            .first()
            .expect("the exact previous solution exists");

        // Exercise the entire inverse path, including arm_branches(), and
        // check ranking and unwrapped continuity of the first solution.
        for joint in 0..6 {
            assert!(
                (best[joint] - previous[joint]).abs() < 1e-7,
                "J5={j5_degrees} degrees: J{} jumped from {} to {} degrees",
                joint + 1,
                previous[joint].to_degrees(),
                best[joint].to_degrees(),
            );
        }
        for solution in &solutions {
            assert_pose(&robot, solution, &pose);
        }
    }
}

#[test]
fn inverse_deduplicates_coincident_arm_branches_at_wrist_poles() {
    let parameters = Parameters {
        a1: 0.25,
        a2: 0.0,
        b: 0.0,
        c1: 0.5,
        c2: 0.5,
        c3: 0.5,
        c4: 0.125,
        offsets: [0.0; 6],
        sign_corrections: [1; 6],
        dof: 6,
    };
    let robot = OPWKinematics::new(parameters);
    let same_joints = |a: &Joints, b: &Joints| {
        a.iter()
            .zip(b.iter())
            .all(|(&a, &b)| angle_error(a, b) < 1e-12)
    };

    for q5 in [0.0, PI, -PI] {
        // At full arm extension, the two elbow branches coincide exactly.
        let target = [0.0, 0.0, 0.0, 0.7, q5, -0.2];
        let pose = robot.forward(&target);
        for (solutions, reference) in [
            (robot.inverse(&pose), [0.0; 6]),
            (robot.inverse_continuing(&pose, &target), target),
        ] {
            // Establish that the fixture supplies duplicate valid candidates;
            // otherwise uniqueness of the public results would be vacuous.
            let near = J4J6Near::from_joints(&reference, &parameters);
            let valid_candidates: Vec<_> = robot
                .inverse_candidates(&pose, &near, None)
                .filter(|candidate| {
                    candidate.iter().all(|joint| joint.is_finite())
                        && compare_poses(
                            &pose,
                            &robot.forward(candidate),
                            robot.distance_tolerance,
                            ANGULAR_TOLERANCE,
                        )
                })
                .collect();
            assert!(
                valid_candidates.iter().enumerate().any(|(i, candidate)| {
                    valid_candidates[i + 1..]
                        .iter()
                        .any(|other| same_joints(candidate, other))
                }),
                "q5={q5}: fixture must produce duplicate valid candidates"
            );
            assert!(!solutions.is_empty());
            assert!(solutions.len() < valid_candidates.len());
            for (i, solution) in solutions.iter().enumerate() {
                assert_pose(&robot, solution, &pose);
                assert!(
                    solutions[i + 1..]
                        .iter()
                        .all(|other| !same_joints(solution, other)),
                    "q5={q5}: duplicate output joint configurations: {solutions:?}"
                );
            }
        }
    }
}

#[test]
fn singular_wrist_correction_wraps_and_preserves_reference_turns() {
    let robot = OPWKinematics::new(model_parameters());
    let near = J4J6Near {
        j4: 4.0 * PI + 3.0,
        j6: -2.0 * PI + 3.1,
    };
    let target = [0.4, 0.6, -0.3, -3.0, 0.0, -2.9];
    let pose = robot.forward(&target);
    let arm = ArmBranch {
        q1: target[0],
        q2: target[1],
        q3: target[2],
    };
    let branches =
        OPWKinematics::wrist_branch(&RotationMatrix::from_quat(pose.rotation), arm, &near)
            .collect::<Vec<_>>();
    // The required sum crosses the +/-pi boundary; its shortest correction is
    // 4*pi - 12 radians, split equally between the two preferred angles.
    let correction = 2.0 * PI - 6.0;
    assert!(
        branches.iter().any(|wrist| {
            angle_error(wrist.q4, near.j4 + correction) < 1e-10
                && angle_error(wrist.q6, near.j6 + correction) < 1e-10
        }),
        "shortest correction lost: {branches:?}"
    );
    for wrist in branches {
        assert_pose(
            &robot,
            &[arm.q1, arm.q2, arm.q3, wrist.q4, wrist.q5, wrist.q6],
            &pose,
        );
    }
    let previous = [arm.q1, arm.q2, arm.q3, near.j4, 0.0, near.j6];
    let solutions = robot.inverse_continuing(&pose, &previous);
    assert!(
        solutions.iter().any(|solution| {
            (solution[J4] - (near.j4 + correction)).abs() < 1e-10
                && (solution[J6] - (near.j6 + correction)).abs() < 1e-10
        }),
        "continuation lost preferred turns: {solutions:?}"
    );
}

#[test]
fn singular_wrist_with_huge_finite_reference_returns_bounded_angles() {
    let robot = OPWKinematics::new(model_parameters());
    let near = J4J6Near {
        j4: 1e100,
        j6: -1e100,
    };
    for q5 in [0.0, PI] {
        let target = [0.4, 0.6, -0.3, 0.7, q5, -0.2];
        let pose = robot.forward(&target);
        let arm = ArmBranch {
            q1: target[0],
            q2: target[1],
            q3: target[2],
        };
        let branches =
            OPWKinematics::wrist_branch(&RotationMatrix::from_quat(pose.rotation), arm, &near);
        for wrist in branches {
            assert!(wrist.q4.abs() <= 4.0 * PI && wrist.q6.abs() <= 4.0 * PI);
            assert_pose(
                &robot,
                &[arm.q1, arm.q2, arm.q3, wrist.q4, wrist.q5, wrist.q6],
                &pose,
            );
        }
    }
}

#[test]
fn wrist_recovery_uses_model_angles_with_offsets_and_opposite_signs() {
    let mut parameters = model_parameters();
    parameters.offsets = [0.2, -0.3, 0.4, 0.45, -0.6, -0.35];
    parameters.sign_corrections = [-1, 1, -1, -1, -1, 1];
    let robot = OPWKinematics::new(parameters);
    for q5 in [0.0, PI, -PI] {
        let target = from_model([0.4, 0.6, -0.3, 0.7, q5, -0.2], &parameters);
        let previous = from_model([0.4, 0.6, -0.3, 0.2, q5, -0.4], &parameters);
        let expected_model = if q5 == 0.0 {
            [0.4, 0.6, -0.3, 0.55, q5, -0.05]
        } else {
            [0.4, 0.6, -0.3, 0.35, q5, -0.55]
        };
        let pose = robot.forward(&target);
        let solutions = robot.inverse_continuing(&pose, &previous);
        assert_contains_joints(&solutions, &from_model(expected_model, &parameters));
        for solution in &solutions {
            assert_pose(&robot, solution, &pose);
        }
    }
}

#[test]
fn plain_inverse_recovers_poles_near_zero_wrist_reference() {
    let robot = OPWKinematics::new(model_parameters());
    for q5 in [0.0, PI, -PI] {
        let target = [0.4, 0.6, -0.3, 0.7, q5, -0.2];
        let expected = if q5 == 0.0 {
            [0.4, 0.6, -0.3, 0.25, q5, 0.25]
        } else {
            [0.4, 0.6, -0.3, 0.45, q5, -0.45]
        };
        let pose = robot.forward(&target);
        let solutions = robot.inverse(&pose);
        assert_contains_joints(&solutions, &expected);
        for solution in &solutions {
            assert_pose(&robot, solution, &pose);
        }
    }
}

#[test]
fn inverse_and_centered_continuation_use_constraint_wrist_centers() {
    let mut lower = [0.0; 6];
    let mut upper = [0.0; 6];
    lower[J4] = -0.8;
    upper[J4] = 1.2;
    lower[J6] = -1.4;
    upper[J6] = 0.6;
    let constraints = Constraints::new(lower, upper, BY_PREV);
    let robot = OPWKinematics::new_with_constraints(model_parameters(), constraints);
    for q5 in [0.0, PI, -PI] {
        let target = [0.4, 0.6, -0.3, 0.7, q5, -0.2];
        let expected = if q5 == 0.0 {
            [0.4, 0.6, -0.3, 0.55, q5, -0.05]
        } else {
            [0.4, 0.6, -0.3, 0.35, q5, -0.55]
        };
        let pose = robot.forward(&target);
        for solutions in [
            robot.inverse(&pose),
            robot.inverse_continuing(&pose, &CONSTRAINT_CENTERED),
        ] {
            assert_contains_joints(&solutions, &expected);
            for solution in &solutions {
                assert!(constraints.compliant(solution));
                assert_pose(&robot, solution, &pose);
            }
        }
    }
}

#[test]
fn constrained_zero_pole_projects_onto_feasible_wrist_continuum() {
    let constraints = Constraints::from_degrees(
        [
            9.0..=11.0,
            19.0..=21.0,
            29.0..=31.0,
            5.0..=15.0,
            -1.0..=1.0,
            60.0..=100.0,
        ],
        BY_PREV,
    );
    let robot = OPWKinematics::new_with_constraints(Parameters::irb2400_10(), constraints);
    let target = [10.0, 20.0, 30.0, 40.0, 0.0, 65.0].map(f64::to_radians);
    let pose = robot.forward(&target);
    // The phase is 105 degrees. Equal splitting from the constraint centers
    // gives (17.5, 87.5), outside J4's limits. The nearest feasible pair is
    // (15, 90); an interior pair establishes reachability independently.
    let feasible = [10.0, 20.0, 30.0, 14.0, 0.0, 91.0].map(f64::to_radians);
    assert!(constraints.compliant(&feasible));
    assert_pose(&robot, &feasible, &pose);
    let expected = [10.0, 20.0, 30.0, 15.0, 0.0, 90.0].map(f64::to_radians);
    for solutions in [
        robot.inverse(&pose),
        robot.inverse_continuing(&pose, &CONSTRAINT_CENTERED),
        robot.inverse_continuing(&pose, &constraints.centers),
    ] {
        assert_contains_joints(&solutions, &expected);
        assert_contains_joints(&solutions[..1], &expected);
        for solution in &solutions {
            assert!(constraints.compliant(solution));
            assert_pose(&robot, solution, &pose);
        }
    }

    // Explicit continuation must use its reference rather than the centers,
    // and retain the previous turn count on every joint.
    let previous = [370.0, -340.0, 750.0, 726.0, 360.0, -622.0].map(f64::to_radians);
    let expected = [370.0, -340.0, 750.0, 726.5, 360.0, -621.5].map(f64::to_radians);
    let solutions = robot.inverse_continuing(&pose, &previous);
    let best = solutions.first().expect("reachable constrained zero pole");
    for (actual, expected) in best.iter().zip(expected) {
        assert!((actual - expected).abs() < 1e-7, "got {best:?}");
    }
    for solution in &solutions {
        assert!(constraints.compliant(solution));
        assert_pose(&robot, solution, &pose);
    }
}

#[test]
fn constrained_pi_poles_project_onto_feasible_wrist_difference() {
    let constraints = Constraints::from_degrees(
        [
            9.0..=11.0,
            19.0..=21.0,
            29.0..=31.0,
            5.0..=15.0,
            179.0..=181.0,
            60.0..=100.0,
        ],
        BY_PREV,
    );
    let robot = OPWKinematics::new_with_constraints(Parameters::irb2400_10(), constraints);
    for pole_degrees in [180.0, -180.0] {
        let target = [10.0, 20.0, 30.0, 40.0, pole_degrees, 125.0].map(f64::to_radians);
        let pose = robot.forward(&target);
        // The difference changes from -70 to -85 degrees. Equal splitting
        // gives (2.5, 87.5), below J4's lower bound; clamp along the pole line.
        let expected = [10.0, 20.0, 30.0, 5.0, pole_degrees, 90.0].map(f64::to_radians);
        let feasible = [10.0, 20.0, 30.0, 6.0, pole_degrees, 91.0].map(f64::to_radians);
        assert!(constraints.compliant(&feasible));
        assert_pose(&robot, &feasible, &pose);
        for solutions in [
            robot.inverse(&pose),
            robot.inverse_continuing(&pose, &CONSTRAINT_CENTERED),
        ] {
            assert_contains_joints(&solutions, &expected);
            assert_contains_joints(&solutions[..1], &expected);
            for solution in &solutions {
                assert!(constraints.compliant(solution));
                assert_pose(&robot, solution, &pose);
            }
        }

        // An explicit reference inside the limits has a different optimum.
        let previous = [10.0, 20.0, 30.0, 12.0, pole_degrees, 99.0].map(f64::to_radians);
        let expected = [10.0, 20.0, 30.0, 13.0, pole_degrees, 98.0].map(f64::to_radians);
        let solutions = robot.inverse_continuing(&pose, &previous);
        let best = solutions.first().expect("reachable constrained pi pole");
        for (actual, expected) in best.iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-7, "got {best:?}");
        }
        for solution in &solutions {
            assert!(constraints.compliant(solution));
            assert_pose(&robot, solution, &pose);
        }
    }
}

#[test]
fn constrained_pole_intersects_wrapped_ranges_and_preserves_turns() {
    let constraints = Constraints::from_degrees(
        [
            9.0..=11.0,
            19.0..=21.0,
            29.0..=31.0,
            175.0..=-175.0,
            -1.0..=1.0,
            350.0..=30.0,
        ],
        BY_PREV,
    );
    let robot = OPWKinematics::new_with_constraints(Parameters::irb2400_10(), constraints);
    let target = [10.0, 20.0, 30.0, 210.0, 0.0, -5.0].map(f64::to_radians);
    let pose = robot.forward(&target);
    // The centers are (180, 370), so the unconstrained projection (187.5,
    // 377.5) falls beyond J4's wrapped arc. Its upper endpoint is 185 degrees.
    let expected = [10.0, 20.0, 30.0, 185.0, 0.0, 380.0].map(f64::to_radians);
    let feasible = [10.0, 20.0, 30.0, 184.0, 0.0, 21.0].map(f64::to_radians);
    assert!(constraints.compliant(&feasible));
    assert_pose(&robot, &feasible, &pose);
    for solutions in [
        robot.inverse(&pose),
        robot.inverse_continuing(&pose, &CONSTRAINT_CENTERED),
    ] {
        assert_contains_joints(&solutions, &expected);
        assert_contains_joints(&solutions[..1], &expected);
        for solution in &solutions {
            assert!(constraints.compliant(solution));
            assert_pose(&robot, solution, &pose);
        }
    }

    let previous = [10.0, 20.0, 30.0, 539.0, 0.0, -719.0].map(f64::to_radians);
    let expected = [10.0, 20.0, 30.0, 545.0, 0.0, -700.0].map(f64::to_radians);
    let solutions = robot.inverse_continuing(&pose, &previous);
    let best = solutions.first().expect("reachable wrapped wrist limits");
    for (actual, expected) in best.iter().zip(expected) {
        assert!((actual - expected).abs() < 1e-7, "got {best:?}");
    }
    for solution in &solutions {
        assert!(constraints.compliant(solution));
        assert_pose(&robot, solution, &pose);
    }
}

#[test]
fn constrained_pole_projects_with_offsets_and_opposite_joint_signs() {
    let mut parameters = model_parameters();
    parameters.offsets = [0.2, -0.3, 0.4, 0.45, -0.6, -0.35];
    parameters.sign_corrections = [-1, 1, -1, -1, -1, 1];
    for pole_degrees in [0.0, 180.0, -180.0] {
        let lower = from_model(
            [9.0, 19.0, 29.0, 5.0, pole_degrees - 1.0, 60.0].map(f64::to_radians),
            &parameters,
        );
        let upper = from_model(
            [11.0, 21.0, 31.0, 15.0, pole_degrees + 1.0, 100.0].map(f64::to_radians),
            &parameters,
        );
        // Joint limits are expressed in user coordinates. Negative signs
        // reverse both endpoints when converting these short model arcs.
        let constraints = Constraints::new(
            std::array::from_fn(|i| lower[i].min(upper[i])),
            std::array::from_fn(|i| lower[i].max(upper[i])),
            BY_PREV,
        );
        let robot = OPWKinematics::new_with_constraints(parameters, constraints);
        let (target_j6, expected_j4) = if pole_degrees == 0.0 {
            (65.0, 15.0)
        } else {
            (125.0, 5.0)
        };
        let target = from_model(
            [10.0, 20.0, 30.0, 40.0, pole_degrees, target_j6].map(f64::to_radians),
            &parameters,
        );
        let expected = from_model(
            [10.0, 20.0, 30.0, expected_j4, pole_degrees, 90.0].map(f64::to_radians),
            &parameters,
        );
        let pose = robot.forward(&target);
        for solutions in [
            robot.inverse(&pose),
            robot.inverse_continuing(&pose, &CONSTRAINT_CENTERED),
        ] {
            assert_contains_joints(&solutions, &expected);
            assert_contains_joints(&solutions[..1], &expected);
            for solution in &solutions {
                assert!(constraints.compliant(solution));
                assert_pose(&robot, solution, &pose);
            }
        }

        let turns = [1.0, -1.0, 2.0, -2.0, 1.0, -1.0];
        let previous = std::array::from_fn(|i| constraints.centers[i] + turns[i] * 2.0 * PI);
        let expected: Joints = std::array::from_fn(|i| expected[i] + turns[i] * 2.0 * PI);
        let solutions = robot.inverse_continuing(&pose, &previous);
        let best = solutions
            .first()
            .expect("reachable transformed wrist limits");
        for (actual, expected) in best.iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-7, "got {best:?}");
        }
        for solution in &solutions {
            assert!(constraints.compliant(solution));
            assert_pose(&robot, solution, &pose);
        }
    }
}

#[test]
fn constrained_poles_reject_disjoint_wrist_phase_ranges() {
    for (pole_degrees, target_j6) in [(0.0, 80.0), (180.0, 140.0)] {
        let constraints = Constraints::from_degrees(
            [
                9.0..=11.0,
                19.0..=21.0,
                29.0..=31.0,
                5.0..=15.0,
                pole_degrees - 1.0..=pole_degrees + 1.0,
                60.0..=100.0,
            ],
            BY_PREV,
        );
        let robot = OPWKinematics::new_with_constraints(Parameters::irb2400_10(), constraints);
        let target = [10.0, 20.0, 30.0, 40.0, pole_degrees, target_j6].map(f64::to_radians);
        let pose = robot.forward(&target);
        // The required sum 120 exceeds the largest permitted sum 115;
        // the required difference -100 is below the minimum difference -95.
        for solutions in [
            robot.inverse(&pose),
            robot.inverse_continuing(&pose, &CONSTRAINT_CENTERED),
            robot.inverse_continuing(&pose, &target),
        ] {
            assert!(solutions.is_empty(), "unexpected solution: {solutions:?}");
        }
    }
}

#[test]
fn constrained_poles_keep_single_feasible_boundary_point() {
    for (pole_degrees, boundary_j4) in [(0.0, 0.25), (180.0, 0.125), (-180.0, 0.125)] {
        let mut lower = [9.0, 19.0, 29.0, 0.0, pole_degrees - 1.0, 0.0].map(f64::to_radians);
        let mut upper = [11.0, 21.0, 31.0, 0.0, pole_degrees + 1.0, 0.0].map(f64::to_radians);
        // Exactly representable radian wrist bounds avoid rounding in the
        // constraint constructor obscuring the closed-endpoint intersection.
        lower[J4] = 0.125;
        upper[J4] = 0.25;
        lower[J6] = 1.0;
        upper[J6] = 1.5;
        let constraints = Constraints::new(lower, upper, BY_PREV);
        let robot = OPWKinematics::new_with_constraints(Parameters::irb2400_10(), constraints);
        let mut expected = [10.0, 20.0, 30.0, 0.0, pole_degrees, 0.0].map(f64::to_radians);
        expected[J4] = boundary_j4;
        expected[J6] = 1.5;
        assert!(constraints.compliant(&expected));
        let pose = robot.forward(&expected);
        // At sum 1.75 or difference -1.375 the feasible pole line touches both
        // closed wrist bounds at one point, with no interior segment.
        for solutions in [
            robot.inverse(&pose),
            robot.inverse_continuing(&pose, &CONSTRAINT_CENTERED),
        ] {
            assert_contains_joints(&solutions, &expected);
            for solution in &solutions {
                assert!(constraints.compliant(solution));
                assert_pose(&robot, solution, &pose);
            }
        }
    }
}

#[test]
fn constrained_pole_searches_beyond_shortest_phase_correction() {
    let constraints = Constraints::from_degrees(
        [
            9.0..=11.0,
            19.0..=21.0,
            29.0..=31.0,
            0.0..=100.0,
            -1.0..=1.0,
            0.0..=100.0,
        ],
        BY_PREV,
    );
    let robot = OPWKinematics::new_with_constraints(Parameters::irb2400_10(), constraints);
    let target = [10.0, 20.0, 30.0, 40.0, 0.0, 20.0].map(f64::to_radians);
    let previous = [10.0, 20.0, 30.0, 170.0, 0.0, 170.0].map(f64::to_radians);
    let expected = [10.0, 20.0, 30.0, 30.0, 0.0, 30.0].map(f64::to_radians);
    let pose = robot.forward(&target);
    // The shortest phase correction is +80 degrees, whose line has no
    // permitted point. A -280 degree correction reaches the feasible segment;
    // the closest point on that segment splits it equally between J4 and J6.
    let solutions = robot.inverse_continuing(&pose, &previous);
    let best = solutions
        .first()
        .expect("reachable non-shortest pole phase");
    for (actual, expected) in best.iter().zip(expected) {
        assert!((actual - expected).abs() < 1e-7, "got {best:?}");
    }
    for solution in &solutions {
        assert!(constraints.compliant(solution));
        assert_pose(&robot, solution, &pose);
    }
}

#[test]
fn small_resolvable_wrist_bends_keep_individual_angles() {
    let robot = OPWKinematics::new(model_parameters());
    let near = J4J6Near { j4: 0.0, j6: 0.0 };
    for bend in [1e-5, 1e-9] {
        for q5 in [bend, -bend, PI - bend, PI + bend] {
            let target = [0.4, 0.6, -0.3, 0.7, q5, -0.2];
            let pose = robot.forward(&target);
            let arm = ArmBranch {
                q1: target[0],
                q2: target[1],
                q3: target[2],
            };
            let branches =
                OPWKinematics::wrist_branch(&RotationMatrix::from_quat(pose.rotation), arm, &near)
                    .collect::<Vec<_>>();
            assert!(
                branches.iter().any(|wrist| {
                    angle_error(wrist.q4, target[J4]) < 1e-5
                        && angle_error(wrist.q5, target[J5]) < 1e-12
                        && angle_error(wrist.q6, target[J6]) < 1e-5
                }),
                "resolvable q5={q5} was rounded to a pole or lost its wrist angles: {branches:?}"
            );
            for wrist in branches {
                assert_pose(
                    &robot,
                    &[arm.q1, arm.q2, arm.q3, wrist.q4, wrist.q5, wrist.q6],
                    &pose,
                );
            }
        }
    }
}

#[test]
fn small_resolvable_wrist_bends_survive_complete_inverse_recovery() {
    let robot = OPWKinematics::new(model_parameters());
    for bend in [1e-5, 1e-9] {
        for q5 in [bend, -bend, PI - bend, PI + bend] {
            let target = [0.4, 0.6, -0.3, 0.7, q5, -0.2];
            let pose = robot.forward(&target);
            // Prefer a different J4/J6 split so a pole approximation cannot
            // stand in for the regular candidate with the genuine tiny bend.
            let previous = [target[0], target[1], target[2], 0.0, q5, 0.0];
            let solutions = robot.inverse_continuing(&pose, &previous);
            assert!(
                solutions.iter().any(|solution| {
                    solution[..3]
                        .iter()
                        .zip(target[..3].iter())
                        .all(|(&actual, &expected)| angle_error(actual, expected) < 1e-7)
                        && angle_error(solution[J4], target[J4]) < 1e-5
                        && angle_error(solution[J5], target[J5]) < 1e-12
                        && angle_error(solution[J6], target[J6]) < 1e-5
                }),
                "resolvable q5={q5} lost its original arm and wrist branch: {solutions:?}"
            );
            for solution in &solutions {
                assert_pose(&robot, solution, &pose);
            }
        }
    }
}

#[test]
fn five_dof_pole_recovery_preserves_fixed_j6_and_tool_position() {
    let mut parameters = model_parameters();
    parameters.dof = 5;
    let robot = OPWKinematics::new(parameters);
    let fixed_j6 = 0.8;
    for q5 in [0.0, PI, -PI] {
        let target = [0.4, 0.6, -0.3, 0.7, q5, fixed_j6];
        let pose = robot.forward(&target);
        let solutions = robot.inverse_5dof(&pose, fixed_j6);
        assert!(
            solutions.iter().any(|solution| {
                solution[..3]
                    .iter()
                    .zip(target[..3].iter())
                    .all(|(&actual, &expected)| angle_error(actual, expected) < 1e-7)
                    && angle_error(solution[J5], q5) < 1e-7
            }),
            "original arm branch was lost at q5={q5}: {solutions:?}"
        );
        for solution in solutions {
            assert!(solution.iter().all(|angle| angle.is_finite()));
            assert_eq!(solution[J6], fixed_j6);
            assert!(
                (robot.forward(&solution).translation - pose.translation).length()
                    <= robot.distance_tolerance
            );
        }
    }
}

#[test]
fn five_dof_generic_inverse_returns_finite_zero_j6() {
    for j6_sign in [0, 1] {
        let mut parameters = model_parameters();
        parameters.dof = 5;
        // File-loaded models lock J6 with zero; manual models may retain one.
        parameters.sign_corrections[J6] = j6_sign;
        let robot = OPWKinematics::new(parameters);
        for j5_degrees in [0.0, 45.0, 180.0, -180.0] {
            let target = [20.0, 35.0, -25.0, 30.0, j5_degrees, 15.0].map(f64::to_radians);
            let pose = robot.forward(&target);
            let solutions = robot.inverse(&pose);
            assert!(
                !solutions.is_empty(),
                "no solutions with J6 sign {j6_sign}, J5={j5_degrees} degrees"
            );
            for solution in solutions {
                assert!(solution.iter().all(|angle| angle.is_finite()));
                assert_eq!(solution[J6], 0.0);
                assert!(
                    (robot.forward(&solution).translation - pose.translation).length()
                        <= robot.distance_tolerance
                );
            }
        }
    }
}

#[test]
fn five_dof_generic_inverse_filters_out_disallowed_zero_j6() {
    let mut parameters = model_parameters();
    parameters.dof = 5;
    let robot = OPWKinematics::new(parameters);
    let target = [20.0, 35.0, -25.0, 30.0, 45.0, 15.0].map(f64::to_radians);
    let pose = robot.forward(&target);
    assert!(!robot.inverse(&pose).is_empty());

    // The target is within limits, but generic inverse fixes J6 at zero.
    let constraints = Constraints::from_degrees(
        [
            -180.0..=180.0,
            -180.0..=180.0,
            -180.0..=180.0,
            -180.0..=180.0,
            -180.0..=180.0,
            10.0..=20.0,
        ],
        BY_PREV,
    );
    assert!(constraints.compliant(&target));
    let constrained = OPWKinematics::new_with_constraints(parameters, constraints);
    assert!(constrained.inverse(&pose).is_empty());
}

#[test]
fn five_dof_generic_continuation_preserves_turns_sorts_and_filters() {
    let mut parameters = model_parameters();
    parameters.dof = 5;
    let robot = OPWKinematics::new(parameters);
    let previous_degrees = [380.0, 395.0, -385.0, 390.0, 405.0, -700.0];
    let previous = previous_degrees.map(f64::to_radians);
    let target = [20.0, 35.0, -25.0, 32.0, 45.0, 20.0].map(f64::to_radians);
    let pose = robot.forward(&target);
    // J4 must move two degrees; all other joints retain their previous turns.
    let expected_degrees = [380.0, 395.0, -385.0, 392.0, 405.0, -700.0];
    let constraints = Constraints::from_degrees(
        [
            15.0..=25.0,
            30.0..=40.0,
            -30.0..=-20.0,
            25.0..=40.0,
            40.0..=50.0,
            10.0..=30.0,
        ],
        BY_PREV,
    );
    let unconstrained_solutions = robot.inverse_continuing(&pose, &previous);
    assert!(
        unconstrained_solutions
            .iter()
            .any(|solution| !constraints.compliant(solution)),
        "fixture must produce alternative branches outside the joint limits"
    );
    let constrained = OPWKinematics::new_with_constraints(parameters, constraints);
    let constrained_solutions = constrained.inverse_continuing(&pose, &previous);
    assert!(constrained_solutions.len() < unconstrained_solutions.len());
    assert!(
        constrained_solutions
            .iter()
            .all(|solution| constraints.compliant(solution))
    );

    for solutions in [&unconstrained_solutions, &constrained_solutions] {
        let best = solutions.first().expect("the nearest branch remains valid");
        for (actual, expected) in best.map(f64::to_degrees).iter().zip(expected_degrees) {
            assert!(
                (actual - expected).abs() < 1e-7,
                "nearest solution should be {expected_degrees:?} degrees, got {:?}",
                best.map(f64::to_degrees)
            );
        }
        for solution in solutions {
            assert!(solution.iter().all(|angle| angle.is_finite()));
            assert_eq!(solution[J6], previous[J6]);
            for (actual, previous) in solution.iter().zip(previous) {
                assert!((actual - previous).abs() <= PI + 1e-12);
            }
            assert!(
                (robot.forward(solution).translation - pose.translation).length()
                    <= robot.distance_tolerance
            );
        }
    }
}
