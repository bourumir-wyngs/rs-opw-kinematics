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
        let branches = OPWKinematics::wrist_branch(&matrix, arm, &near);

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
        OPWKinematics::wrist_branch(&RotationMatrix::from_quat(pose.rotation), arm, &near);
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
                OPWKinematics::wrist_branch(&RotationMatrix::from_quat(pose.rotation), arm, &near);
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
