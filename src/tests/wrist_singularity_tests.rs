use crate::constraints::{BY_CONSTRAINS, BY_PREV, Constraints};
use crate::kinematic_traits::{CONSTRAINT_CENTERED, J2, J3, J4, J5, J6, Joints, Kinematics, Pose};
use crate::kinematics_impl::{
    ANGULAR_TOLERANCE, ArmBranch, J4J6Near, OPWKinematics, RotationMatrix, calculate_distance,
    compare_poses, normalize_near,
};
use crate::parameters::opw_kinematics::Parameters;
use glam::{DQuat, DVec3};
use std::f64::consts::PI;

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

fn assert_five_dof_pose(robot: &OPWKinematics, joints: &Joints, target: &Pose) {
    assert!(joints.iter().all(|joint| joint.is_finite()));
    let actual = robot.forward(joints);
    assert!(
        (actual.translation - target.translation).length() <= robot.distance_tolerance,
        "translation mismatch for {joints:?}"
    );
    assert!(
        (actual.rotation * DVec3::Z - target.rotation * DVec3::Z).length() <= ANGULAR_TOLERANCE,
        "tool-axis mismatch for {joints:?}"
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
            let valid_candidates: Vec<_> = robot
                .inverse_candidates(&pose, &reference, None)
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
fn constrained_pole_uses_original_reference_after_offset_roundtrip() {
    let mut parameters = model_parameters();
    parameters.offsets[J4] = 0.4977127594596098;
    parameters.offsets[J6] = -0.2850598842442724;
    parameters.sign_corrections[J4] = -1;
    let constraints = Constraints::new(
        [
            0.15,
            0.4,
            0.65,
            0.4181030310493927,
            -0.1,
            -0.30075397288156247,
        ],
        [
            0.35,
            0.6,
            0.85,
            0.7904064910935993,
            0.1,
            0.47759475787368677,
        ],
        BY_PREV,
    );
    let target = [0.25, 0.5, 0.75, 0.5769756721919006, 0.0, 0.2666219665660442];
    let previous = [
        0.25,
        0.5,
        0.75,
        0.5402470406450871,
        0.0,
        -0.1961656696032879,
    ];
    assert!(constraints.compliant(&target));
    assert!(constraints.compliant(&previous));

    // The caller's reference is lost by one ULP in user -> model -> user
    // conversion. Pole fitting must check its endpoint against the original
    // reference, since final continuation normalization uses that value too.
    let near = J4J6Near::from_joints(&previous, &parameters);
    let reconstructed_j4 =
        (near.j4 + parameters.offsets[J4]) * parameters.sign_corrections[J4] as f64;
    assert_eq!(reconstructed_j4.to_bits(), previous[J4].to_bits() + 1);

    let robot = OPWKinematics::new_with_constraints(parameters, constraints);
    let pose = robot.forward(&target);
    let solutions = robot.inverse_continuing(&pose, &previous);
    let best = solutions
        .first()
        .expect("reachable pole with original reference");
    // The requested ranking selects the lower J4 boundary. Validating with
    // the reconstructed reference used to let final normalization push it out.
    assert!((best[J4] - constraints.from[J4]).abs() < 1e-12);
    for solution in &solutions {
        assert!(constraints.compliant(solution));
        assert_pose(&robot, solution, &pose);
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
fn constrained_pole_ranking_prefers_170_degrees_over_190_degrees_of_motion() {
    for reversed_wrist in [false, true] {
        let mut parameters = model_parameters();
        if reversed_wrist {
            parameters.offsets = [5.0, -10.0, 15.0, 25.0, -35.0, 45.0].map(f64::to_radians);
            parameters.sign_corrections[J4] = -1;
        }
        for pole_degrees in [0.0, 180.0, -180.0] {
            let sign = if pole_degrees == 0.0 { 1.0 } else { -1.0 };
            let model_lower = [
                9.0,
                19.0,
                29.0,
                -100.0,
                pole_degrees - 1.0,
                if sign > 0.0 { -100.0 } else { -5.0 },
            ]
            .map(f64::to_radians);
            let model_upper = [
                11.0,
                21.0,
                31.0,
                175.0,
                pole_degrees + 1.0,
                if sign > 0.0 { 5.0 } else { 100.0 },
            ]
            .map(f64::to_radians);
            let bound_a = from_model(model_lower, &parameters);
            let bound_b = from_model(model_upper, &parameters);
            let constraints = Constraints::new(
                std::array::from_fn(|i| bound_a[i].min(bound_b[i])),
                std::array::from_fn(|i| bound_a[i].max(bound_b[i])),
                BY_PREV,
            );
            let previous = from_model(
                [10.0, 20.0, 30.0, 0.0, pole_degrees, 0.0].map(f64::to_radians),
                &parameters,
            );
            let witness = from_model(
                [10.0, 20.0, 30.0, 168.0, pole_degrees, sign * 2.0].map(f64::to_radians),
                &parameters,
            );
            let robot = OPWKinematics::new_with_constraints(parameters, constraints);
            let pose = robot.forward(&witness);
            assert!(constraints.compliant(&witness));

            // The shortest phase winding permits 170 degrees of total motion.
            // Squared distance instead prefers splitting the opposite winding
            // into two 95-degree moves, totaling 190 degrees.
            let solutions = robot.inverse_continuing(&pose, &previous);
            let best = solutions
                .first()
                .expect("the interior witness is reachable");
            let movement_degrees = calculate_distance(best, &previous).to_degrees();
            assert!(
                (movement_degrees - 170.0).abs() < 1e-7,
                "pole={pole_degrees}, reversed_wrist={reversed_wrist}: selected {movement_degrees} degrees of movement: {best:?}"
            );
            for solution in &solutions {
                assert!(constraints.compliant(solution));
                assert_pose(&robot, solution, &pose);
            }
        }
    }
}

#[test]
fn constrained_pole_ranking_honors_constraint_centers_and_mixed_weights() {
    for sorting_weight in [0.25, 0.75, BY_CONSTRAINS] {
        for pole_degrees in [0.0, 180.0, -180.0] {
            let sign = if pole_degrees == 0.0 { 1.0 } else { -1.0 };
            let constraints = Constraints::from_degrees(
                [
                    9.0..=11.0,
                    19.0..=21.0,
                    29.0..=31.0,
                    -60.0..=60.0,
                    pole_degrees - 1.0..=pole_degrees + 1.0,
                    if sign > 0.0 {
                        -10.0..=210.0
                    } else {
                        -210.0..=10.0
                    },
                ],
                sorting_weight,
            );
            let robot = OPWKinematics::new_with_constraints(model_parameters(), constraints);
            let previous = [10.0, 20.0, 30.0, 0.0, pole_degrees, 0.0].map(f64::to_radians);
            let witness = [10.0, 20.0, 30.0, 0.0, pole_degrees, sign * 20.0].map(f64::to_radians);
            let pose = robot.forward(&witness);
            assert!(constraints.compliant(&witness));

            // Equal splitting gives (10, +/-10), needlessly moving J4 away
            // from its center. The interior witness (0, +/-20) has the same
            // total motion and is 20 degrees closer to the constraint centers.
            // Compare scores without prescribing a split, including turns in
            // the reference: the sorter uses unwrapped distance to the centers.
            for extra_turns in [false, true] {
                let mut previous = previous;
                let mut witness = witness;
                if extra_turns {
                    previous[J4] += 4.0 * PI;
                    previous[J6] -= sign * 2.0 * PI;
                    for joint in [J4, J6] {
                        normalize_near(&mut witness[joint], previous[joint]);
                    }
                }
                assert!(constraints.compliant(&witness));
                assert_pose(&robot, &witness, &pose);
                let score = |joints: &Joints| {
                    (1.0 - sorting_weight) * calculate_distance(joints, &previous)
                        + sorting_weight * calculate_distance(joints, &constraints.centers)
                };
                let solutions = robot.inverse_continuing(&pose, &previous);
                let best = solutions
                    .first()
                    .expect("the interior witness is reachable");
                assert!(
                    score(best) <= score(&witness) + 1e-7,
                    "pole={pole_degrees}, weight={sorting_weight}, extra_turns={extra_turns}: score {} exceeds feasible witness score {}; selected {best:?}",
                    score(best),
                    score(&witness)
                );
                for solution in &solutions {
                    assert!(constraints.compliant(solution));
                    assert_pose(&robot, solution, &pose);
                }
            }
        }
    }
}

#[test]
fn constrained_pole_ranking_uses_raw_centers_at_normalization_cut() {
    let constraints = Constraints::from_degrees(
        [
            9.0..=11.0,
            19.0..=21.0,
            29.0..=31.0,
            -180.0..=180.0,
            -1.0..=1.0,
            0.0..=360.0,
        ],
        BY_CONSTRAINS,
    );
    let robot = OPWKinematics::new_with_constraints(model_parameters(), constraints);
    let previous = [10.0, 20.0, 30.0, 0.0, 0.0, 0.0].map(f64::to_radians);
    let witness = [10.0, 20.0, 30.0, 1.0, 0.0, 179.0].map(f64::to_radians);
    let pose = robot.forward(&witness);
    assert!(constraints.compliant(&witness));

    // The phase is pi and the J6 center is +pi. Normalizing the exact
    // (0, pi) endpoint near previous instead produces (0, -pi), far from
    // that raw center. Nearby interior points retain J6 just below +pi.
    let solutions = robot.inverse_continuing(&pose, &previous);
    let best = solutions
        .first()
        .expect("the interior witness is reachable");
    assert!(
        calculate_distance(best, &constraints.centers)
            <= calculate_distance(&witness, &constraints.centers) + 1e-7,
        "selected {best:?} despite the better interior witness {witness:?}"
    );
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
fn five_dof_poles_ignore_tool_roll_with_narrow_joint_limits() {
    for dof in [5, 6] {
        for variant in 0..3 {
            let mut parameters = model_parameters();
            parameters.dof = dof;
            if variant == 1 {
                parameters.offsets = [0.05, -0.1, 0.15, 0.2, -0.25, 0.3];
                parameters.sign_corrections = [-1, 1, -1, -1, -1, -1];
            } else if variant == 2 {
                parameters.sign_corrections[J6] = 0;
            }
            for q5 in [0.0, PI, -PI] {
                let target = from_model([0.4, 0.6, -0.3, 0.7, q5, 0.8], &parameters);
                let mut lower = target.map(|joint| joint - 0.1);
                let mut upper = target.map(|joint| joint + 0.1);
                // Keep only the original arm branch and a narrow J4 interval;
                // allow both explicit J6 and the generic inverse's fixed zero.
                lower[J6] = -2.0;
                upper[J6] = 2.0;
                let constraints = Constraints::new(lower, upper, BY_PREV);
                let robot = OPWKinematics::new_with_constraints(parameters, constraints);
                let pose = robot.forward(&target);
                let mut previous = target;
                previous[J4] += 0.03;
                assert!(constraints.compliant(&previous));

                for roll in [-2.0, -0.6, 0.0, 0.9, 2.5] {
                    let mut rolled_pose = pose;
                    // Rotation about the tool's local Z changes only the
                    // ignored roll, leaving the five-axis target identical.
                    rolled_pose.rotation *= DQuat::from_rotation_z(roll);
                    assert_five_dof_pose(&robot, &previous, &rolled_pose);
                    let mut cases = vec![
                        (
                            "explicit inverse",
                            robot.inverse_5dof(&rolled_pose, target[J6]),
                            constraints.centers[J4],
                            target[J6],
                        ),
                        (
                            "explicit continuation",
                            robot.inverse_continuing_5dof(&rolled_pose, &previous),
                            previous[J4],
                            previous[J6],
                        ),
                    ];
                    if dof == 5 {
                        cases.push((
                            "generic inverse",
                            robot.inverse(&rolled_pose),
                            constraints.centers[J4],
                            0.0,
                        ));
                        cases.push((
                            "generic continuation",
                            robot.inverse_continuing(&rolled_pose, &previous),
                            previous[J4],
                            previous[J6],
                        ));
                    }
                    for (api, solutions, expected_j4, fixed_j6) in cases {
                        assert!(
                            !solutions.is_empty(),
                            "{api}: ignored roll {roll} lost the compliant pole \
                             at q5={q5}, dof={dof}, variant={variant}"
                        );
                        assert!(
                            (solutions[0][J4] - expected_j4).abs() < 1e-12,
                            "{api}: ignored roll {roll} changed free J4 from \
                             {expected_j4} to {}, q5={q5}, dof={dof}, variant={variant}",
                            solutions[0][J4],
                        );
                        for solution in solutions {
                            assert!(constraints.compliant(&solution));
                            assert_eq!(solution[J6], fixed_j6);
                            assert_five_dof_pose(&robot, &solution, &rolled_pose);
                        }
                    }
                }
            }
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
fn five_dof_constraint_centered_matches_explicit_centers_with_nonzero_j6() {
    for dof in [5, 6] {
        let mut parameters = Parameters::irb2400_10();
        parameters.dof = dof;
        let constraints = Constraints::from_degrees(
            [
                10.0..=30.0,
                25.0..=45.0,
                -35.0..=-15.0,
                20.0..=40.0,
                35.0..=55.0,
                60.0..=100.0,
            ],
            BY_PREV,
        );
        let centers = constraints.centers;
        let robot = OPWKinematics::new_with_constraints(parameters, constraints);
        // An ordinary pose, with tool roll different from the 80-degree J6 center.
        let target = [20.0, 35.0, -25.0, 30.0, 45.0, 70.0].map(f64::to_radians);
        assert!(constraints.compliant(&target));
        let pose = robot.forward(&target);
        let explicit = robot.inverse_continuing_5dof(&pose, &centers);
        assert!(!explicit.is_empty(), "explicit centers failed, dof={dof}");

        // The sentinel's raw J6 is zero and violates the limits. Both entry
        // points must fix J6 at the resolved center, just like explicit centers.
        let centered = robot.inverse_continuing_5dof(&pose, &CONSTRAINT_CENTERED);
        assert_eq!(centered, explicit, "sentinel differs, dof={dof}");
        if dof == 5 {
            assert_eq!(robot.inverse_continuing(&pose, &centers), explicit);
            assert_eq!(
                robot.inverse_continuing(&pose, &CONSTRAINT_CENTERED),
                explicit
            );
        }
        for solution in centered {
            assert!(solution.iter().all(|angle| angle.is_finite()));
            assert!(constraints.compliant(&solution));
            assert_eq!(solution[J6], centers[J6]);
            let actual = robot.forward(&solution);
            assert!((actual.translation - pose.translation).length() <= robot.distance_tolerance);
            assert!(
                (actual.rotation * DVec3::Z - pose.rotation * DVec3::Z).length()
                    <= ANGULAR_TOLERANCE
            );
        }
    }
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

#[test]
fn six_dof_continuation_rejects_nonfinite_previous_joints() {
    let robot = OPWKinematics::new(Parameters::irb2400_10());
    let target = [0.25, 0.5, 0.75, 1.0, 0.625, 0.125];
    let pose = robot.forward(&target);
    assert!(!robot.inverse_continuing(&pose, &target).is_empty());
    assert!(
        !robot
            .inverse_continuing(&pose, &CONSTRAINT_CENTERED)
            .is_empty()
    );

    // NaN in J1 is the documented sentinel. Other nonfinite reference
    // coordinates cannot identify a nearby finite joint representation.
    for joint in (0..6).rev() {
        for invalid in [f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
            if joint == 0 && invalid.is_nan() {
                continue;
            }
            let mut previous = target;
            previous[joint] = invalid;
            let solutions = robot.inverse_continuing(&pose, &previous);
            assert!(
                solutions.is_empty(),
                "J{}={invalid} returned {solutions:?}",
                joint + 1
            );
        }
    }
}

#[test]
fn six_dof_continuation_validates_final_normalized_pose() {
    let robot = OPWKinematics::new(Parameters::irb2400_10());
    let target = [0.25, 0.5, 0.75, 1.0, 0.625, 0.125];
    let pose = robot.forward(&target);
    for reference in [target, target.map(|joint| joint + 20.0 * PI)] {
        let solutions = robot.inverse_continuing(&pose, &reference);
        assert!(!solutions.is_empty(), "ordinary turns remain recoverable");
        for solution in &solutions {
            assert_pose(&robot, solution, &pose);
        }
    }

    // Adding turns near 1e16 loses the low bits that define the pose. A
    // candidate validated before this normalization must not escape with a
    // changed orientation or translation. No representable solution is required.
    for joint in [J6, J4, J2] {
        for large in [1e16, -1e16] {
            let mut previous = target;
            previous[joint] = large;
            let solutions = robot.inverse_continuing(&pose, &previous);
            for solution in &solutions {
                assert_pose(&robot, solution, &pose);
            }
        }
    }

    // Each restored angle is finite, but their sum overflows inside FK.
    for large in [1e308, -1e308] {
        let mut previous = target;
        previous[J2] = large;
        previous[J3] = large;
        assert!(robot.inverse_continuing(&pose, &previous).is_empty());
    }
}

#[test]
fn five_dof_inverse_rejects_nonfinite_fixed_j6_and_previous_joints() {
    for dof in [5, 6] {
        let mut parameters = Parameters::irb2400_10();
        parameters.dof = dof;
        let robot = OPWKinematics::new(parameters);
        let target = [0.25, 0.5, 0.75, 1.0, 0.625, 0.125];
        let pose = robot.forward(&target);
        assert!(!robot.inverse_5dof(&pose, target[J6]).is_empty());
        assert!(!robot.inverse_continuing_5dof(&pose, &target).is_empty());

        for invalid in [f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
            assert!(
                robot.inverse_5dof(&pose, invalid).is_empty(),
                "fixed J6={invalid}, dof={dof}"
            );
            for joint in 0..6 {
                if joint == 0 && invalid.is_nan() {
                    continue;
                }
                let mut previous = target;
                previous[joint] = invalid;
                assert!(
                    robot.inverse_continuing_5dof(&pose, &previous).is_empty(),
                    "explicit five-axis continuation: J{}={invalid}, dof={dof}",
                    joint + 1
                );
                if dof == 5 {
                    assert!(
                        robot.inverse_continuing(&pose, &previous).is_empty(),
                        "generic five-axis continuation: J{}={invalid}",
                        joint + 1
                    );
                }
            }
        }
    }
}

#[test]
fn five_dof_continuation_validates_final_normalized_pose() {
    for dof in [5, 6] {
        for c4 in [0.0, Parameters::irb2400_10().c4] {
            let mut parameters = Parameters::irb2400_10();
            parameters.dof = dof;
            parameters.c4 = c4;
            let robot = OPWKinematics::new(parameters);
            let target = [0.25, 0.5, 0.75, 1.0, 0.625, 0.125];
            let pose = robot.forward(&target);
            for use_generic in [false, true] {
                if use_generic && dof != 5 {
                    continue;
                }
                let inverse = |previous: &Joints| {
                    if use_generic {
                        robot.inverse_continuing(&pose, previous)
                    } else {
                        robot.inverse_continuing_5dof(&pose, previous)
                    }
                };
                for reference in [target, target.map(|joint| joint + 20.0 * PI)] {
                    let solutions = inverse(&reference);
                    assert!(!solutions.is_empty(), "ordinary turns remain recoverable");
                    for solution in &solutions {
                        assert_eq!(solution[J6], reference[J6]);
                        assert_five_dof_pose(&robot, solution, &pose);
                    }
                }

                // Five-axis validation preserves translation and the tool axis;
                // it must inspect the actual output after J1-J5 normalization.
                // When c4=0, J4 can corrupt direction without moving the tool.
                for joint in [J4, J2] {
                    for large in [1e16, -1e16] {
                        let mut previous = target;
                        previous[joint] = large;
                        for solution in &inverse(&previous) {
                            assert_eq!(solution[J6], previous[J6]);
                            assert_five_dof_pose(&robot, solution, &pose);
                        }
                    }
                }

                // Both continuation entry points must reject FK overflow
                // before constructing a pose, including when c4 is zero.
                for large in [1e308, -1e308] {
                    let mut previous = target;
                    previous[J2] = large;
                    previous[J3] = large;
                    assert!(inverse(&previous).is_empty());
                }
            }
        }
    }
}

#[test]
fn five_dof_inverse_preserves_large_finite_fixed_j6() {
    for dof in [5, 6] {
        let mut parameters = Parameters::irb2400_10();
        parameters.dof = dof;
        let robot = OPWKinematics::new(parameters);
        let target = [0.25, 0.5, 0.75, 1.0, 0.625, 0.125];
        let pose = robot.forward(&target);
        for fixed_j6 in [1e16, -1e16, 1e100, -1e100] {
            let mut previous = target;
            previous[J6] = fixed_j6;
            let mut results = vec![
                robot.inverse_5dof(&pose, fixed_j6),
                robot.inverse_continuing_5dof(&pose, &previous),
            ];
            if dof == 5 {
                results.push(robot.inverse_continuing(&pose, &previous));
            }
            for solutions in results {
                assert!(
                    !solutions.is_empty(),
                    "fixed J6={fixed_j6}, dof={dof} must preserve five-axis reachability"
                );
                for solution in solutions {
                    // J6 is a prescribed tool roll, so no normalization or
                    // full-orientation check may change or reject its value.
                    assert_eq!(solution[J6], fixed_j6);
                    assert_five_dof_pose(&robot, &solution, &pose);
                }
            }
        }
    }
}

fn assert_five_dof_ranking_matches_witness(
    robot: &OPWKinematics,
    constraints: &Constraints,
    pose: &Pose,
    previous: &Joints,
    witness: &Joints,
    context: &str,
) {
    assert!(constraints.compliant(witness), "{context}: invalid witness");
    assert_eq!(witness[J6], previous[J6]);
    assert_five_dof_pose(robot, witness, pose);
    // Compute the public objective independently, retaining raw reference and
    // center coordinates so normalization cuts cannot be hidden by wrapping.
    let score = |joints: &Joints| {
        joints
            .iter()
            .zip(previous.iter().zip(constraints.centers.iter()))
            .map(|(joint, (previous, center))| {
                (1.0 - constraints.sorting_weight) * (joint - previous).abs()
                    + constraints.sorting_weight * (joint - center).abs()
            })
            .sum::<f64>()
    };
    let mut results = vec![(
        "explicit continuation",
        robot.inverse_continuing_5dof(pose, previous),
    )];
    if robot.parameters.dof == 5 {
        results.push((
            "generic continuation",
            robot.inverse_continuing(pose, previous),
        ));
    }
    for (api, solutions) in results {
        let best = solutions
            .first()
            .unwrap_or_else(|| panic!("{context}, {api}: feasible witness was lost"));
        assert!(
            score(best) <= score(witness) + 1e-9,
            "{context}, {api}: score {} exceeds witness score {}; selected {:?} degrees, witness {:?} degrees",
            score(best),
            score(witness),
            best.map(f64::to_degrees),
            witness.map(f64::to_degrees),
        );
        if constraints.sorting_weight == 0.5 {
            assert!(
                (best[J4] - previous[J4]).abs() <= 1e-9,
                "{context}, {api}: tied scores should retain the feasible previous J4"
            );
        }
        for solution in &solutions {
            assert!(constraints.compliant(solution), "{context}, {api}");
            assert_eq!(solution[J6], previous[J6], "{context}, {api}");
            assert_five_dof_pose(robot, solution, pose);
        }
    }
}

#[test]
fn five_dof_free_j4_ranking_honors_previous_centers_and_mixed_weights() {
    for dof in [5, 6] {
        for reversed in [false, true] {
            let mut parameters = model_parameters();
            parameters.dof = dof;
            if reversed {
                parameters.offsets = [5.0, -10.0, 15.0, 25.0, -35.0, 45.0].map(f64::to_radians);
                parameters.sign_corrections = [-1, 1, -1, -1, -1, -1];
            }
            for pole in [0.0, 180.0, -180.0] {
                let lower = from_model(
                    [9.0, 19.0, 29.0, 0.0, pole - 1.0, -1.0].map(f64::to_radians),
                    &parameters,
                );
                let upper = from_model(
                    [11.0, 21.0, 31.0, 100.0, pole + 1.0, 1.0].map(f64::to_radians),
                    &parameters,
                );
                let previous = from_model(
                    [10.0, 20.0, 30.0, 10.0, pole, 0.0].map(f64::to_radians),
                    &parameters,
                );
                let target = from_model(
                    [10.0, 20.0, 30.0, 40.0, pole, 0.0].map(f64::to_radians),
                    &parameters,
                );
                for weight in [BY_PREV, 0.25, 0.5, 0.75, BY_CONSTRAINS] {
                    let constraints = Constraints::new(
                        std::array::from_fn(|i| lower[i].min(upper[i])),
                        std::array::from_fn(|i| lower[i].max(upper[i])),
                        weight,
                    );
                    let robot = OPWKinematics::new_with_constraints(parameters, constraints);
                    let pose = robot.forward(&target);
                    let mut witness = previous;
                    // Below half weight the previous J4 wins; above it the
                    // center wins. At half weight every split between them ties.
                    if weight > 0.5 {
                        witness[J4] = constraints.centers[J4];
                    } else if weight == 0.5 {
                        witness[J4] = (previous[J4] + constraints.centers[J4]) / 2.0;
                    }
                    assert_five_dof_ranking_matches_witness(
                        &robot,
                        &constraints,
                        &pose,
                        &previous,
                        &witness,
                        &format!("dof={dof}, reversed={reversed}, pole={pole}, weight={weight}"),
                    );
                }
            }
        }
    }
}

#[test]
fn five_dof_free_j4_ranking_handles_wrapped_limits_and_previous_turns() {
    let mut parameters = model_parameters();
    parameters.dof = 5;
    for pole in [0.0, 180.0, -180.0] {
        let target = [10.0, 20.0, 30.0, -160.0, pole, 20.0].map(f64::to_radians);
        for extra_turns in [false, true] {
            let mut previous = target;
            previous[J4] = if extra_turns { 545.0_f64 } else { -175.0_f64 }.to_radians();
            if extra_turns {
                previous[J6] += 2.0 * PI;
            }
            for weight in [BY_PREV, 0.25, 0.5, 0.75, BY_CONSTRAINS] {
                let constraints = Constraints::from_degrees(
                    [
                        9.0..=11.0,
                        19.0..=21.0,
                        29.0..=31.0,
                        std::ops::RangeInclusive::new(170.0, -130.0),
                        pole - 1.0..=pole + 1.0,
                        19.0..=21.0,
                    ],
                    weight,
                );
                let robot = OPWKinematics::new_with_constraints(parameters, constraints);
                let pose = robot.forward(&target);
                let mut witness = previous;
                if weight >= 0.5 {
                    // Raw center is +200 degrees. The nearest-turn feasible
                    // interval is [-190,-130] or [530,590], so its preferred
                    // endpoint changes after adding previous turns. Keep the
                    // witness one degree inside to avoid exact-limit rounding.
                    witness[J4] = if extra_turns { 531.0_f64 } else { -131.0_f64 }.to_radians();
                }
                assert_five_dof_ranking_matches_witness(
                    &robot,
                    &constraints,
                    &pose,
                    &previous,
                    &witness,
                    &format!("pole={pole}, extra_turns={extra_turns}, weight={weight}"),
                );
            }
        }
    }
}

#[test]
fn five_dof_free_j4_ranking_uses_raw_center_at_normalization_cut() {
    let mut parameters = model_parameters();
    parameters.dof = 5;
    for pole in [0.0, 180.0, -180.0] {
        let constraints = Constraints::from_degrees(
            [
                9.0..=11.0,
                19.0..=21.0,
                29.0..=31.0,
                0.0..=360.0,
                pole - 1.0..=pole + 1.0,
                19.0..=21.0,
            ],
            BY_CONSTRAINS,
        );
        let robot = OPWKinematics::new_with_constraints(parameters, constraints);
        let previous = [10.0, 20.0, 30.0, 0.0, pole, 20.0].map(f64::to_radians);
        let pose = robot.forward(&previous);
        let mut witness = previous;
        // +pi itself normalizes to -pi near zero. Values just below +pi
        // remain close to the raw +pi center and must beat zero or -pi.
        witness[J4] = PI - 1e-6;
        assert_five_dof_ranking_matches_witness(
            &robot,
            &constraints,
            &pose,
            &previous,
            &witness,
            &format!("normalization cut, pole={pole}"),
        );
    }
}
