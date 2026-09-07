use crate::constraints::{BY_PREV, Constraints};
use crate::kinematic_traits::{J5, J6, Joints, Kinematics};
use crate::kinematics_impl::{
    ANGULAR_TOLERANCE, ArmBranch, J4J6Near, OPWKinematics, RELATIVE_DISTANCE_TOLERANCE,
    RotationMatrix, normalize_near,
};
use crate::parameters::opw_kinematics::Parameters;
use glam::DMat3;
use std::f64::consts::PI;

#[test]
fn constructors_reject_invalid_joint_offsets() {
    let outside = f64::from_bits((2.0 * PI).to_bits() + 1);
    let constraints = Constraints::new([-PI; 6], [PI; 6], BY_PREV);
    for dof in [5, 6] {
        for joint in 0..6 {
            for offset in [
                outside,
                -outside,
                1e20,
                -1e20,
                f64::NAN,
                f64::INFINITY,
                f64::NEG_INFINITY,
            ] {
                let mut parameters = Parameters::irb2400_10();
                parameters.dof = dof;
                parameters.offsets[joint] = offset;
                for result in [
                    std::panic::catch_unwind(|| OPWKinematics::new(parameters)),
                    std::panic::catch_unwind(|| {
                        OPWKinematics::new_with_constraints(parameters, constraints)
                    }),
                ] {
                    assert!(
                        result.is_err(),
                        "accepted invalid J{} offset {offset} for {dof}-DOF robot",
                        joint + 1,
                    );
                }
            }
        }
    }
}

#[test]
fn constructors_accept_inclusive_full_turn_joint_offsets() {
    let constraints = Constraints::new([-PI; 6], [PI; 6], BY_PREV);
    let joints = [0.25, 0.5, 0.75, 1.0, 0.625, 0.125];
    for dof in [5, 6] {
        for joint in 0..6 {
            for offset in [-2.0 * PI, 2.0 * PI] {
                let mut parameters = Parameters::irb2400_10();
                parameters.dof = dof;
                parameters.offsets[joint] = offset;
                for robot in [
                    OPWKinematics::new(parameters),
                    OPWKinematics::new_with_constraints(parameters, constraints),
                ] {
                    assert_eq!(robot.parameters.offsets, parameters.offsets);
                    let pose = robot.forward(&joints);
                    assert!(pose.translation.is_finite() && pose.rotation.is_finite());
                    assert!(
                        !robot.inverse(&pose).is_empty(),
                        "could not recover pose with J{} offset {offset} for {dof}-DOF robot",
                        joint + 1,
                    );
                }
            }
        }
    }
}

fn scale_geometry(mut parameters: Parameters, scale: f64) -> Parameters {
    parameters.a1 *= scale;
    parameters.a2 *= scale;
    parameters.b *= scale;
    parameters.c1 *= scale;
    parameters.c2 *= scale;
    parameters.c3 *= scale;
    parameters.c4 *= scale;
    parameters
}

#[test]
fn constructors_classify_arm_continuum_capabilities_from_geometry() {
    let folded = Parameters {
        a1: 0.0,
        ..Parameters::staubli_tx2_140()
    };
    let offset_forearm = Parameters {
        a2: 0.375,
        c3: 0.5,
        ..folded
    };
    let unequal_offset_arm = Parameters {
        c2: 0.3,
        ..Parameters::staubli_tx40()
    };
    let beyond_base_axis_reach = Parameters { a1: 1.5, ..folded };
    let cases = [
        ("IRB2400", Parameters::irb2400_10(), [true, false, false]),
        (
            "TX2-140",
            Parameters::staubli_tx2_140(),
            [true, true, false],
        ),
        ("TX40", Parameters::staubli_tx40(), [false, true, false]),
        ("folded at base axis", folded, [true, true, true]),
        ("offset forearm", offset_forearm, [true, true, true]),
        (
            "unequal offset arm",
            unequal_offset_arm,
            [false, false, false],
        ),
        (
            "base axis beyond reach",
            beyond_base_axis_reach,
            [false, true, false],
        ),
    ];

    // Constraints and the choice of length units cannot change geometric eligibility.
    let constraints = Constraints::new([0.0; 6], [0.0; 6], BY_PREV);
    for (name, parameters, expected) in cases {
        for scale in [1e-6, 1.0, 1e6] {
            let parameters = scale_geometry(parameters, scale);
            for robot in [
                OPWKinematics::new(parameters),
                OPWKinematics::new_with_constraints(parameters, constraints),
            ] {
                assert_eq!(
                    [robot.j1free, robot.j2free, robot.j1j2free],
                    expected,
                    "unexpected continuum capabilities for {name} at scale {scale}",
                );
            }
        }
    }
}

#[test]
fn arm_continuum_capabilities_allow_roundoff_but_preserve_geometry_differences() {
    let folded = Parameters {
        a1: 0.0,
        ..Parameters::staubli_tx2_140()
    };
    let roundoff_folded = Parameters {
        a1: f64::EPSILON,
        c2: f64::from_bits(folded.c2.to_bits() + 1),
        ..folded
    };
    let unequal_links = Parameters {
        c2: folded.c2 + 1e-8,
        ..folded
    };
    let displaced_shoulder = Parameters { a1: 1e-8, ..folded };
    let nonzero_cylinder = Parameters { b: 1e-15, ..folded };
    for (parameters, expected) in [
        (roundoff_folded, [true, true, true]),
        (unequal_links, [true, false, false]),
        (displaced_shoulder, [true, true, false]),
        (nonzero_cylinder, [false, true, false]),
    ] {
        let robot = OPWKinematics::new(parameters);
        assert_eq!(
            [robot.j1free, robot.j2free, robot.j1j2free],
            expected,
            "unexpected continuum capabilities for {parameters:?}",
        );
    }
}

#[test]
fn distance_tolerance_scales_with_robot_geometry() {
    let parameters = Parameters::irb2400_10();
    let robot = OPWKinematics::new(parameters);
    let expected = 2.395 * RELATIVE_DISTANCE_TOLERANCE;
    assert!(
        (robot.distance_tolerance - expected).abs() <= expected * f64::EPSILON,
        "distance tolerance {} does not match expected {}",
        robot.distance_tolerance,
        expected
    );

    let scale = 1_000_000.0;
    let scaled_parameters = scale_geometry(parameters, scale);
    let scaled_robot = OPWKinematics::new(scaled_parameters);
    let expected_scaled = robot.distance_tolerance * scale;
    assert!(
        (scaled_robot.distance_tolerance - expected_scaled).abs() <= expected_scaled * f64::EPSILON,
        "scaled distance tolerance {} does not match expected {}",
        scaled_robot.distance_tolerance,
        expected_scaled
    );

    let constraints = Constraints::new([0.0; 6], [0.0; 6], BY_PREV);
    let constrained_robot = OPWKinematics::new_with_constraints(scaled_parameters, constraints);
    assert_eq!(
        constrained_robot.distance_tolerance,
        scaled_robot.distance_tolerance
    );
}

#[test]
fn inverse_continuing_scales_singularity_recovery_with_geometry() {
    let parameters = scale_geometry(Parameters::irb2400_10(), 1_000.0);
    let robot = OPWKinematics::new(parameters);
    let previous: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.4];
    let target: Joints = [0.0, 0.1, 0.2, 0.5, 0.0, 0.6];
    let pose = robot.forward(&target);

    let base = robot.inverse(&pose);
    let continuing = robot.inverse_continuing(&pose, &previous);
    assert_eq!(
        continuing.len(),
        base.len(),
        "singularity recovery should use existing wrist branches for scaled geometry"
    );

    let recovered = continuing
        .iter()
        .find(|solution| solution[J5].abs() < ANGULAR_TOLERANCE)
        .expect("expected a recovered J5≈0 solution for scaled geometry");
    let resolved_pose = robot.forward(recovered);
    let translation_error = (resolved_pose.translation - pose.translation).length();
    let angular_error = resolved_pose.angular_distance(pose);
    assert!(
        translation_error <= robot.distance_tolerance,
        "resolved FK translation error {} exceeds scaled tolerance {}",
        translation_error,
        robot.distance_tolerance
    );
    assert!(
        angular_error <= ANGULAR_TOLERANCE,
        "resolved FK angular error {} exceeds tolerance {}",
        angular_error,
        ANGULAR_TOLERANCE
    );
}

#[test]
fn inverse_cross_validation_scales_with_robot_geometry() {
    let parameters = Parameters::irb2400_10();
    let scaled_parameters = scale_geometry(parameters, 1E12);
    let robot = OPWKinematics::new(parameters);
    let scaled_robot = OPWKinematics::new(scaled_parameters);
    let joints: Joints = [0.37, -0.61, 0.83, -1.11, 0.72, 1.39];

    let solutions = robot.inverse(&robot.forward(&joints));
    let scaled_pose = scaled_robot.forward(&joints);
    let scaled_solutions = scaled_robot.inverse(&scaled_pose);
    assert!(
        !solutions.is_empty(),
        "baseline inverse returned no solutions"
    );
    assert_eq!(
        scaled_solutions.len(),
        solutions.len(),
        "inverse solution count changed when the robot geometry was scaled"
    );

    for solution in scaled_solutions {
        let resolved_pose = scaled_robot.forward(&solution);
        let translation_error = (resolved_pose.translation - scaled_pose.translation).length();
        assert!(
            translation_error <= scaled_robot.distance_tolerance,
            "resolved FK translation error {} exceeds scaled tolerance {}",
            translation_error,
            scaled_robot.distance_tolerance
        );
    }
}

#[test]
fn inverse_5dof_cross_validation_scales_with_robot_geometry() {
    let parameters = Parameters::irb2400_10();
    let scaled_parameters = scale_geometry(parameters, 1E12);
    let robot = OPWKinematics::new(parameters);
    let scaled_robot = OPWKinematics::new(scaled_parameters);
    let joints: Joints = [0.37, -0.61, 0.83, -1.11, 0.72, 1.39];

    let solutions = robot.inverse_5dof(&robot.forward(&joints), joints[J6]);
    let scaled_pose = scaled_robot.forward(&joints);
    let scaled_solutions = scaled_robot.inverse_5dof(&scaled_pose, joints[J6]);
    assert!(
        !solutions.is_empty(),
        "baseline 5-DOF inverse returned no solutions"
    );
    assert_eq!(
        scaled_solutions.len(),
        solutions.len(),
        "5-DOF inverse solution count changed when the robot geometry was scaled"
    );

    for solution in scaled_solutions {
        let resolved_translation = scaled_robot.forward(&solution).translation;
        let translation_error = (resolved_translation - scaled_pose.translation).length();
        assert!(
            translation_error <= scaled_robot.distance_tolerance,
            "resolved 5-DOF FK translation error {} exceeds scaled tolerance {}",
            translation_error,
            scaled_robot.distance_tolerance
        );
    }
}

#[test]
fn wrist_branch_handles_cosine_roundoff_at_both_poles() {
    let arm = ArmBranch {
        q1: 0.0,
        q2: 0.0,
        q3: 0.0,
    };
    let near = J4J6Near { j4: 0.0, j6: 0.0 };
    for (rotation, cosine, expected_q5) in [
        (DMat3::IDENTITY, 1.0 + f64::EPSILON, 0.0),
        (DMat3::from_rotation_y(PI), -1.0 - f64::EPSILON, PI),
    ] {
        let mut matrix = RotationMatrix { matrix: rotation };
        matrix.matrix.z_axis.z = cosine;
        for branch in OPWKinematics::wrist_branch(&matrix, arm, &near) {
            assert!(branch.q4.is_finite() && branch.q5.is_finite() && branch.q6.is_finite());
            assert_eq!(branch.q5.abs(), expected_q5);
        }
    }
}

#[test]
fn test_inverse_continuing_large_j6_angles() {
    let robot = OPWKinematics::new(Parameters::irb2400_10());

    let angles_deg: [f64; 10] = [
        -90000.0, -9000.0, -900.0, -90.0, -9.0, 9.0, 90.0, 900.0, 9000.0, 90000.0,
    ];

    for &angle_deg in &angles_deg {
        let j6_rad = angle_deg.to_radians();

        let pose = robot.forward(&[0.0, 0.1, 0.2, 0.3, 0.1, j6_rad]);

        let previous: Joints = [0.0, 0.1, 0.2, 0.3, 0.1, j6_rad];
        let solutions = robot.inverse_continuing(&pose, &previous);

        assert!(
            !solutions.is_empty(),
            "No solutions found for angle {} degrees",
            angle_deg
        );

        let solution_j6 = solutions[0][J6];

        // Normalize near previous angle
        let mut normalized_solution_j6 = solution_j6;
        normalize_near(&mut normalized_solution_j6, previous[J6]);

        let diff = (normalized_solution_j6 - previous[J6]).abs();

        // Allow small epsilon due to floating-point errors
        assert!(
            diff < 1e-6,
            "J6 mismatch for angle {} degrees: difference was {} radians",
            angle_deg,
            diff
        );
    }
}

#[test]
fn test_inverse_continuing_recovers_wrist_at_j5_pi() {
    use crate::kinematic_traits::{J4, J5, J6, Joints, Kinematics};
    use std::f64::consts::PI;

    // Use a known-good robot model; adjust if you prefer a different preset.
    let robot = OPWKinematics::new(Parameters::irb2400_10());

    // Previous configuration with the wrist at the π singularity.
    // For J5 ≈ π, the *orientation* depends on (J4 - J6),
    // and the continuity-preserving update is δ4 = -δ6.
    let previous: Joints = [0.0, 0.1, 0.2, 0.3, PI, -0.8];

    // Create a target pose by moving J4 and J6 in OPPOSITE directions by ±Δ
    // while keeping J5 at π. This changes (J4 - J6) by 2Δ.
    let delta = 0.20_f64;
    let target: Joints = [
        previous[0],
        previous[1],
        previous[2],
        previous[J4] + delta,
        previous[J5], // keep J5 at π
        previous[J6] - delta,
    ];

    // Pose generated from the "target" configuration
    let pose = robot.forward(&target);

    // Baseline: plain IK solutions (no continuity logic)
    let base = robot.inverse(&pose);
    assert!(
        !base.is_empty(),
        "baseline IK returned no solutions for the target pose"
    );

    // Continuation recovers the existing wrist branches near `previous`.
    let cont = robot.inverse_continuing(&pose, &previous);
    assert!(!cont.is_empty(), "inverse_continuing returned no solutions");
    assert_eq!(
        cont.len(),
        base.len(),
        "recovery at J5≈π should preserve the wrist branch count"
    );

    // The recovered wrist solution should sort closest to `previous`.
    let mut best = cont[0];
    for j in 0..6 {
        normalize_near(&mut best[j], previous[j]);
    }

    // Opposite-direction motion relative to previous: δ4 + δ6 ≈ 0
    let d4 = best[J4] - previous[J4];
    let d6 = best[J6] - previous[J6];
    assert!(
        (d4 + d6).abs() < 1e-6,
        "expected opposite-direction update at J5≈π, got δ4={} δ6={}",
        d4,
        d6
    );

    // And (J4 - J6) must match the pose-implied (target) value (mod 2π)
    let mut best_diff = best[J4] - best[J6];
    let target_diff = target[J4] - target[J6];
    normalize_near(&mut best_diff, target_diff);
    assert!(
        (best_diff - target_diff).abs() < 1e-6,
        "q4 - q6 mismatch: got {}, want {}",
        best_diff,
        target_diff
    );
}

#[test]
fn test_inverse_continuing_recovers_wrist_at_j5_zero() {
    use crate::kinematic_traits::{J4, J5, J6, Joints, Kinematics};

    let robot = OPWKinematics::new(Parameters::irb2400_10());

    // At J5 = 0, the orientation depends on J4 + J6. The target changes
    // that sum while `previous` supplies the preferred wrist distribution.
    let previous: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.4];
    let delta = 0.20_f64;
    let target: Joints = [
        previous[0],
        previous[1],
        previous[2],
        previous[J4] + delta,
        previous[J5],
        previous[J6] + delta,
    ];
    let pose = robot.forward(&target);

    let base = robot.inverse(&pose);
    assert!(
        !base.is_empty(),
        "baseline IK returned no solutions for the target pose"
    );

    let cont = robot.inverse_continuing(&pose, &previous);
    assert_eq!(
        cont.len(),
        base.len(),
        "recovery at J5≈0 should preserve the wrist branch count"
    );

    // The continuity solution is the minimum joint-space change from
    // `previous`, so sorting should place it first.
    let mut best = cont[0];
    for j in 0..6 {
        normalize_near(&mut best[j], previous[j]);
    }

    assert!(
        best[J5].abs() < 1e-6,
        "expected a J5≈0 continuity solution, got J5={}",
        best[J5]
    );

    // The remap must move J4 and J6 in the same direction by equal
    // amounts. An opposite-sign update preserves the old sum and fails
    // the final forward-pose validation.
    let d4 = best[J4] - previous[J4];
    let d6 = best[J6] - previous[J6];
    assert!(
        (d4 - d6).abs() < 1e-6,
        "expected same-direction update at J5≈0, got δ4={} δ6={}",
        d4,
        d6
    );

    let mut best_sum = best[J4] + best[J6];
    let target_sum = target[J4] + target[J6];
    normalize_near(&mut best_sum, target_sum);
    assert!(
        (best_sum - target_sum).abs() < 1e-6,
        "q4 + q6 mismatch: got {}, want {}",
        best_sum,
        target_sum
    );

    let resolved_pose = robot.forward(&best);
    let translation_error = (resolved_pose.translation - pose.translation).length();
    let angular_error = resolved_pose.angular_distance(pose);
    assert!(
        translation_error <= robot.distance_tolerance,
        "resolved FK translation error {} exceeds tolerance {}",
        translation_error,
        robot.distance_tolerance
    );
    assert!(
        angular_error <= ANGULAR_TOLERANCE,
        "resolved FK angular error {} exceeds tolerance {}",
        angular_error,
        ANGULAR_TOLERANCE
    );
}

#[test]
fn test_inverse_continuing_handles_non_finite_previous_joint() {
    let robot = OPWKinematics::new(Parameters::irb2400_10());
    let pose = robot.forward(&[0.0, 0.1, 0.2, 0.3, 0.1, 0.2]);
    let previous: Joints = [0.0, 0.1, 0.2, 0.3, 0.1, f64::INFINITY];

    assert!(robot.inverse_continuing(&pose, &previous).is_empty());
}
