#[cfg(test)]
mod tests {
    use crate::kinematic_traits::{Joints, Kinematics, Solutions};
    use crate::kinematics_impl::OPWKinematics;
    use crate::parameters::opw_kinematics::Parameters;
    use crate::tests::test_utils;
    use glam::DQuat;
    use std::f64::consts::PI;

    // Stored solutions have six decimal places in degrees. One decimal unit
    // allows their rounding error plus the much smaller IK roundoff.
    const FIXTURE_JOINT_TOLERANCE: f64 = 1e-6 * PI / 180.0;
    // The reference implementation's acos can store an exact pole as
    // 179.999999 degrees, so pole classification and J5 matching need two units.
    const FIXTURE_POLE_TOLERANCE: f64 = 2.0 * FIXTURE_JOINT_TOLERANCE;

    fn found_fixture_branch(
        solutions: &Solutions,
        expected: &Joints,
        parameters: &Parameters,
    ) -> bool {
        let q5 = expected[4] * parameters.sign_corrections[4] as f64 - parameters.offsets[4];
        if q5.sin().abs() > FIXTURE_POLE_TOLERANCE {
            return test_utils::found_joints_approx_equal(
                solutions,
                expected,
                FIXTURE_JOINT_TOLERANCE,
            )
            .is_some();
        }

        // At poles, require every stored arm/J5 family. The independent C++
        // fixture's J4/J6 splits can be invalid (e.g. case 1241 has a 9-degree
        // orientation error), so they cannot be used as the wrist-phase oracle.
        // assert_fixture_branches checks every returned full pose against the
        // stored target instead, accepting only valid members of that family.
        solutions.iter().any(|solution| {
            let mut family = *expected;
            family[3] = solution[3];
            family[5] = solution[5];
            test_utils::found_joints_approx_equal(
                std::slice::from_ref(solution),
                &family,
                FIXTURE_POLE_TOLERANCE,
            )
            .is_some()
        })
    }

    fn assert_fixture_branches(
        kinematics: &OPWKinematics,
        parameters: &Parameters,
        case: &test_utils::Case,
        solutions: &Solutions,
    ) {
        assert!(
            !case.solutions.is_empty(),
            "No stored branches for case {}",
            case.id
        );
        assert!(
            !solutions.is_empty(),
            "No inverse solution for case {} on {}",
            case.id,
            case.parameters
        );
        let pose = case.pose.as_pose();
        for solution in solutions {
            assert!(
                solution.iter().all(|joint| joint.is_finite())
                    && test_utils::are_poses_close(
                        &kinematics.forward(solution),
                        &pose,
                        1e-6,
                        1e-6,
                    ),
                "Invalid inverse solution for case {} on {}: {:?}",
                case.id,
                case.parameters,
                solution
            );
        }
        for (branch, expected) in case.solutions.iter().enumerate() {
            assert!(
                found_fixture_branch(solutions, &expected.map(f64::to_radians), parameters),
                "Missing stored branch {branch} for case {} on {}: {:?} degrees",
                case.id,
                case.parameters,
                expected
            );
        }
        assert!(
            found_fixture_branch(solutions, &case.joints_in_radians(), parameters),
            "Missing original joint branch for case {} on {}",
            case.id,
            case.parameters
        );
    }

    #[test]
    fn test_load_yaml() {
        let filename = "src/tests/data/cases.yaml";
        let result = test_utils::load_yaml(filename);

        if let Err(e) = &result {
            println!("Error loading or parsing YAML file: {}", e);
        }

        assert!(result.is_ok(), "Failed to load or parse the YAML file");

        let cases = result.expect("Expected a valid Cases struct after parsing");

        // Example assertion: the list of cases should not be empty.
        assert!(!cases.is_empty(), "No cases were loaded from the YAML file");
    }

    #[test]
    fn test_forward_ik() {
        let filename = "src/tests/data/cases.yaml";
        let result = test_utils::load_yaml(filename);
        assert!(
            result.is_ok(),
            "Failed to load or parse the YAML file: {}",
            result.unwrap_err()
        );
        let cases = result.expect("Expected a valid Cases struct after parsing");
        let all_parameters = test_utils::create_parameter_map();
        println!("Forward IK: {} test cases", cases.len());

        for case in cases.iter() {
            let parameters = all_parameters.get(&case.parameters).unwrap_or_else(|| {
                panic!(
                    "Parameters for the robot [{}] are unknown",
                    &case.parameters
                )
            });
            let kinematics = OPWKinematics::new(*parameters);

            // Try forward on the initial data set first.
            let ik = kinematics.forward(&case.joints_in_radians());
            let pose = test_utils::Pose::from_pose(&ik);

            if !test_utils::are_poses_approx_equal(&ik, &case.pose.as_pose(), 0.00001) {
                println!("Seems not equal");
                println!("joints: {:?} ", &case.joints);
                println!("case: {:?} ", &pose);
                println!("IK  : {:?} ", &case.pose);
                println!();

                panic!("Forward kinematics of the primary pose seems not equal");
            }
        }
    }

    #[test]
    fn test_forward_ik_with_joint_poses() {
        let filename = "src/tests/data/cases.yaml";
        let result = test_utils::load_yaml(filename);
        assert!(
            result.is_ok(),
            "Failed to load or parse the YAML file: {}",
            result.unwrap_err()
        );
        let cases = result.expect("Expected a valid Cases struct after parsing");
        let all_parameters = test_utils::create_parameter_map();
        println!("Forward IK: {} test cases", cases.len());

        for case in cases.iter() {
            let parameters = all_parameters.get(&case.parameters).unwrap_or_else(|| {
                panic!(
                    "Parameters for the robot [{}] are unknown",
                    &case.parameters
                )
            });
            let kinematics = OPWKinematics::new(*parameters);

            // This test only checks the final pose so far.
            let joints = case.joints_in_radians();
            let ik = kinematics.forward_with_joint_poses(&joints)[5];
            let pose = test_utils::Pose::from_pose(&ik);

            let case_pose = case.pose.as_pose();
            if !test_utils::are_poses_approx_equal(&ik, &case_pose, 0.00001) {
                println!("Seems not equal for {}", &case.id);
                println!("joints: {:?} ", &case.joints);
                println!("case: {:?} ", &pose);
                println!("IK  : {:?} ", &case.pose);
                println!("{}", parameters.to_yaml());

                println!("Checking for tcp-only condition");
                assert!(test_utils::are_poses_approx_equal(
                    &case_pose,
                    &kinematics.forward(&joints),
                    0.0001
                ));
                println!("Passed, checking for tcp-only result");
                assert!(test_utils::are_poses_approx_equal(
                    &ik,
                    &kinematics.forward(&joints),
                    0.0001
                ));
                println!("Passed??!!");

                panic!("Forward kinematics of the primary pose seems not equal");
            }
        }
    }

    #[test]
    fn test_inverse_ik() {
        let filename = "src/tests/data/cases.yaml";
        let result = test_utils::load_yaml(filename);
        assert!(result.is_ok(), "Failed to load or parse the YAML file");
        let cases = result.expect("Expected a valid Cases struct after parsing");
        let all_parameters = test_utils::create_parameter_map();
        println!("Inverse IK: {} test cases", cases.len());

        for case in cases.iter() {
            let parameters = all_parameters.get(&case.parameters).unwrap_or_else(|| {
                panic!(
                    "Parameters for the robot [{}] are unknown",
                    &case.parameters
                )
            });
            let kinematics = OPWKinematics::new(*parameters);

            let pose = case.pose.as_pose();
            let solutions = kinematics.inverse(&pose);
            assert_fixture_branches(&kinematics, parameters, case, &solutions);
        }
    }

    #[test]
    fn test_inverse_ik_continuing() {
        let filename = "src/tests/data/cases.yaml";
        let result = test_utils::load_yaml(filename);
        assert!(result.is_ok(), "Failed to load or parse the YAML file");
        let cases = result.expect("Expected a valid Cases struct after parsing");
        let all_parameters = test_utils::create_parameter_map();
        println!("Inverse IK: {} test cases", cases.len());

        for case in cases.iter() {
            let parameters = all_parameters.get(&case.parameters).unwrap_or_else(|| {
                panic!(
                    "Parameters for the robot [{}] are unknown",
                    &case.parameters
                )
            });
            let kinematics = OPWKinematics::new(*parameters);
            let solutions =
                kinematics.inverse_continuing(&case.pose.as_pose(), &case.joints_in_radians());
            assert_fixture_branches(&kinematics, parameters, case, &solutions);
            let found_matching = test_utils::found_joints_approx_equal(
                &solutions,
                &case.joints_in_radians(),
                0.001_f64.to_radians(),
            );
            if !matches!(found_matching, Some(0)) {
                println!(
                    "**** No valid solution: {:?} for case {} on {} ****",
                    found_matching, case.id, case.parameters
                );
                let joints_str = &case
                    .joints
                    .iter()
                    .map(|&val| format!("{:5.2}", val))
                    .collect::<Vec<String>>()
                    .join(" ");
                println!("Expected joints: [{}]", joints_str);

                println!("Solutions Matrix:");
                for solution in &solutions {
                    let mut row_str = String::new();
                    for computed in solution {
                        row_str.push_str(&format!("{:5.2} ", computed.to_degrees()));
                    }
                    println!("[{}]", row_str.trim_end());
                }

                println!("---");
            }
            assert!(
                matches!(found_matching, Some(0)),
                "Fully matching joints must come first. At {}, Expected Some(0), got {:?}",
                case.id,
                found_matching
            );
        }
    }

    #[test]
    fn test_inverse_positioning_continuing() {
        let filename = "src/tests/data/cases.yaml";
        let result = test_utils::load_yaml(filename);
        assert!(result.is_ok(), "Failed to load or parse the YAML file");
        let cases = result.expect("Expected a valid Cases struct after parsing");
        let all_parameters = test_utils::create_parameter_map();
        println!("Inverse Positioning IK: {} test cases", cases.len());

        for case in cases.iter() {
            let parameters = all_parameters.get(&case.parameters).unwrap_or_else(|| {
                panic!(
                    "Parameters for the robot [{}] are unknown",
                    &case.parameters
                )
            });
            let kinematics = OPWKinematics::new(*parameters);

            // Use translation instead of full pose
            let pose = case.pose.as_pose();
            let solutions = kinematics.inverse_continuing_5dof(&pose, &case.joints_in_radians());
            assert!(!solutions.is_empty());

            for solution in solutions {
                // Check if TCP stays in the same location
                let reconstructed = kinematics.forward(&solution);
                let reconstructed_translation = reconstructed.translation;
                let expected_translation = pose.translation;
                let translation_diff = (reconstructed_translation - expected_translation).length();
                assert!(
                    translation_diff < 1E-6,
                    "Reconstructed translation does not match. Diff: {}",
                    translation_diff
                );
            }
        }
    }

    #[test]
    fn test_singularity_a_continuing() {
        // This robot has both A and B type singularity
        // B type singularity two angles, maestro
        let parameters = Parameters::staubli_tx2_160l();
        let kinematics = OPWKinematics::new(parameters);
        investigate_singularity_continuing(&kinematics, [10, 20, 30, 40, 0, 60]);
        investigate_singularity_continuing(&kinematics, [10, 20, 30, 0, 0, 60]);
        investigate_singularity_continuing(&kinematics, [10, 20, 30, 0, 0, 0]);
        investigate_singularity_continuing(&kinematics, [10, 20, 30, 40, 0, 0]);
        investigate_singularity_continuing(&kinematics, [10, 20, 30, 40, 180, 60]);
        investigate_singularity_continuing(&kinematics, [10, 20, 30, 40, -180, 60]);
        investigate_singularity_continuing(&kinematics, [10, 20, 30, 41, 0, 59]);
        investigate_singularity_continuing(&kinematics, [15, 25, 25, 39, 0, 60]);
    }

    fn investigate_singularity_continuing(kinematics: &dyn Kinematics, joints: [i32; 6]) {
        let mut joints_in_radians: [f64; 6] = [0.0; 6];
        for (i, &deg) in joints.iter().enumerate() {
            joints_in_radians[i] = deg as f64 * std::f64::consts::PI / 180.0;
        }
        let ik = kinematics.forward(&joints_in_radians);
        let solutions = kinematics.inverse_continuing(&ik, &joints_in_radians);

        println!();
        println!("**** Singularity case ****");
        let joints_str = &joints
            .iter()
            .map(|&val| format!("{:5}", val))
            .collect::<Vec<String>>()
            .join(" ");
        println!("Joints joints: [{}]", joints_str);

        println!("Solutions:");
        for (sol_idx, solution) in solutions.iter().enumerate() {
            let mut row_str = String::new();
            for computed in solution {
                row_str.push_str(&format!("{:5.2} ", computed.to_degrees()));
            }
            println!("{}. [{}]", sol_idx, row_str.trim_end());
        }

        // Make sure singularity is found and included
        let found_matching = test_utils::found_joints_approx_equal(
            &solutions,
            &joints_in_radians,
            0.001_f64.to_radians(),
        );
        assert!(
            matches!(found_matching, Some(0)),
            "Fully matching joints must come first. Expected Some(0), got {:?}",
            found_matching
        );
    }

    #[test]
    fn test_5dof() {
        let mut parameters = Parameters::irb2400_10();
        parameters.dof = 5; // Make it 5 DOF robot
        let kinematics = OPWKinematics::new(parameters);

        let joints = [0.0, 0.1, 0.2, 0.3, 0.4, PI];
        let previous = [0.0, 0.1, 0.2, 0.3, 0.4, 0.55];

        let mut pose = kinematics.forward(&joints);

        // Wipe the rotation
        pose.rotation = DQuat::IDENTITY;

        // As this is 5 DOF robot now, J6 comes from "previous"
        let solutions = kinematics.inverse_continuing(&pose, &previous);
        assert!(!solutions.is_empty());
        for solution in &solutions {
            // J6 must be as we passed.
            assert!(f64::abs(0.55 - &solution[5]) < 1E-6);
            // Translation must match
            let reconstructed_translation = kinematics.forward(solution).translation;
            let expected_translation = pose.translation;
            let translation_diff = (reconstructed_translation - expected_translation).length();
            assert!(
                translation_diff < 1E-6,
                "Reconstructed translation does not match. Diff: {}",
                translation_diff
            );
        }
    }
}
