#[cfg(test)]
mod tests {
    use std::f64::consts::PI;
    use nalgebra::{UnitQuaternion, Vector3};
    use crate::kinematic_traits::{Kinematics, Singularity};
    use crate::parameters::opw_kinematics::Parameters;
    use crate::kinematics_impl::OPWKinematics;
    use crate::tests::test_utils;

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
        assert!(result.is_ok(), "Failed to load or parse the YAML file: {}", result.unwrap_err());
        let cases = result.expect("Expected a valid Cases struct after parsing");
        let all_parameters = test_utils::create_parameter_map();
        println!("Forward IK: {} test cases", cases.len());

        for case in cases.iter() {
            let parameters = all_parameters.get(&case.parameters).unwrap_or_else(|| {
                panic!("Parameters for the robot [{}] are unknown", &case.parameters)
            });
            let kinematics = OPWKinematics::new(parameters.clone());

            // Try forward on the initial data set first.
            let ik = kinematics.forward(&case.joints_in_radians());
            let pose = test_utils::Pose::from_isometry(&ik);

            if !test_utils::are_isometries_approx_equal(&ik, &case.pose.to_isometry(), 0.00001) {
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
        assert!(result.is_ok(), "Failed to load or parse the YAML file: {}", result.unwrap_err());
        let cases = result.expect("Expected a valid Cases struct after parsing");
        let all_parameters = test_utils::create_parameter_map();
        println!("Forward IK: {} test cases", cases.len());

        for case in cases.iter() {
            let parameters = all_parameters.get(&case.parameters).unwrap_or_else(|| {
                panic!("Parameters for the robot [{}] are unknown", &case.parameters)
            });
            let kinematics = OPWKinematics::new(parameters.clone());

            // This test only checks the final pose so far.
            let ik = kinematics.forward_with_joint_poses(&case.joints_in_radians())[5];
            let pose = test_utils::Pose::from_isometry(&ik);

            if !test_utils::are_isometries_approx_equal(&ik, &case.pose.to_isometry(), 0.00001) {
                println!("Seems not equal");
                println!("joints: {:?} ", &case.joints);
                println!("case: {:?} ", &pose);
                println!("IK  : {:?} ", &case.pose);
                println!("{}", parameters.to_yaml());


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
                panic!("Parameters for the robot [{}] are unknown", &case.parameters)
            });
            let kinematics = OPWKinematics::new(parameters.clone());

            // Exclude singularity cases that are covered by another test
            if kinematics.kinematic_singularity(&case.joints_in_radians()).is_none() {
                // Try forward on the initial data set first.
                let solutions = kinematics.inverse(&case.pose.to_isometry());
                if test_utils::found_joints_approx_equal(&solutions, &case.joints_in_radians(),
                                                         0.001_f64.to_radians()).is_none() {
                    println!("**** No valid solution for case {} on {} ****", case.id, case.parameters);
                    let joints_str = &case.joints.iter()
                        .map(|&val| format!("{:5.2}", val))
                        .collect::<Vec<String>>()
                        .join(" ");
                    println!("Expected joints: [{}]", joints_str);

                    println!("Solutions Matrix:");
                    for sol_idx in 0..solutions.len() {
                        let mut row_str = String::new();
                        for joint_idx in 0..6 {
                            let computed = solutions[sol_idx][joint_idx];
                            row_str.push_str(&format!("{:5.2} ", computed.to_degrees()));
                        }
                        println!("[{}]", row_str.trim_end());
                    }

                    println!("---");
                    panic!("Inverse kinematics does not produce valid solution");
                }
            }
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
            if case.id != 1241 {
                //continue;
            }
            let parameters = all_parameters.get(&case.parameters).unwrap_or_else(|| {
                panic!("Parameters for the robot [{}] are unknown", &case.parameters)
            });
            let kinematics = OPWKinematics::new(parameters.clone());
            let solutions = kinematics.inverse_continuing(
                &case.pose.to_isometry(), &case.joints_in_radians());
            let found_matching =
                test_utils::found_joints_approx_equal(&solutions, &case.joints_in_radians(),
                                                      0.001_f64.to_radians());
            if !matches!(found_matching, Some(0)) {
                println!("**** No valid solution: {:?} for case {} on {} ****",
                         found_matching, case.id, case.parameters);
                let joints_str = &case.joints.iter()
                    .map(|&val| format!("{:5.2}", val))
                    .collect::<Vec<String>>()
                    .join(" ");
                println!("Expected joints: [{}]", joints_str);

                println!("Solutions Matrix:");
                for sol_idx in 0..solutions.len() {
                    let mut row_str = String::new();
                    for joint_idx in 0..6 {
                        let computed = solutions[sol_idx][joint_idx];
                        row_str.push_str(&format!("{:5.2} ", computed.to_degrees()));
                    }
                    println!("[{}]", row_str.trim_end());
                }

                println!("---");
            }
            assert!(matches!(found_matching, Some(0)),
                    "Fully matching joints must come first. At {}, Expected Some(0), got {:?}",
                    case.id, found_matching);
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
                panic!("Parameters for the robot [{}] are unknown", &case.parameters)
            });
            let kinematics = OPWKinematics::new(parameters.clone());

            // Use translation instead of full pose
            let isometry = case.pose.to_isometry();
            let solutions = kinematics.inverse_continuing_5dof(
                &isometry, &case.joints_in_radians());
            assert!(solutions.len() > 0);

            for solution in solutions {
                // Check if TCP stays in the same location
                let reconstructed = kinematics.forward(&solution);
                let reconstructed_translation: Vector3<f64> = reconstructed.translation.vector;
                let expected_translation: Vector3<f64> = isometry.translation.vector;
                let translation_diff = (reconstructed_translation - expected_translation).norm();
                assert!(
                    translation_diff < 1E-6,
                    "Reconstructed translation does not match. Diff: {}", translation_diff
                );
            }
        }
    }

    #[test]
    fn test_singularity_a_continuing() {
        // This robot has both A and B type singularity
        // B type singularity two angles, maestro
        let parameters = Parameters::staubli_tx2_160l();
        let kinematics = OPWKinematics::new(parameters.clone());
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
        let joints_str = &joints.iter()
            .map(|&val| format!("{:5}", val))
            .collect::<Vec<String>>()
            .join(" ");
        println!("Joints joints: [{}]", joints_str);

        println!("Solutions:");
        for sol_idx in 0..solutions.len() {
            let mut row_str = String::new();
            for joint_idx in 0..6 {
                let computed = solutions[sol_idx][joint_idx];
                row_str.push_str(&format!("{:5.2} ", computed.to_degrees()));
            }
            println!("{}. [{}]", sol_idx, row_str.trim_end());
        }

        // Make sure singularity is found and included
        let found_matching =
            test_utils::found_joints_approx_equal(&solutions, &joints_in_radians, 0.001_f64.to_radians());
        assert!(matches!(found_matching, Some(0)),
                "Fully matching joints must come first. Expected Some(0), got {:?}", found_matching);
    }

    #[test]
    fn test_singularity_a() {
        // Assuming joint[4] close to π triggers A type singularity
        let robot = OPWKinematics::new(Parameters::irb2400_10());
        assert_eq!(robot.kinematic_singularity(&[0.0, 0.8, 0.0, 0.0, PI, 0.0]).unwrap(),
                   Singularity::A);
        assert_eq!(robot.kinematic_singularity(&[0.0, 0.8, 0.0, 0.0, -PI, 0.0]).unwrap(),
                   Singularity::A);
        assert_eq!(robot.kinematic_singularity(&[0.0, 0.8, 0.0, 0.0, 0.0, PI]).unwrap(),
                   Singularity::A);
        assert_eq!(robot.kinematic_singularity(&[0.0, 0.8, 0.0, 0.0, 3. * PI, 0.0]).unwrap(),
                   Singularity::A);
    }

    #[test]
    fn test_no_singularity() {
        let robot = OPWKinematics::new(Parameters::irb2400_10());
        let joints = [0.0, 0.1, 0.2, 0.3, 0.4, PI];
        assert_eq!(robot.kinematic_singularity(&joints), None);
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
        pose.rotation = UnitQuaternion::identity();

        // As this is 5 DOF robot now, J6 comes from "previous"
        let solutions = kinematics.inverse_continuing(&pose, &previous);
        assert!(solutions.len() > 0);
        for solution in &solutions {
            // J6 must be as we passed.
            assert!(f64::abs(0.55 - &solution[5]) < 1E-6);
            // Translation must match
            let reconstructed_translation: Vector3<f64> = kinematics.forward(solution).translation.vector;
            let expected_translation: Vector3<f64> = pose.translation.vector;
            let translation_diff = (reconstructed_translation - expected_translation).norm();
            assert!(
                translation_diff < 1E-6,
                "Reconstructed translation does not match. Diff: {}", translation_diff
            );
        }
    }
}
