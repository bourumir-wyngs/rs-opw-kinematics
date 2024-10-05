use std::fs::File;
use std::io::Read;

#[derive(Debug)]
struct Pose {
    translation: [f64; 3],
    quaternion: [f64; 4], // Assuming [x, y, z, w] ordering here
}

#[derive(Debug)]
struct Case {
    id: i32,
    parameters: String,
    joints: [f64; 6],
    _solutions: Vec<[f64; 6]>,
    pose: Pose,
}

impl Case {
    // Method to return joints in radians
    pub fn joints_in_radians(&self) -> [f64; 6] {
        let mut joints_in_radians = [0.0; 6];
        for (i, &joint) in self.joints.iter().enumerate() {
            joints_in_radians[i] = joint.to_radians();
        }
        joints_in_radians
    }
}

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use yaml_rust2::{Yaml, YamlLoader};

impl Pose {
    pub fn to_isometry(&self) -> Isometry3<f64> {
        let translation = Translation3::new(self.translation[0], self.translation[1], self.translation[2]);

        // Adjusting quaternion creation to match [x, y, z, w] ordering
        let quaternion = UnitQuaternion::from_quaternion(Quaternion::new(
            self.quaternion[3], // w
            self.quaternion[0], // x
            self.quaternion[1], // y
            self.quaternion[2], // z
        ));

        Isometry3::from_parts(translation, quaternion)
    }

    pub fn from_isometry(isometry: &Isometry3<f64>) -> Self {
        let translation = isometry.translation.vector;

        // Extract the quaternion from isometry (rotation part)
        let quaternion = isometry.rotation.quaternion();

        Pose {
            translation: [translation.x, translation.y, translation.z],
            quaternion: [quaternion.i, quaternion.j, quaternion.k, quaternion.w], // [x, y, z, w] ordering
        }
    }
}

fn load_yaml(file_path: &str) -> Result<Vec<Case>, String> {
    let mut file = File::open(file_path).map_err(|e| e.to_string())?;
    let mut contents = String::new();
    file.read_to_string(&mut contents).map_err(|e| e.to_string())?;

    let docs = YamlLoader::load_from_str(&contents).map_err(|e| e.to_string())?;
    let cases_yaml = &docs[0]["cases"];

    let mut cases: Vec<Case> = Vec::new();
    if let Some(cases_vec) = cases_yaml.as_vec() {
        for case_yaml in cases_vec {
            let id = case_yaml["id"].as_i64().ok_or("Missing id")? as i32;
            let parameters = case_yaml["parameters"].as_str().ok_or("Missing parameters")?.to_string();
            let joints = parse_array::<f64, 6>(&case_yaml["joints"])?;
            let solutions = parse_solutions(&case_yaml["solutions"])?;
            let pose = parse_pose(&case_yaml["pose"])?;

            cases.push(Case {
                id,
                parameters,
                joints,
                _solutions: solutions,
                pose,
            });
        }
    } else {
        return Err("Expected 'cases' to be a sequence".to_string());
    }

    Ok(cases)
}

fn parse_array<T: Copy + std::str::FromStr, const N: usize>(yaml: &Yaml) -> Result<[T; N], String>
where
    <T as std::str::FromStr>::Err: std::fmt::Display, // Ensure the error type of T's FromStr can be displayed
{
    let vec = yaml.as_vec().ok_or("Expected an array in YAML")?;

    if vec.is_empty() {
        return Err("Array in YAML is empty, cannot initialize".to_string());
    }

    // Initialize the array using the first element if it can be parsed, otherwise return an error
    let first_value = vec.get(0)
        .ok_or_else(|| "Array is non-empty but no first item found".to_string())
        .and_then(|item| {
            match item {
                Yaml::Real(s) => s.parse::<T>()
                    .map_err(|e| format!("Failed to parse first item as real: {}, found: '{}'", e, s)),
                Yaml::Integer(i) => i.to_string().parse::<T>()
                    .map_err(|e| format!("Failed to parse first item as integer: {}, found: '{}'", e, i)),
                _ => Err(format!("First item is not a real or integer value, found: {:?}", item))
            }
        })?;

    let mut array: [T; N] = [first_value; N];  // Use the first parsed value to initialize the array

    // Parse each element in the vector and fill the array
    for (i, item) in vec.iter().enumerate() {
        array[i] = match item {
            Yaml::Real(s) => s.parse::<T>()
                .map_err(|e| format!("Error parsing real at index {}: {}, found: '{}'", i, e, s))?,
            Yaml::Integer(i) => i.to_string()
                .parse::<T>().map_err(|e| format!("Error parsing integer at index {}: {}, found: '{}'", i, e, i))?,
            _ => return Err(format!("Expected a real or integer value at index {}, found: {:?}", i, item))
        };
    }

    Ok(array)
}

fn parse_solutions(yaml: &Yaml) -> Result<Vec<[f64; 6]>, String> {
    let mut solutions = Vec::new();
    for solution_yaml in yaml.as_vec().ok_or("Expected solutions array")? {
        solutions.push(parse_array::<f64, 6>(solution_yaml)?);
    }
    Ok(solutions)
}

fn parse_pose(yaml: &Yaml) -> Result<Pose, String> {
    let translation = parse_array::<f64, 3>(&yaml["translation"])?;
    let quaternion = parse_array::<f64, 4>(&yaml["quaternion"])?;

    Ok(Pose {
        translation,
        quaternion,
    })
}

fn are_isometries_approx_equal(a: &Isometry3<f64>, b: &Isometry3<f64>, tolerance: f64) -> bool {
    let translation_diff = a.translation.vector - b.translation.vector;
    if translation_diff.norm() > tolerance {
        return false;
    }

    // Check if the rotation components are approximately equal
    // This part is a bit more complex due to quaternion properties.
    // One way is to calculate the angle between the two quaternions and see if it's within the tolerance.
    // This involves converting the unit quaternion difference into an angle.
    let rotation_diff = a.rotation.inverse() * b.rotation;
    let angle = rotation_diff.angle();

    angle.abs() <= tolerance
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use std::f64::consts::PI;
    use std::sync::Arc;
    use nalgebra::Vector3;
    use crate::kinematic_traits::{Joints, Kinematics, Singularity, Solutions};
    use crate::parameters::opw_kinematics::Parameters;
    use crate::kinematics_impl::OPWKinematics;
    use crate::tool::{Base, Tool};
    use crate::utils::{dump_joints, dump_solutions_degrees};
    use super::*;

    /// Check if 'expected' exists in the given vector of solutions. This function is also
    /// used by other tests.
    pub fn found_joints_approx_equal(solutions: &Solutions, expected: &Joints, tolerance: f64) -> Option<i32> {
        for sol_idx in 0..solutions.len() {
            // println!("Checking solution at index {}", sol_idx);

            let mut solution_matches = true;
            for joint_idx in 0..6 {
                let computed = solutions[sol_idx][joint_idx];
                let asserted = expected[joint_idx];

                let diff = (computed - asserted).abs();
                //println!("Column value: {}, Expected value: {}, Difference: {}",
                //         computed, asserted, diff);

                if diff >= tolerance && (diff - 2. * PI).abs() > tolerance {
                    // For angles, 360 degree difference means the same angle.
                    solution_matches = false;
                    break;
                }
            }

            if solution_matches {
                return Some(sol_idx as i32); // Return the index of the matching solution
            }
        }

        println!("No matching solution found");
        return None; // Explicitly indicate that no matching column was found
    }

    #[test]
    fn test_load_yaml() {
        let filename = "src/tests/data/cases.yaml";
        let result = load_yaml(filename);

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
        let result = load_yaml(filename);
        assert!(result.is_ok(), "Failed to load or parse the YAML file: {}", result.unwrap_err());
        let cases = result.expect("Expected a valid Cases struct after parsing");
        let all_parameters = create_parameter_map();
        println!("Forward IK: {} test cases", cases.len());

        for case in cases.iter() {
            let parameters = all_parameters.get(&case.parameters).unwrap_or_else(|| {
                panic!("Parameters for the robot [{}] are unknown", &case.parameters)
            });
            let kinematics = OPWKinematics::new(parameters.clone());

            // Try forward on the initial data set first.
            let ik = kinematics.forward(&case.joints_in_radians());
            let pose = Pose::from_isometry(&ik);

            if !are_isometries_approx_equal(&ik, &case.pose.to_isometry(), 0.00001) {
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
    fn test_inverse_ik() {
        let filename = "src/tests/data/cases.yaml";
        let result = load_yaml(filename);
        assert!(result.is_ok(), "Failed to load or parse the YAML file");
        let cases = result.expect("Expected a valid Cases struct after parsing");
        let all_parameters = create_parameter_map();
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
                if found_joints_approx_equal(&solutions, &case.joints_in_radians(),
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
        let result = load_yaml(filename);
        assert!(result.is_ok(), "Failed to load or parse the YAML file");
        let cases = result.expect("Expected a valid Cases struct after parsing");
        let all_parameters = create_parameter_map();
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
                found_joints_approx_equal(&solutions, &case.joints_in_radians(),
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
        let result = load_yaml(filename);
        assert!(result.is_ok(), "Failed to load or parse the YAML file");
        let cases = result.expect("Expected a valid Cases struct after parsing");
        let all_parameters = create_parameter_map();
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
            found_joints_approx_equal(&solutions, &joints_in_radians, 0.001_f64.to_radians());
        assert!(matches!(found_matching, Some(0)),
                "Fully matching joints must come first. Expected Some(0), got {:?}", found_matching);
    }


    fn create_parameter_map() -> HashMap<String, Parameters> {
        // Create map to get actual parameters that are not in the yaml file (maybe should be?)
        let all_parameters: HashMap<String, Parameters> = vec![
            (String::from("Irb2400_10"), Parameters::irb2400_10()),
            (String::from("KukaKR6_R700_sixx"), Parameters::kuka_kr6_r700_sixx()),
            (String::from("Fanuc_r2000ib_200r"), Parameters::fanuc_r2000ib_200r()),
            (String::from("Staubli_tx40"), Parameters::staubli_tx40()),
            (String::from("Irb2600_12_165"), Parameters::irb2600_12_165()),
            (String::from("Irb4600_60_205"), Parameters::irb4600_60_205()),
            (String::from("Staubli_tx2_140"), Parameters::staubli_tx2_140()),
            (String::from("Staubli_tx2_160"), Parameters::staubli_tx2_160()),
            (String::from("Staubli_tx2_160l"), Parameters::staubli_tx2_160l()),
        ]
            .into_iter()
            .collect();
        all_parameters
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

    #[test]
    fn test_parameters_from_yaml() {
        let filename = "src/tests/data/fanuc/fanuc_m16ib20.yaml";
        let loaded =
            Parameters::from_yaml_file(filename).expect("Failed to load parameters from file");

        let expected = Parameters {
            a1: 0.15,
            a2: -0.10,
            b: 0.0,
            c1: 0.525,
            c2: 0.77,
            c3: 0.74,
            c4: 0.10,
            offsets: [0.0, 0.0, -90.0_f64.to_radians(), 0.0, 0.0, 180.0_f64.to_radians()],
            sign_corrections: [1, 1, -1, -1, -1, -1],
            dof: 6,
        };


        assert_eq!(expected.a1, loaded.a1);
        assert_eq!(expected.a2, loaded.a2);
        assert_eq!(expected.b, loaded.b);
        assert_eq!(expected.c1, loaded.c1);
        assert_eq!(expected.c2, loaded.c2);
        assert_eq!(expected.c3, loaded.c3);
        assert_eq!(expected.c4, loaded.c4);
        assert_eq!(expected.offsets, loaded.offsets);
        assert_eq!(expected.sign_corrections, loaded.sign_corrections);
    }


    #[test]
    fn test_complex_robot_reversible() {
        let filename = "src/tests/data/cases.yaml";
        let result = load_yaml(filename);
        assert!(result.is_ok(), "Failed to load or parse the YAML file: {}", result.unwrap_err());
        let cases = result.expect("Expected a valid Cases struct after parsing");
        let all_parameters = create_parameter_map();
        println!("Forward IK: {} test cases", cases.len());

        for case in cases.iter() {
            let parameters = all_parameters.get(&case.parameters).unwrap_or_else(|| {
                panic!("Parameters for the robot [{}] are unknown", &case.parameters)
            });
            let robot_alone = OPWKinematics::new(parameters.clone());

            // 1 meter high pedestal
            let pedestal = 0.5;
            let base_translation = Isometry3::from_parts(
                Translation3::new(0.0, 0.0, pedestal).into(),
                UnitQuaternion::identity(),
            );

            let robot_with_base = Base {
                robot: Arc::new(robot_alone),
                base: base_translation,
            };

            // Tool extends 1 meter in the Z direction, envisioning something like sword
            let sword = 1.0;
            let tool_translation = Isometry3::from_parts(
                Translation3::new(0.0, 0.0, sword).into(),
                UnitQuaternion::identity(),
            );

            // Create the Tool instance with the transformation
            let kinematics = Tool {
                robot: Arc::new(robot_with_base),
                tool: tool_translation,
            };


            // Try forward on the initial data set first.
            let joints = case.joints_in_radians();
            let pose = kinematics.forward(&joints);
            let solutions = kinematics.inverse_continuing(&pose, &joints);

            // It must be the matching solution, it must be the first in solutions.
            if !matches!(found_joints_approx_equal(&solutions, &joints, 
                0.001_f64.to_radians()), Some(0)) {
                println!("Not found or not first:");
                dump_joints(&joints);
                dump_solutions_degrees(&solutions);
                panic!();
            }
        }
    }
}
