#[cfg(test)]
mod tests {
    use std::sync::Arc;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion};
    use crate::kinematic_traits::{Kinematics};
    use crate::kinematics_impl::OPWKinematics;
    use crate::tool::{Base, Tool};
    use crate::utils::{dump_joints, dump_solutions_degrees};
    use crate::tests::test_utils;

    #[test]
    fn test_complex_robot_reversible() {
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
            if !matches!(test_utils::found_joints_approx_equal(&solutions, &joints,
                    0.001_f64.to_radians()), Some(0)) {
                println!("Not found or not first:");
                dump_joints(&joints);
                dump_solutions_degrees(&solutions);
                panic!();
            }

            // Try also 5DOF with fixed J6:
            let solutions_5dof = kinematics.inverse_5dof(&pose, 77.0_f64.to_radians());
            for solution in &solutions_5dof {
                let xyz = kinematics.forward(solution).translation;
                if (pose.translation.vector - xyz.vector).norm() > 1E-6 {
                    println!("The XYZ location {}, {}, {} of TCP is not at pose {}, {}, {}",
                             xyz.x, xyz.y, xyz.z,
                             pose.translation.x, pose.translation.y, pose.translation.z);
                    panic!();
                }
            }
        }
    }
}