#[cfg(test)]
mod tests {
    use crate::constraints::{BY_PREV, Constraints};
    use crate::kinematic_traits::{J1, J2, J3, Joints, Kinematics, Pose};
    use crate::kinematics_impl::OPWKinematics;
    use crate::parallelogram::Parallelogram;
    use crate::parameters::opw_kinematics::Parameters;
    use crate::utils::dump_joints;
    use std::sync::Arc;

    #[test]
    fn continuation_preserves_turns_through_nested_couplings() {
        let travel = Constraints::new(
            [(-540.0_f64).to_radians(); 6],
            [540.0_f64.to_radians(); 6],
            BY_PREV,
        );
        let inner = Parallelogram {
            robot: Arc::new(OPWKinematics::new_with_constraints(
                Parameters::irb2400_10(),
                travel,
            )),
            driven: J1,
            coupled: J2,
            scaling: 1.0,
        };
        let outer = Parallelogram {
            robot: Arc::new(inner),
            driven: J2,
            coupled: J3,
            scaling: 1.0,
        };
        let actuator_joints = [225.0, 205.0, 50.0, 30.0, 45.0, 60.0].map(f64::to_radians);
        let previous = outer.from_constraint_joints(&actuator_joints);
        let target = outer.forward(&previous);
        for solutions in [
            outer.inverse_continuing(&target, &previous),
            outer.inverse_continuing_5dof(&target, &previous),
        ] {
            assert!(solutions.iter().any(|solution| {
                solution
                    .iter()
                    .zip(previous)
                    .all(|(joint, expected)| (joint - expected).abs() < 1e-10)
            }));
        }
    }

    #[test]
    fn test_parallelogram_orientation_changes() {
        // Robot without parallelogram
        let robot_no_parallelogram = Arc::new(OPWKinematics::new(Parameters::irb2400_10()));

        // Robot with parallelogram
        let robot_with_parallelogram = Parallelogram {
            robot: Arc::new(OPWKinematics::new(Parameters::irb2400_10())),
            driven: J2,
            coupled: J3,
            scaling: 1.0,
        };

        // Initial joint positions in degrees
        let joints_degrees: [f64; 6] = [0.0, 5.7, 11.5, 17.2, 0.0, 28.6]; // Values in degrees
        // Convert joint positions from degrees to radians
        let joints: Joints = joints_degrees.map(f64::to_radians); // Joints are alias of [f64; 6]

        println!("\nInitial joints: ");
        dump_joints(&joints);

        // Forward kinematics for both robots
        let pose_no_parallelogram: Pose = robot_no_parallelogram.forward(&joints);
        let pose_with_parallelogram: Pose = robot_with_parallelogram.forward(&joints);

        // Apply change to joint 2 (this will show the difference in behavior between the two robots)
        let mut modified_joints = joints;
        modified_joints[1] += 10_f64.to_radians();
        println!("\nModified joints (joint 2 increased by 10 degrees): ");
        dump_joints(&modified_joints);

        // Forward kinematics after modifying joints for both robots
        let modified_pose_no_parallelogram: Pose = robot_no_parallelogram.forward(&modified_joints);
        let modified_pose_with_parallelogram: Pose =
            robot_with_parallelogram.forward(&modified_joints);

        // Calculate orientation changes for both robots
        let orientation_change_no_parallelogram = pose_no_parallelogram
            .angular_distance(modified_pose_no_parallelogram)
            .to_degrees();
        let orientation_change_with_parallelogram = pose_with_parallelogram
            .angular_distance(modified_pose_with_parallelogram)
            .to_degrees();

        // Print orientation changes for both robots
        println!("\nOrientation changes after joint change:");
        println!(
            "Robot without parallelogram: angle = {:.3}",
            orientation_change_no_parallelogram
        );
        println!(
            "Robot with parallelogram: angle = {:.3}",
            orientation_change_with_parallelogram
        );

        // For robot without parallelogram, orientation change should be above the threshold (0.001 degrees)
        assert!(orientation_change_no_parallelogram > 0.001);

        // For robot with parallelogram, orientation change should be below the threshold (0.001 degrees)
        assert!(orientation_change_with_parallelogram < 0.001);
    }
}
