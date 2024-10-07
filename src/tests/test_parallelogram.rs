#[cfg(test)]
mod tests {
    use std::sync::Arc;
    use crate::kinematic_traits::{Joints, Kinematics, Pose, J2, J3};
    use crate::kinematics_impl::OPWKinematics;
    use crate::parameters::opw_kinematics::Parameters;
    use crate::utils::dump_joints;
    use crate::parallelogram::Parallelogram;

    fn euler_angles_in_degrees(pose: &Pose) -> (f64, f64, f64) {
        let euler_angles = pose.rotation.euler_angles();
        (
            euler_angles.0.to_degrees(), // roll
            euler_angles.1.to_degrees(), // pitch
            euler_angles.2.to_degrees()  // yaw
        )
    }

    fn orientation_change(
        initial: (f64, f64, f64),
        modified: (f64, f64, f64)
    ) -> (f64, f64, f64) {
        let roll_change = modified.0 - initial.0;
        let pitch_change = modified.1 - initial.1;
        let yaw_change = modified.2 - initial.2;

        (roll_change, pitch_change, yaw_change)
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
            scaling: 1.0
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

        // Get initial orientation in degrees for both robots
        let initial_orientation_no_parallelogram = euler_angles_in_degrees(&pose_no_parallelogram);
        let initial_orientation_with_parallelogram = euler_angles_in_degrees(&pose_with_parallelogram);

        // Apply change to joint 2 (this will show the difference in behavior between the two robots)
        let mut modified_joints = joints;
        modified_joints[1] += 10_f64.to_radians();
        println!("\nModified joints (joint 2 increased by 10 degrees): ");
        dump_joints(&modified_joints);

        // Forward kinematics after modifying joints for both robots
        let modified_pose_no_parallelogram: Pose = robot_no_parallelogram.forward(&modified_joints);
        let modified_pose_with_parallelogram: Pose = robot_with_parallelogram.forward(&modified_joints);

        // Get modified orientation in degrees for both robots
        let modified_orientation_no_parallelogram = euler_angles_in_degrees(&modified_pose_no_parallelogram);
        let modified_orientation_with_parallelogram = euler_angles_in_degrees(&modified_pose_with_parallelogram);

        // Calculate orientation changes for both robots
        let orientation_change_no_parallelogram = orientation_change(
            initial_orientation_no_parallelogram,
            modified_orientation_no_parallelogram
        );
        let orientation_change_with_parallelogram = orientation_change(
            initial_orientation_with_parallelogram,
            modified_orientation_with_parallelogram
        );

        // Print orientation changes for both robots
        println!("\nOrientation changes after joint change:");
        println!(
            "Robot without parallelogram: roll = {:.3}, pitch = {:.3}, yaw = {:.3}",
            orientation_change_no_parallelogram.0,
            orientation_change_no_parallelogram.1,
            orientation_change_no_parallelogram.2
        );
        println!(
            "Robot with parallelogram: roll = {:.3}, pitch = {:.3}, yaw = {:.3}",
            orientation_change_with_parallelogram.0,
            orientation_change_with_parallelogram.1,
            orientation_change_with_parallelogram.2
        );

        // Assertions: 
        // For robot without parallelogram, orientation change should be above the threshold (0.001)
        assert!(orientation_change_no_parallelogram.0.abs() > 0.001);
        assert!(orientation_change_no_parallelogram.1.abs() > 0.001);
        assert!(orientation_change_no_parallelogram.2.abs() > 0.001);

        // For robot with parallelogram, orientation change should be below the threshold (0.001)
        assert!(orientation_change_with_parallelogram.0.abs() < 0.001);
        assert!(orientation_change_with_parallelogram.1.abs() < 0.001);
        assert!(orientation_change_with_parallelogram.2.abs() < 0.001);
    }
}

