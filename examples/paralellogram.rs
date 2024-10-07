use std::sync::Arc;
use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose, J2, J3};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::utils::dump_joints;
use rs_opw_kinematics::parallelogram::Parallelogram;

fn euler_angles_in_degrees(pose: &Pose) -> (f64, f64, f64) {
    let euler_angles = pose.rotation.euler_angles();
    (
        euler_angles.0.to_degrees(), // roll
        euler_angles.1.to_degrees(), // pitch
        euler_angles.2.to_degrees()  // yaw
    )
}

fn print_orientation_change(
    initial: (f64, f64, f64),
    modified: (f64, f64, f64),
    label: &str
) {
    let roll_change = modified.0 - initial.0;
    let pitch_change = modified.1 - initial.1;
    let yaw_change = modified.2 - initial.2;

    println!(
        "{} orientation change: roll = {:.3}, pitch = {:.3}, yaw = {:.3}",
        label, roll_change, pitch_change, yaw_change
    );
}

fn calculate_travel_distance(pose_initial: &Pose, pose_modified: &Pose) -> f64 {
    (pose_initial.translation.vector - pose_modified.translation.vector).norm()
}

fn main() {
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

    // Print orientation changes for both robots
    println!("\nOrientation changes after joint change:");
    print_orientation_change(
        initial_orientation_no_parallelogram,
        modified_orientation_no_parallelogram,
        "Robot without parallelogram"
    );
    print_orientation_change(
        initial_orientation_with_parallelogram,
        modified_orientation_with_parallelogram,
        "Robot with parallelogram"
    );

    // Calculate and print travel distances
    let travel_distance_no_parallelogram = calculate_travel_distance(&pose_no_parallelogram, &modified_pose_no_parallelogram);
    let travel_distance_with_parallelogram = calculate_travel_distance(&pose_with_parallelogram, &modified_pose_with_parallelogram);

    println!("\nTravel distances after joint change:");
    println!(
        "Robot without parallelogram: travel distance = {:.6}",
        travel_distance_no_parallelogram
    );
    println!(
        "Robot with parallelogram: travel distance = {:.6}",
        travel_distance_with_parallelogram
    );

    // The result should show that the robot without a parallelogram experiences a larger change in orientation.
    // The robot with parallelogram has much less orientation change, but still travels a reasonable distance.
}
