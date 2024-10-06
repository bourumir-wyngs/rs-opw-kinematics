use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose, JOINTS_AT_ZERO};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::utils::{dump_joints, dump_solutions};

fn main() {
    let robot = OPWKinematics::new(Parameters::irb2400_10());
    let joints: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.5]; // Joints are alias of [f64; 6]
    println!("\nInitial joints with singularity J5 = 0: ");
    dump_joints(&joints);

    println!("\nSolutions (original angle set is lacking due singularity there: ");
    let pose: Pose = robot.forward(&joints); // Pose is alias of nalgebra::Isometry3<f64>

    let solutions = robot.inverse(&pose); // Solutions is alias of Vec<Joints>
    dump_solutions(&solutions);

    println!("\nSolutions assuming we continue from somewhere close. The 'lost solution' returns");
    let when_continuing_from: [f64; 6] = [0.0, 0.11, 0.22, 0.3, 0.1, 0.5];
    let solutions = robot.inverse_continuing(&pose, &when_continuing_from);
    dump_solutions(&solutions);

    println!("\nSame pose, all J4+J6 rotation assumed to be previously concentrated on J4 only");
    let when_continuing_from_j6_0: [f64; 6] = [0.0, 0.11, 0.22, 0.8, 0.1, 0.0];
    let solutions = robot.inverse_continuing(&pose, &when_continuing_from_j6_0);
    dump_solutions(&solutions);

    println!("\nIf we do not have the previous position, we can assume we want J4, J6 close to 0.0 \
    The solution appears and the needed rotation is now equally distributed between J4 and J6.");
    let solutions = robot.inverse_continuing(&pose, &JOINTS_AT_ZERO);
    dump_solutions(&solutions);

    println!("\n5 DOF, J6 at fixed angle 77 degrees");
    let solutions5dof = robot.inverse_5dof(&pose, 77.0_f64.to_radians());
    dump_solutions(&solutions5dof);
    println!("The XYZ location of TCP is still as in the original pose x = {:.3}, y = {:.3}, z = {:.3}:",
             pose.translation.x, pose.translation.y, pose.translation.z);
    for solution in &solutions {
        let translation = robot.forward(solution).translation;
        println!("Translation: x = {:.3}, y = {:.3}, z = {:.3}", translation.x, translation.y, translation.z);
    }
}