use std::f64::consts::PI;
use rs_opw_kinematics::constraints::{BY_CONSTRAINS, BY_PREV, Constraints};
use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose, JOINTS_AT_ZERO};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::utils::{dump_joints, dump_solutions};

/// Usage example.
fn main() {
    let robot = OPWKinematics::new(Parameters::irb2400_10());
    let joints: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.5]; // Joints are alias of [f64; 6]
    println!("Initial joints with singularity J5 = 0: ");
    dump_joints(&joints);

    println!("Solutions (original angle set is lacking due singularity there: ");
    let pose: Pose = robot.forward(&joints); // Pose is alias of nalgebra::Isometry3<f64>

    let solutions = robot.inverse(&pose); // Solutions is alias of Vec<Joints>
    dump_solutions(&solutions);

    println!("Solutions assuming we continue from somewhere close. The 'lost solution' returns");
    let when_continuing_from: [f64; 6] = [0.0, 0.11, 0.22, 0.3, 0.1, 0.5];
    let solutions = robot.inverse_continuing(&pose, &when_continuing_from);
    dump_solutions(&solutions);

    println!("Same pose, all J4+J6 rotation assumed to be previously concentrated on J4 only");
    let when_continuing_from_j6_0: [f64; 6] = [0.0, 0.11, 0.22, 0.8, 0.1, 0.0];
    let solutions = robot.inverse_continuing(&pose, &when_continuing_from_j6_0);
    dump_solutions(&solutions);

    println!("If we do not have the previous position, we can assume we want J4, J6 close to 0.0");
    println!("The solution appears and the needed rotation is now equally distributed between J4 and J6.");
    let solutions = robot.inverse_continuing(&pose, &JOINTS_AT_ZERO);
    dump_solutions(&solutions);

    let robot = OPWKinematics::new_with_constraints(
        Parameters::irb2400_10(), Constraints::new(
            [-0.1, 0.0, 0.0, 0.0, -PI, -PI],
            [ PI, PI, 2.0*PI, PI, PI, PI],
            BY_PREV,
        ));
    println!("With constraints, sorted by proximity to the previous pose");
    let solutions = robot.inverse_continuing(&pose, &when_continuing_from_j6_0);
    dump_solutions(&solutions);

    let robot = OPWKinematics::new_with_constraints(
        Parameters::irb2400_10(), Constraints::new(
            [-0.1, 0.0, 0.0, 0.0, -PI, -PI],
            [ PI, PI, 2.0*PI, PI, PI, PI],
            BY_CONSTRAINS,
        ));
    println!("With constraints, sorted by proximity to the center of constraints");
    let solutions = robot.inverse_continuing(&pose, &when_continuing_from_j6_0);
    dump_solutions(&solutions);
    
    #[cfg(feature = "allow_filesystem")] {
        // This requires YAML library
        let parameters = Parameters::irb2400_10();
        println!("Reading:\n{}", &parameters.to_yaml());
    }
}