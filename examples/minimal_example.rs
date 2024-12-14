use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose, JOINTS_AT_ZERO};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::utils::{dump_joints, dump_solutions};

fn main() {
    // Create robot specifying parameters. This is ABB IRB 4600 industrial arm:
    let robot = OPWKinematics::new(Parameters {
        a1: 0.175,
        a2: -0.175,
        b: 0.000,
        c1: 0.495,
        c2: 0.900,
        c3: 0.960,
        c4: 0.135,
        offsets: [0.0, 0.0, -180.0_f64.to_radians(), 0.0, 0.0, 0.0],
        sign_corrections: [1; 6]
    });

    // Joints are alias of [f64; 6], values in radians    
    let joints: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.5]; 
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

    println!(
        "If we do not have the previous position, we can assume we want J4, J6 close to 0.0 \
    The solution appears and the needed rotation is now equally distributed between J4 and J6."
    );
    let solutions = robot.inverse_continuing(&pose, &JOINTS_AT_ZERO);
    dump_solutions(&solutions);
}
