use rs_opw_kinematics::kinematic_traits::{AngleConversion, Joints, Kinematics, Pose, JOINTS_AT_ZERO};
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
        offsets: Joints::from_degrees([0.0, 0.0, -180.0, 0.0, 0.0, 0.0]),
        sign_corrections: [1; 6],
    });

    // Initialize joints in degrees and convert to radians using Joints::from_degrees
    let joints: Joints = Joints::from_degrees([0.0, 5.7, 11.5, 17.2, /*>*/ 0.0 /*<*/, 28.6]);
    println!("Initial joints with singularity J5 = 0: ");
    dump_joints(&joints);

    println!("Solutions (original angle set is lacking due singularity there): ");
    let pose: Pose = robot.forward(&joints); // Pose is alias of nalgebra::Isometry3<f64>

    let solutions = robot.inverse(&pose); // Solutions is alias of Vec<Joints>
    dump_solutions(&solutions);

    println!("Solutions assuming we continue from somewhere close. The 'lost solution' returns");
    let when_continuing_from: Joints = Joints::from_degrees([0.0, 6.3, 12.6, 17.2, 5.7, 28.6]);
    let solutions = robot.inverse_continuing(&pose, &when_continuing_from);
    dump_solutions(&solutions);
}