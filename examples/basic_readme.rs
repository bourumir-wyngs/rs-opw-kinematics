use nalgebra::Isometry3;
use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;

fn main() {
    // Create a robot with a built-in parameter set.
    // Plain kinematics; no collision checks.
    let robot = OPWKinematics::new(Parameters::irb2400_10());

    // Joints are an alias of [f64; 6], given in radians here.
    let joints: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.5];
    let pose: Isometry3<f64> = robot.forward(&joints);
    let when_continuing_from: Joints = [0.0, 0.11, 0.22, 0.3, 0.1, 0.5];

    let solutions = robot.inverse_continuing(&pose, &when_continuing_from);
}