use rs_opw_kinematics::constraints::{Constraints, BY_CONSTRAINS, BY_PREV};
use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose, CONSTRAINT_CENTERED};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::utils::dump_solutions;
use std::f64::consts::PI;

fn main() {
    // Create robot specifying parameters. This is ABB IRB 4600 industrial arm:
    let robot = OPWKinematics::new_with_constraints(
        Parameters {
            a1: 0.175,
            a2: -0.175,
            b: 0.000,
            c1: 0.495,
            c2: 0.900,
            c3: 0.960,
            c4: 0.135,
            offsets: [0.0, 0.0, -180.0_f64.to_radians(), 0.0, 0.0, 0.0],
            sign_corrections: [1; 6],
        },
        Constraints::new(
            [-0.1, 0.0, 0.0, 0.0, -PI, -PI],
            [PI, PI, 2.0 * PI, PI, PI, PI],
            BY_PREV,
        ),
    );

    let joints: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.5]; // Joints are alias of [f64; 6]
    let when_continuing_from_j6_0: [f64; 6] = [0.0, 0.11, 0.22, 0.8, 0.1, 0.0];

    let pose: Pose = robot.forward(&joints); // Pose is alias of nalgebra::Isometry3<f64>

    println!(
        "If we do not have the previous pose yet, we can now ask to prever the pose \
    closer to the center of constraints."
    );
    let solutions = robot.inverse_continuing(&pose, &CONSTRAINT_CENTERED);
    dump_solutions(&solutions);

    println!("With constraints, sorted by proximity to the previous pose");
    let solutions = robot.inverse_continuing(&pose, &when_continuing_from_j6_0);
    dump_solutions(&solutions);

    let robot = OPWKinematics::new_with_constraints(
        Parameters::irb2400_10(),
        Constraints::new(
            [-0.1, 0.0, 0.0, 0.0, -PI, -PI],
            [PI, PI, 2.0 * PI, PI, PI, PI],
            BY_CONSTRAINS,
        ),
    );
    println!("With constraints, sorted by proximity to the center of constraints");
    let solutions = robot.inverse_continuing(&pose, &when_continuing_from_j6_0);
    dump_solutions(&solutions);
}
