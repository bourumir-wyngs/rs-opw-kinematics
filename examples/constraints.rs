use rs_opw_kinematics::constraints::{Constraints, BY_CONSTRAINS, BY_PREV};
use rs_opw_kinematics::kinematic_traits::{AngleConversion, Joints, Kinematics, Pose, CONSTRAINT_CENTERED};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::utils::dump_solutions;

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
            offsets: Joints::from_degrees([0.0, 0.0, -180.0, 0.0, 0.0, 0.0]),
            sign_corrections: [1; 6],
        },
        Constraints::from_degrees(
            [
                -5.0..=5.0,
                0.0..=180.0,
                0.0..=360.0,
                -180.0..=180.0,
                -90.0..=90.0,
                -180.0..=180.0,
            ],
            BY_PREV,
        ),
    );

    // Initialize joint angles in degrees and convert them to radians.
    let joints: Joints = Joints::from_degrees(
        [0.0, 5.7, 11.5, 17.2, 0.0, 28.6]);
    let when_continuing_from_j6_0: Joints = Joints::from_degrees(
        [0.0, 6.3, 12.6, 45.9, 5.7, 0.0]);

    // Perform forward kinematics to get the pose.
    let pose: Pose = robot.forward(&joints);

    println!(
        "If we do not have the previous pose yet, we can now ask to prefer the pose \
    closer to the center of constraints."
    );
    let solutions = robot.inverse_continuing(&pose, &CONSTRAINT_CENTERED);
    dump_solutions(&solutions);

    println!("With constraints, sorted by proximity to the previous pose");
    let solutions = robot.inverse_continuing(&pose, &when_continuing_from_j6_0);
    dump_solutions(&solutions);

    // Create a new robot instance using preset IRB 2400 parameters.
    let robot = OPWKinematics::new_with_constraints(
        Parameters::irb2400_10(),
        Constraints::from_degrees(
            [
                -180.0..=180.0,
                0.0..=180.0,
                0.0..=360.0,
                -180.0..=180.0,
                -90.0..=90.0,
                -180.0..=180.0,
            ],
            BY_CONSTRAINS,
        ),
    );

    println!("With constraints, sorted by proximity to the center of constraints");
    let solutions = robot.inverse_continuing(&pose, &when_continuing_from_j6_0);
    dump_solutions(&solutions);
}
