use super::{ANGULAR_TOLERANCE, OPWKinematics, compare_poses};
use crate::kinematic_traits::{J4, J5, J6, Joints, Kinematics, Singularity};
use crate::parameters::opw_kinematics::Parameters;
use std::f64::consts::PI;

fn joints_from_degrees(joints: [f64; 6]) -> Joints {
    joints.map(f64::to_radians)
}

fn circular_angle_distance(left: f64, right: f64) -> f64 {
    let difference = (left - right).rem_euclid(2.0 * PI);
    difference.min(2.0 * PI - difference)
}

fn same_joint_configuration(left: &Joints, right: &Joints, tolerance_degrees: f64) -> bool {
    let tolerance = tolerance_degrees.to_radians();
    (0..6).all(|joint| circular_angle_distance(left[joint], right[joint]) <= tolerance)
}

fn assert_half_turn_apart(left: f64, right: f64, joint_name: &str) {
    let separation_degrees = circular_angle_distance(left, right).to_degrees();
    assert!(
        (separation_degrees - 180.0).abs() <= 1e-8,
        "{joint_name} separation is {separation_degrees} degrees; expected 180 degrees",
    );
}

#[test]
fn analytical_wrist_branches_stay_half_turn_apart_in_singularity_zone() {
    let robot = OPWKinematics::new(Parameters::irb2400_10());

    // The threshold is 0.01 degrees. These cases are inside the library's
    // singularity zone but are not exact mathematical singularities.
    for j5_degrees in [0.005_f64, 179.995] {
        let joints = joints_from_degrees([10.0, 20.0, 30.0, 40.0, j5_degrees, 60.0]);

        assert_eq!(
            robot.kinematic_singularity(&joints),
            Some(Singularity::A),
            "J5={j5_degrees} degrees should be inside the singularity zone",
        );

        let pose = robot.forward(&joints);
        let analytical = robot.inverse(&pose);

        assert_eq!(
            analytical.len(),
            8,
            "expected all analytical solutions for J5={j5_degrees} degrees",
        );

        // inverse_intern constructs solutions 4..8 as the opposite wrist
        // representations of solutions 0..4.
        for arm_branch in 0..4 {
            let first_wrist = &analytical[arm_branch];
            let second_wrist = &analytical[arm_branch + 4];

            for joint in 0..3 {
                assert!(
                    circular_angle_distance(first_wrist[joint], second_wrist[joint]) <= 1e-10,
                    "arm joint J{} differs between wrist branches at J5={} degrees",
                    joint + 1,
                    j5_degrees,
                );
            }

            assert_half_turn_apart(first_wrist[J4], second_wrist[J4], "J4");
            assert_half_turn_apart(first_wrist[J6], second_wrist[J6], "J6");

            assert!(
                circular_angle_distance(first_wrist[J5], -second_wrist[J5]) <= 1e-10,
                "J5 values are not opposite wrist representations at \
                 J5={j5_degrees} degrees",
            );
        }

        // Continuing IK may normalize, sort, and add one synthetic solution,
        // but every original analytical branch must remain present.
        let continuing = robot.inverse_continuing(&pose, &joints);

        for analytical_solution in &analytical {
            assert!(
                continuing.iter().any(|continued_solution| {
                    same_joint_configuration(continued_solution, analytical_solution, 1e-8)
                }),
                "inverse_continuing dropped an analytical wrist branch at \
                 J5={j5_degrees} degrees",
            );
        }
    }
}

#[test]
fn exact_wrist_singularities_have_a_continuum_of_joint_solutions() {
    let robot = OPWKinematics::new(Parameters::irb2400_10());

    // At J5 = 0 degrees, only J4 + J6 is observable. These configurations
    // differ by 30 degrees, not 180 degrees, but produce the same pose.
    let zero_first = joints_from_degrees([10.0, 20.0, 30.0, 40.0, 0.0, 50.0]);
    let zero_second = joints_from_degrees([10.0, 20.0, 30.0, 10.0, 0.0, 80.0]);

    assert!(compare_poses(
        &robot.forward(&zero_first),
        &robot.forward(&zero_second),
        robot.distance_tolerance,
        ANGULAR_TOLERANCE,
    ));

    // At J5 = 180 degrees, only J4 - J6 is observable. Both examples
    // have a difference of -10 degrees.
    let pi_first = joints_from_degrees([10.0, 20.0, 30.0, 40.0, 180.0, 50.0]);
    let pi_second = joints_from_degrees([10.0, 20.0, 30.0, 10.0, 180.0, 20.0]);

    assert!(compare_poses(
        &robot.forward(&pi_first),
        &robot.forward(&pi_second),
        robot.distance_tolerance,
        ANGULAR_TOLERANCE,
    ));
}
