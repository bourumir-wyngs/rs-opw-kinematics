use std::f64::consts::PI;
use std::sync::Arc;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::constraints::{BY_CONSTRAINS, Constraints};

/// Calculating Jacobian matrices for kinematic analysis.
fn main() {
    let robot = OPWKinematics::new_with_constraints(
        Parameters::irb2400_10(), Constraints::new(
            [-0.1, 0.0, 0.0, 0.0, -PI, -PI],
            [PI, PI, 2.0 * PI, PI, PI, PI],
            BY_CONSTRAINS,
        ));

    let joints: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.5]; // Joints are alias of [f64; 6]
    let jakobian = rs_opw_kinematics::jacobian::Jacobian::new(&robot, &joints, 1E-6);
    let desired_velocity_isometry =
        Isometry3::new(Vector3::new(0.0, 1.0, 0.0),
                       Vector3::new(0.0, 0.0, 1.0));
    let joint_velocities = jakobian.velocities(&desired_velocity_isometry);
    println!("Computed joint velocities: {:?}", joint_velocities.unwrap());

    let desired_force_torque =
        Isometry3::new(Vector3::new(0.0, 0.0, 0.0),
                       Vector3::new(0.0, 0.0, 1.234));

    let joint_torques = jakobian.torques(&desired_force_torque);
    println!("Computed joint torques: {:?}", joint_torques);

    // Robot with the tool, standing on a base:
    let robot_alone = OPWKinematics::new(Parameters::staubli_tx2_160l());

    // 1 meter high pedestal
    let pedestal = 0.5;
    let base_translation = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, pedestal).into(),
        UnitQuaternion::identity(),
    );

    let robot_with_base = rs_opw_kinematics::tool::Base {
        robot: Arc::new(robot_alone),
        base: base_translation,
    };

    // Tool extends 1 meter in the Z direction, envisioning something like sword
    let sword = 1.0;
    let tool_translation = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, sword).into(),
        UnitQuaternion::identity(),
    );

    // Create the Tool instance with the transformation
    let robot_complete = rs_opw_kinematics::tool::Tool {
        robot: Arc::new(robot_with_base),
        tool: tool_translation,
    };

    let tcp_pose: Pose = robot_complete.forward(&joints);
    println!("The sword tip is at: {:?}", tcp_pose);
}