use std::sync::Arc;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::utils::{dump_joints, dump_solutions};

fn main() {
    let joints: Joints = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]; // Joints are alias of [f64; 6]
    dump_joints(&joints);

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
    let robot_on_base_with_tool = rs_opw_kinematics::tool::Tool {
        robot: Arc::new(robot_with_base),
        tool: tool_translation,
    };

    let tcp_pose: Pose = robot_on_base_with_tool.forward(&joints);
    println!("The sword tip is at: {:?}", tcp_pose);

    // robot_complete implements Kinematics so have the usual inverse kinematics methods available.    
    let inverse = robot_on_base_with_tool.inverse_continuing(&tcp_pose, &joints);
    dump_solutions(&inverse);

}