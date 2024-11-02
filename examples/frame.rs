use rs_opw_kinematics::utils::{dump_joints, dump_solutions};
use std::sync::Arc;
use nalgebra::Point3;
use rs_opw_kinematics::frame::Frame;
use rs_opw_kinematics::kinematic_traits::Kinematics;
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;

/// Using frames, a foundational concept in robotic programming for managing coordinates.
fn main() {
    let robot = OPWKinematics::new(Parameters::irb2400_10());
    // Shift not too much to have values close to previous
    let frame_transform = Frame::translation(
        Point3::new(0.0, 0.0, 0.0), 
        Point3::new(0.011, 0.022, 0.033));

    let framed = Frame {
        robot: Arc::new(robot),
        frame: frame_transform,
    };
    let joints_no_frame: [f64; 6] = [0.0, 0.11, 0.22, 0.3, 0.1, 0.5]; // without frame

    println!("No frame transform:");
    dump_joints(&joints_no_frame);

    println!("Possible joint values after the frame transform:");
    let (solutions, _transformed_pose) = framed.forward_transformed(
        &joints_no_frame, &joints_no_frame);
    dump_solutions(&solutions);

    let framed = robot.forward(&solutions[0]).translation;
    let unframed = robot.forward(&joints_no_frame).translation;

    println!("Distance between framed and not framed pose {:.3} {:.3} {:.3}",
             framed.x - unframed.x, framed.y - unframed.y, framed.z - unframed.z);
}