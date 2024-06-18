use rs_opw_kinematics::utils::{dump_joints, dump_solutions};
use std::sync::Arc;
use nalgebra::Point3;
use rs_opw_kinematics::frame::Frame;
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;

fn main() {
    let robot = OPWKinematics::new(Parameters::irb2400_10());
    // Shift only x by 10 mm to have values close to previous
    let frame_transform = Frame::translation(
        Point3::new(0.0, 0.0, 0.0), Point3::new(0.01, 0.00, 0.0));
    
    let framed = rs_opw_kinematics::frame::Frame {
        robot: Arc::new(robot),
        frame: frame_transform,
    };
    let joints_no_frame: [f64; 6] = [0.0, 0.11, 0.22, 0.3, 0.1, 0.5]; // without frame

    println!("No frame transform:");
    dump_joints(&joints_no_frame);

    println!("Possible joint values after the frame tranform:");
    let (solutions, _transformed_pose) = framed.forward_transformed(
        &joints_no_frame, &joints_no_frame);
    dump_solutions(&solutions);
}