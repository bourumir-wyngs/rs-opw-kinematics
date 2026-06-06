use rs_opw_kinematics::frame::Frame;
use rs_opw_kinematics::glam::{DQuat, DVec3};
use rs_opw_kinematics::kinematic_traits::Kinematics;
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::utils::{dump_joints, dump_solutions};
use std::error::Error;
use std::sync::Arc;

/// Retarget a canonical trajectory by computing a transport frame from three tie point pairs.
fn main() -> Result<(), Box<dyn Error>> {
    let robot: Arc<dyn Kinematics> = Arc::new(OPWKinematics::new(Parameters::irb2400_10()));

    // This is the original, canonical program. In a real cell these joint positions
    // might be taught once against a fixture, pallet, or workpiece at a nominal location.
    let canonical_program: [[f64; 6]; 3] = [
        [0.0, 0.11, 0.22, 0.30, 0.10, 0.50],
        [0.04, 0.14, 0.19, 0.34, 0.12, 0.45],
        [-0.03, 0.10, 0.25, 0.27, 0.08, 0.56],
    ];

    // Tie points are corresponding points between the original program and the
    // required target setup. Three non-collinear tie point pairs define the
    // transport frame: source/original trajectory points -> target/measured
    // points. The frame may include uniform scale in addition to rotation and
    // translation.
    let original_tie_points = canonical_program.map(|joints| robot.forward(&joints).translation);

    // Simulate the workpiece being moved, slightly rotated, and uniformly scaled
    // around the first tie point. In production these target tie points would be
    // measured or provided by calibration.
    let target_rotation = DQuat::from_rotation_z(5.0_f64.to_radians());
    let target_shift = DVec3::new(0.011, 0.022, 0.033);
    let target_scale = 1.02;
    let anchor = original_tie_points[0];
    let target_tie_points = original_tie_points.map(|point| {
        transform_about_anchor(point, anchor, target_rotation, target_shift, target_scale)
    });

    println!("Tie point pairs used to compute the transport frame:");
    for (index, (original, target)) in original_tie_points
        .iter()
        .zip(target_tie_points.iter())
        .enumerate()
    {
        println!(
            "  {}: original {} -> target {}",
            index + 1,
            format_point(*original),
            format_point(*target)
        );
    }

    let frame_transform = Frame::from_tie(original_tie_points, target_tie_points)?;
    println!("Computed frame scale: {:.6}", frame_transform.scale);

    let framed = Frame {
        robot: Arc::clone(&robot),
        frame: frame_transform,
    };

    println!("\nCanonical trajectory retargeted through the tie-point frame:");
    let mut previous = canonical_program[0];
    for (index, canonical_joints) in canonical_program.iter().enumerate() {
        println!("\nWaypoint {}", index + 1);
        println!("Canonical joints:");
        dump_joints(canonical_joints);

        let (solutions, target_pose) = framed.forward_transformed(canonical_joints, &previous);
        println!("Retargeted joint solutions:");
        dump_solutions(&solutions);

        if let Some(selected) = solutions.first() {
            let achieved_pose = robot.forward(selected);
            let translation_error = (achieved_pose.translation - target_pose.translation).length();
            println!(
                "Target TCP {}, achieved {}, translation error {:.6}",
                format_point(target_pose.translation),
                format_point(achieved_pose.translation),
                translation_error
            );
            previous = *selected;
        } else {
            println!("No IK solution for this retargeted waypoint");
        }
    }

    Ok(())
}

fn transform_about_anchor(
    point: DVec3,
    anchor: DVec3,
    rotation: DQuat,
    translation: DVec3,
    scale: f64,
) -> DVec3 {
    anchor + translation + rotation * ((point - anchor) * scale)
}

fn format_point(point: DVec3) -> String {
    format!("[{:.4}, {:.4}, {:.4}]", point.x, point.y, point.z)
}
