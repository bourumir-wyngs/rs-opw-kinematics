use std::f32::consts::PI;
use nalgebra::{Isometry3};
use rs_opw_kinematics::engraving::{build_engraving_path_cylindric, build_engraving_path_side_projected};
use rs_opw_kinematics::projector::{Axis, RayDirection};
use std::fs::File;
use std::io::Write;
use rs_cxx_ros2_opw_bridge::sender::Sender;
use rs_read_trimesh::load_trimesh;

/// Generate the waypoint that make the letter R
#[allow(non_snake_case)] // we generate uppercase R
fn generate_R_waypoints(height: f32, min_dist: f32) -> Vec<(f32, f32)> {
    let mut waypoints = Vec::new();

    let width = height / 2.0; // Define the width as half the height
    let half_circle_radius = width / 2.0;

    // Helper to generate straight line points
    let generate_line = |start: (f32, f32), end: (f32, f32)| {
        let mut points = Vec::new();
        let dx = end.0 - start.0;
        let dy = end.1 - start.1;
        let dist = (dx * dx + dy * dy).sqrt();
        let steps = (dist / min_dist).ceil() as usize;
        for step in 0..=steps {
            let t = step as f32 / steps as f32;
            points.push((start.0 + t * dx, start.1 + t * dy));
        }
        points
    };

    // Generate points for the curved part (half-circle)
    let generate_half_circle = |center: (f32, f32), radius: f32, start_angle: f32, end_angle: f32| {
        let mut points = Vec::new();
        let arc_length = radius * (end_angle - start_angle).abs(); // Arc length of the curve
        let num_points = (arc_length / min_dist).ceil() as usize; // Number of points based on min_dist
        let step = (end_angle - start_angle) / num_points as f32;

        for i in 0..=num_points {
            let angle = start_angle + i as f32 * step;
            let x = center.0 + radius * angle.cos();
            let y = center.1 - radius * angle.sin();
            points.push((x, y));
        }

        points
    };

    // Step 1: Generate points for the vertical line from bottom-left to top-left
    waypoints.extend(generate_line((0.0, 0.0), (0.0, height)));

    // Step 2: Generate the points for the half-circle
    let half_circle_center = (0.0, height * 0.75);
    waypoints.extend(generate_half_circle(
        half_circle_center,
        half_circle_radius,
        -PI / 2.0, // Start at the bottom of the half-circle
        PI / 2.0   // End at the top of the half-circle
    ));

    // Step 3: Generate points for the line from circle's top to vertical midpoint
    waypoints.extend(generate_line((0.0, height * 0.75 - half_circle_radius), (0.0, height * 0.5)));

    // Step 4: Generate points for the diagonal stroke
    waypoints.extend(generate_line((0.0, height * 0.5), (width * 0.5, 0.0)));

    waypoints
}

fn write_isometries_to_json(
    file_path: &str,
    isometries: Vec<Isometry3<f32>>,
) -> Result<(), String> {
    let mut json_output = String::from("[");

    for (i, iso) in isometries.iter().enumerate() {
        let translation = iso.translation;
        let rotation = iso.rotation;

        json_output.push_str(&format!(
            "{{\"position\":{{\"x\":{},\"y\":{},\"z\":{}}},\"rotation\":{{\"x\":{},\"y\":{},\"z\":{},\"w\":{}}}}}\n",
            translation.x, translation.y, translation.z,
            rotation.i, rotation.j, rotation.k, rotation.w
        ));

        if i < isometries.len() - 1 {
            json_output.push_str(",");
        }
    }

    json_output.push_str("]");

    let mut file = File::create(file_path).expect("Failed to create file");
    file.write_all(json_output.as_bytes())
        .expect("Failed to write to file");
    Ok(())
}

fn main() -> Result<(), String> {
    // Load the mesh from a PLY file
    let mesh = load_trimesh("src/tests/data/goblet/goblet.stl", 1.0)?;
    let path = generate_R_waypoints(1.0, 0.01);

    let engraving = build_engraving_path_cylindric(&mesh, &path, 
      0.5, 0.4.. 0.58, 0. ..1.0*PI, RayDirection::FromPositive)?;
    
    //let engraving = build_engraving_path_side_projected(&mesh, &path, Axis::X, RayDirection::FromPositive)?; // works

    // pose rotation observed
    //let engraving = build_engraving_path(&mesh, &path, Axis::X, RayDirection::FromNegative)?; // works
    
    // Works
    //let engraving = build_engraving_path(&mesh, &path, Axis::Y, RayDirection::FromPositive)?;

    // Works
    //let engraving = build_engraving_path(&mesh, &path, Axis::Y, RayDirection::FromNegative)?;

    // Works, shape is not extracted
    //let engraving = build_engraving_path(&mesh, &path, Axis::Z, RayDirection::FromNegative)?;

    // Works, shape is not extracted
    //let engraving = build_engraving_path(&mesh, &path, Axis::Z, RayDirection::FromPositive)?;
    
    println!("Engraving length of {} for the path of length {}", engraving.len(), path.len());

    let sender = Sender::new("127.0.0.1", 5555);

    // Handle the result of `send_pose_message`
    match sender.send_pose_message32(&engraving) {
        Ok(_) => {
            println!("Pose message sent successfully.");
        }
        Err(err) => {
            eprintln!("Failed to send pose message: {}", err);
        }
    }    
    
    //return Ok(());

    if let Err(e) = write_isometries_to_json("work/isometries.json", engraving) {
        return Err(e)
    };
    
    Ok(())
}
