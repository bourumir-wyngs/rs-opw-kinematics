use std::f32::consts::PI;
use nalgebra::{Isometry3, Vector3};
use rs_opw_kinematics::engraving::build_engraving_path;
use rs_opw_kinematics::projector::{Axis, RayDirection};
use rs_opw_kinematics::read_trimesh::load_trimesh_from_ply;
use std::fs::File;
use std::io::Write;
/// Generate the waypoint that make the letter R
fn generate_R_waypoints(height: f32) -> Vec<(f32, f32)> {
    let mut waypoints = Vec::new();

    let circle_center = (0.0_f32, height / 2.0);
    let r = 0.5 * height * 0.8;
    let n = 256;
    /*
    for i in 0..=n {
        let angle = 2.0*std::f32::consts::PI * (i as f32) / (n as f32); // Half-circle from 0 to PI
        let x = circle_center.0 + r * angle.cos();
        let y = circle_center.1 - r * angle.sin();
        waypoints.push((x, y));
    }
    return waypoints;
    
    waypoints.push((0.0, 0.0));
    waypoints.push((0.5, 0.0));
    waypoints.push((1.0, 0.0));
    waypoints.push((1.0, 0.5));
    waypoints.push((1.0, 1.0));
    waypoints.push((0.5, 1.0));
    waypoints.push((0.0, 1.0));
    waypoints.push((0.5, 0.5));
    
    return waypoints;
    */
     
    
    fn compute_segments_from_height(height: f32) -> usize {
        let k = 0.1; // Adjust resolution factor as needed
        (k * height).ceil().max(4.0) as usize
    }

    let n = 50; //compute_segments_from_height(height); // Dynamically compute n
    let width = height / 2.0; // Define the width as half the height
    let half_circle_radius = width / 2.0;

    // Step 1: Add the bottom-left and top-left points of the vertical line
    waypoints.push((0.0, 0.0)); // Bottom-left
    waypoints.push((0.0, height)); // Top-left

    // Step 2: Add the half-circle waypoints
    let half_circle_center = (0.0, height * 0.75);
    for i in 0..=n {
        let angle = PI * (i as f32) / (n as f32) - PI/2.0; // Half-circle from 0 to PI
        let x = half_circle_center.0 + half_circle_radius * angle.cos();
        let y = half_circle_center.1 - half_circle_radius * angle.sin();
        waypoints.push((x, y));
    }

    // Step 3: Add the point where the half-circle ends on the vertical line
    waypoints.push((0.0, height * 0.5)); // Middle of vertical line

    // Step 4: Add the diagonal stroke's end point
    waypoints.push((width *0.5, 0.0)); // Bottom-right corner of diagonal

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
    let mesh = load_trimesh_from_ply("src/tests/data/goblet/ToriLeighR.ply");
    let path = generate_R_waypoints(1.0);
    
    let engraving = build_engraving_path(&mesh, &path, Axis::X, RayDirection::FromPositive)?; // works
    
    // let engraving = build_engraving_path(&mesh, &path, Axis::X, RayDirection::FromNegative)?; // works
    
    // Works, shape is not extracted
    // let engraving = build_engraving_path(&mesh, &path, Axis::Y, RayDirection::FromPositive)?;

    // Works, shape is not extracted
    //let engraving = build_engraving_path(&mesh, &path, Axis::Y, RayDirection::FromNegative)?;

    // Works, shape is not extracted
    // let engraving = build_engraving_path(&mesh, &path, Axis::Z, RayDirection::FromNegative)?;

    // Works, shape is not extracted
    // let engraving = build_engraving_path(&mesh, &path, Axis::Z, RayDirection::FromPositive)?;
    
    println!("Engraving length of {} for the path of length {}", engraving.len(), path.len());

    if let Err(e) = write_isometries_to_json("isometries.json", engraving) {
        return Err(e)
    };
    
    Ok(())
}
