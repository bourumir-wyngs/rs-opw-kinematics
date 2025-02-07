use rs_cxx_ros2_opw_bridge::sender::Sender;

use anyhow::{anyhow, ensure, Result};
use parry3d::bounding_volume::Aabb;
use parry3d::math::Point;
use rs_opw_kinematics::annotations::AnnotatedPose;
use rs_opw_kinematics::realsense::{calibrate_realsense, observe_3d};
use rs_opw_kinematics::transform_io::transform_to_json;
use std::fs::File;
use std::io::Write;
use std::thread::sleep;
use nalgebra::Point3;
use rs_opw_kinematics::colors::{ColorId, DefinedColor};
use rs_opw_kinematics::find_transform::base_height;

/// Write a vector of points to a PLY file compatible with MeshLab
fn write_point_cloud_to_ply(points: &[Point<f32>], file_path: &str) -> Result<()> {
    let mut file = File::create(file_path)?;

    // Write the PLY header
    writeln!(file, "ply")?;
    writeln!(file, "format ascii 1.0")?;
    writeln!(file, "element vertex {}", points.len())?;
    writeln!(file, "property float x")?;
    writeln!(file, "property float y")?;
    writeln!(file, "property float z")?;
    writeln!(file, "end_header")?;

    // Write the point data
    for point in points {
        writeln!(file, "{} {} {}", point.x, point.y, point.z)?;
    }

    Ok(())
}

/// Function to filter points that belong to a given AABB
fn filter_points_in_aabb(points: &[Point<f32>], aabb: &Aabb) -> Vec<Point<f32>> {
    points
        .iter()
        .filter(|&point| is_point_in_aabb(point, aabb)) // Filter points inside the AABB
        .cloned() // Clone the points (since we work with references here)
        .collect() // Collect into a new Vec
}

fn filter_points_not_in_aabb(points: &[Point<f32>], aabb: &Aabb) -> Vec<Point<f32>> {
    points
        .iter()
        .filter(|&point| !is_point_in_aabb(point, aabb)) // Filter points inside the AABB
        .cloned() // Clone the points (since we work with references here)
        .collect() // Collect into a new Vec
}

/// Check if a point is inside an AABB

fn is_point_in_aabb(point: &Point<f32>, aabb: &Aabb) -> bool {
    point.x >= aabb.mins.x
        && point.x <= aabb.maxs.x
        && point.y >= aabb.mins.y
        && point.y <= aabb.maxs.y
        && point.z >= aabb.mins.z
        && point.z <= aabb.maxs.z
}

pub fn main() -> anyhow::Result<()> {
    let (serial, transform, ests) = calibrate_realsense()?;
    println!("{:?}", transform);

    // Save to file
    /*
    let json_data = transform_to_json(&serial, &transform);

    // Write the JSON string to the specified file
    let mut file = File::create("calibration.json")?;
    file.write_all(json_data.as_bytes())?;
     */

    let points = observe_3d()?;

    let aabb = Aabb::new(
        Point::new(-0.1, -1.0, 0.0), // Min bounds
        Point::new(0.1, 0.1, 0.35), // Max bounds
    );

    let filtered_points = filter_points_in_aabb(&points, &aabb);
    let unfiltered_points = filter_points_not_in_aabb(&points, &aabb);

    let transformed_points: Vec<Point<f32>> = filtered_points
        .iter()
        .map(|point| transform.transform_point(&point))
        .collect();
    
    let red = ests[&ColorId::Red];
    let green = ests[&ColorId::Green];
    let blue = ests[&ColorId::Blue];

    let vred = vec![red];
    let vgreen = vec![green];
    let vblue = vec![blue];


    let centroid = Point3::new(
        (red.x + green.x + blue.x) / 3.0,
        (red.y + green.y + blue.y) / 3.0,
        (red.z + green.z + blue.z) / 3.0,
    );    
    
    // This is at the height of the color bubble plane
    let t_centroid = transform.transform_point(&centroid.into());
    let base_height = base_height(0.032 + 2.0* 0.01);
    let t_centroid = Point3::new(t_centroid.x, t_centroid.y, t_centroid.z - base_height);

    send_cloud(&unfiltered_points, (200, 0, 0), 0.01);
    send_cloud(&filtered_points, (200, 200, 200), 0.01);
    send_cloud(&vred, (255, 0, 0), 1.0);
    send_cloud(&vgreen, (0, 128, 0), 1.0);
    send_cloud(&vblue, (0, 0, 255), 1.0);
    send_cloud(&vec![centroid], (255, 255, 255), 1.0);
    send_cloud(&vec![t_centroid], (255, 255, 0), 0.5);

    Ok(())
}

fn send_cloud(points: &Vec<Point<f32>>, color: (i32, i32, i32), transparency: f32) -> Result<()> {
    let sender = Sender::new("127.0.0.1", 5555);
    let points: Vec<(f32, f32, f32)> = points.iter().map(|p| (p.x, p.y, p.z)).collect();
    match sender.send_point_cloud_message(
      &points,
     (color.0 as u8, color.1 as u8, color.2 as u8), transparency
    ) {
        Ok(_) => {
            println!("Pose message sent successfully.");
        }
        Err(err) => {
            eprintln!("Failed to send pose message: {}", err);
        }
    }
    sleep(std::time::Duration::from_millis(200));
    Ok(())
}
