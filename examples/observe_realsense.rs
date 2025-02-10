use rs_cxx_ros2_opw_bridge::sender::Sender;

use anyhow::{Result};
use bevy::render::render_resource::encase::private::RuntimeSizedArray;
use parry3d::bounding_volume::Aabb;
use parry3d::math::Point;
use parry3d::shape::{TriMesh};
use rs_opw_kinematics::dbscan_r::{largest_cluster, Model};
use rs_opw_kinematics::find_transform::compute_tetrahedron_geometry;
use rs_opw_kinematics::realsense::{observe_3dx};
use rs_opw_kinematics::transform_io;
use std::fs::File;
use std::io::{Read};
use std::thread::sleep;
use rs_opw_kinematics::meshbuilder::construct_parry_trimesh;
use rs_opw_kinematics::organized_point::OrganizedPoint;

/// Function to filter points that belong to a given AABB
fn filter_points_in_aabb(points: &Vec<OrganizedPoint>, aabb: &Aabb) -> Vec<OrganizedPoint> {
    points
        .iter()
        .filter(|&point| is_point_in_aabb(&point.point, aabb)) // Filter points inside the AABB
        .cloned() // Clone the points (since we work with references here)
        .collect() // Collect into a new Vec
}

fn filter_points_not_in_aabb(points: &Vec<OrganizedPoint>, aabb: &Aabb) -> Vec<OrganizedPoint> {
    points
        .iter()
        .filter(|&point| !is_point_in_aabb(&point.point, aabb)) // Filter points inside the AABB
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

fn biggest_object_dbscan(
    points: &Vec<OrganizedPoint>,
    r: f32,
    min_points: usize,
) -> Vec<OrganizedPoint> {
    let now = std::time::Instant::now();
    let model = Model::new(r, min_points);
    let output = model.run(points);
    let output_len = output.len();
    let c = largest_cluster(output, points);
    println!(
        "dbscan {:?} cluster size {} -> {}",
        now.elapsed(),
        output_len,
        points.len()
    );
    c
}

fn send_cloud(
    points: &Vec<OrganizedPoint>,
    color: (i32, i32, i32),
    transparency: f32,
) -> Result<()> {
    let sender = Sender::new("127.0.0.1", 5555);
    let points: Vec<(f32, f32, f32)> = points
        .iter()
        .map(|p| (p.point.x, p.point.y, p.point.z))
        .collect();
    match sender.send_point_cloud_message(
        &points,
        &vec![],
        (color.0 as u8, color.1 as u8, color.2 as u8),
        transparency,
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

fn send_mesh(mesh: &TriMesh, color: (i32, i32, i32), transparency: f32) -> Result<()> {
    let sender = Sender::new("127.0.0.1", 5555);
    let points: Vec<(f32, f32, f32)> = mesh.vertices().iter().map(|p| (p.x, p.y, p.z)).collect();
    let triangles = mesh
        .indices()
        .iter()
        .map(|t| (t[0] as u32, t[1] as u32, t[2] as u32))
        .collect();

    match sender.send_point_cloud_message(
        &points,
        &triangles,
        (color.0 as u8, color.1 as u8, color.2 as u8),
        transparency,
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

pub fn main() -> anyhow::Result<()> {
    let file_path = "calibration_ok.json";
    let mut file = File::open(file_path)?;
    let mut json_str = String::new();
    file.read_to_string(&mut json_str)?;

    // Convert JSON string to a Transform3
    let transform = transform_io::json_to_transform(&json_str);

    let points = observe_3dx()?;

    let aabb = Aabb::new(
        Point::new(-0.05, -0.2, 0.035), // Min bounds
        Point::new(0.3, 0.25, 1.0),     // Max bounds
    );

    let _aabb = Aabb::new(
        Point::new(-1., -1., 0.035), // Min bounds
        Point::new(1., 1., 1.0),     // Max bounds
    );

    println!("Observed {} points", points.len());

    let transformed_points: Vec<OrganizedPoint> = points
        .iter()
        .map(|point| OrganizedPoint {
            point: transform.transform_point(&point.point),
            row: point.row,
            col: point.col,
        })
        .collect();

    // Discard, another point must be within 5 mm
    //let transformed_points: Vec<Point<f32>> = filter_points_by_distance(transformed_points, 0.005, 4);

    let filtered_points = filter_points_in_aabb(&transformed_points, &aabb);
    let unfiltered_points = filter_points_not_in_aabb(&transformed_points, &aabb);
    let linfa = biggest_object_dbscan(&filtered_points, 0.03, 20);

    println!(
        "Observed {}, filtered {}, unfiltered {}, linfa {} points",
        points.len(),
        filtered_points.len(),
        unfiltered_points.len(),
        linfa.len()
    );

    let bond = 0.032 + 2.0 * 0.01;
    send_cloud(&linfa, (255, 255, 0), 1.0)?;
    send_cloud(&filtered_points, (200, 200, 200), 0.5)?;
    send_cloud(&unfiltered_points, (200, 0, 0), 0.2)?;

    let (rred, rgreen, rblue) = compute_tetrahedron_geometry(bond);

    send_cloud(
        &vec![
            OrganizedPoint::from_point(rred),
            OrganizedPoint::from_point(rgreen),
            OrganizedPoint::from_point(rblue),
        ],
        (0, 255, 0),
        0.5,
    )?;

    let now = std::time::Instant::now();
    let mesh = construct_parry_trimesh(linfa);
    println!(
        "Trimesh construction took {:?}, indices {}, vertices {}",
        now.elapsed(),
        mesh.indices().len(),
        mesh.vertices().len()
    );
    //send_cloud(&mesh.vertices(),  (0, 225, 0), 0.5)?;
    send_mesh(&mesh, (0, 225, 0), 0.8)?;

    Ok(())
}


