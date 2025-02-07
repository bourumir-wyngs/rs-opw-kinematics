use std::collections::HashMap;
use rs_cxx_ros2_opw_bridge::sender::Sender;

use anyhow::{anyhow, ensure, Result};
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use nalgebra::{min, Point3};
use parry3d::bounding_volume::Aabb;
use parry3d::math::Point;
use parry3d::query::distance;
use rs_opw_kinematics::annotations::AnnotatedPose;
use rs_opw_kinematics::colors::{ColorId, DefinedColor};
use rs_opw_kinematics::find_transform::{base_height, compute_tetrahedron_geometry};
use rs_opw_kinematics::realsense::{calibrate_realsense, observe_3d};
use rs_opw_kinematics::transform_io;
use rs_opw_kinematics::transform_io::transform_to_json;
use std::fs::File;
use std::io::{Read, Write};
use std::thread::sleep;
use dbscan::{Classification, Model};
use geo::Euclidean;
use ndarray::{array, Array2};

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

fn filter_points_by_distance(
    points: Vec<Point<f32>>,
    r: f32,
    n_neighbours: usize,
) -> Vec<Point<f32>> {
    // Create a KdTree for 3D points (3 dimensions).
    let mut kdtree = KdTree::new(3);

    // Populate the tree with points and their indices.
    for (i, point) in points.iter().enumerate() {
        kdtree.add([point.x, point.y, point.z], i).unwrap();
    }

    // Prepare the result vector for filtered points.
    let mut filtered_points = Vec::new();

    // Square the distance threshold to avoid extra square root operations.
    let r_squared = r * r;

    // Iterate over all points and check for neighbors within distance `r`.
    for point in points.iter() {
        let neighbors = kdtree
            .within(&[point.x, point.y, point.z], r_squared, &squared_euclidean)
            .unwrap();

        // If there is more than 1 neighbor (itself + others), retain the point.
        if neighbors.len() >= n_neighbours {
            filtered_points.push(*point);
        }
    }

    filtered_points
}


fn largest_cluster(output: Vec<Classification>, points: &Vec<Point<f32>>) -> Vec<Point<f32>> {
    let mut cluster_counts: HashMap<usize, usize> = HashMap::new();

    // Count occurrences of each cluster (excluding Noise)
    for classification in output.iter() {
        if let Classification::Core(cluster_id) = classification {
            *cluster_counts.entry(*cluster_id).or_insert(0) += 1;
        }
    }

    // Find the largest cluster
    let largest_cluster_id = cluster_counts.into_iter()
        .max_by_key(|&(_, count)| count)
        .map(|(id, _)| id);

    // Filter output to keep only the largest cluster
    if let Some(largest_cluster_id) = largest_cluster_id {
        points
            .iter()
            .zip(output.iter()) // Combine points and output into an iterator of pairs
            .filter_map(|(point, classification)| match classification {
                Classification::Core(cluster_id) | Classification::Edge(cluster_id) if *cluster_id == largest_cluster_id => {
                    Some(*point)
                }
                _ => None,
            })
            .collect() // Collect the filtered points into a Vec<Point<f32>>
    } else {
        Vec::new()
    }
}

fn biggest_object_dbscan(points: &Vec<Point<f32>>, r: f32, min_points: usize) -> Vec<Point<f32>> {
    let model = Model::new(r as f64, min_points);
    let inputs: Vec<Vec<f32>> = points
        .iter()
        .map(|p| vec![p.x, p.y, p.z]) // Create a Vec<f32> for each point
        .collect();
    let output = model.run(&inputs);
    largest_cluster(output, points)
}


pub fn main() -> anyhow::Result<()> {
    let file_path = "calibration_ok.json";
    let mut file = File::open(file_path)?;
    let mut json_str = String::new();
    file.read_to_string(&mut json_str)?;

    // Convert JSON string to a Transform3
    let transform = transform_io::json_to_transform(&json_str);

    let points = observe_3d()?;

    let aabb = Aabb::new(
        Point::new(-0.05, -0.2, 0.035), // Min bounds
        Point::new(0.3, 0.25, 1.0),      // Max bounds
    );

    let transformed_points: Vec<Point<f32>> = points
        .iter()
        .map(|point| transform.transform_point(&point))
        .collect();

    // Discard, another point must be within 5 mm
    //let transformed_points: Vec<Point<f32>> = filter_points_by_distance(transformed_points, 0.005, 4);
    
    let filtered_points = filter_points_in_aabb(&transformed_points, &aabb);
    let unfiltered_points = filter_points_not_in_aabb(&transformed_points, &aabb);

    let linfa = biggest_object_dbscan(&filtered_points, 0.03, 20);    
    
    let bond = 0.032 + 2.0 * 0.01;
    send_cloud(&linfa, (255, 255, 0), 1.0);
    send_cloud(&filtered_points, (200, 200, 200), 0.5);
    send_cloud(&unfiltered_points, (200, 0, 0), 0.2);

    let (rred, rgreen, rblue) = compute_tetrahedron_geometry(bond);

    send_cloud(&vec![rred, rgreen, rblue], (0, 255, 0), 1.0);

    Ok(())
}

fn send_cloud(points: &Vec<Point<f32>>, color: (i32, i32, i32), transparency: f32) -> Result<()> {
    let sender = Sender::new("127.0.0.1", 5555);
    let points: Vec<(f32, f32, f32)> = points.iter().map(|p| (p.x, p.y, p.z)).collect();
    match sender.send_point_cloud_message(
        &points,
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
