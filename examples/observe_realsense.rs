use rs_cxx_ros2_opw_bridge::sender::Sender;
use std::f32::consts::PI;

use anyhow::Result;
use bevy::render::render_resource::encase::private::RuntimeSizedArray;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use parry3d::bounding_volume::Aabb;
use parry3d::math::Point;
use parry3d::shape::TriMesh;
use rs_opw_kinematics::annotations::AnnotatedPose;
use rs_opw_kinematics::dbscan_r::{cluster_stats, Model};
use rs_opw_kinematics::engraving::{generate_raster_points, project_flat_to_rect_on_mesh};
use rs_opw_kinematics::find_transform::compute_tetrahedron_geometry;
use rs_opw_kinematics::mesh_builder::construct_parry_trimesh;
use rs_opw_kinematics::organized_point::OrganizedPoint;
use rs_opw_kinematics::projector::{Axis, Projector, RayDirection};
use rs_opw_kinematics::realsense::{observe_3d_rgb, observe_3d_depth};
use rs_opw_kinematics::transform_io;
use std::fs::File;
use std::io::Read;
use std::thread::sleep;
use rs_opw_kinematics::plane_builder::build_plane;
use rs_opw_kinematics::rect_builder::RectangleEstimator;

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

fn best_object_dbscan(
    points: &Vec<OrganizedPoint>,
    r: f32,
    min_points: usize,
) -> Vec<OrganizedPoint> {
    let now = std::time::Instant::now();
    let model = Model::new(r, min_points);
    let output = model.run(points);
    let output_len = output.len();
    let c;
    let mut stats = cluster_stats(&output, points);

    // Take the object closest to the X axis.
    stats.sort_by(|a, b| {
        a.center
            .y
            .abs()
            .partial_cmp(&b.center.y.abs())
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    if let Some(stats) = stats.first() {
        c = stats.points(&output, points);
    } else {
        c = Vec::new();
    }

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

    let points = observe_3d_depth()?;

    let aabb = Aabb::new(
        Point::new(-0.05, -0.2, 0.035), // Min bounds
        Point::new(0.3, 0.25, 1.0),     // Max bounds
    );

    let aabb = Aabb::new(
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
    let linfa = best_object_dbscan(&filtered_points, 0.03, 20);

    println!(
        "Observed {}, filtered {}, unfiltered {}, linfa {} points",
        points.len(),
        filtered_points.len(),
        unfiltered_points.len(),
        linfa.len()
    );

    let bond = 0.032 + 2.0 * 0.01;
    send_cloud(&linfa, (255, 255, 0), 0.2)?;
    //send_cloud(&filtered_points, (200, 200, 200), 0.5)?;
    //send_cloud(&unfiltered_points, (200, 0, 0), 0.2)?;

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
    let mesh = construct_parry_trimesh(linfa.clone());
    println!(
        "Trimesh construction took {:?}, indices {}, vertices {}",
        now.elapsed(),
        mesh.indices().len(),
        mesh.vertices().len()
    );
    //send_cloud(&mesh.vertices(),  (0, 225, 0), 0.5)?;
    send_mesh(&mesh, (0, 128, 128), 0.8)?;


    // Define the rectangle dimensions (width and height) to fit
    let rectangle_width = 0.08;
    let rectangle_height = 0.04;

    // Define the number of RANSAC iterations to try (e.g., 1000)
    let ransac_iterations = 10000;

    // Call the RANSAC rectangle fitting estimator
    println!("Fitting a rectangle on the provided points...");
    match RectangleEstimator::ransac_rectangle_fitting(&linfa, ransac_iterations, rectangle_width, rectangle_height) {
        Some(model) => {
            send_cloud(&model, (0, 0, 255), 1.0)?;
        }
        None => {
            println!("Failed to fit a rectangle on the provided points.");
        }
    }
    
    if false {
    let projector = Projector {
        check_points: 64,
        check_points_required: 60,
        radius: 0.005,
    };

    let path = generate_raster_points(20, 20);
    if false {
        let a = 0_f32.to_radians();
        let b = 180_f32.to_radians();

        if let Ok(stroke) = projector.project_cylinder_path_centered(
            &mesh,
            &path,
            a..b,
            Axis::Z,
        ) {
            let sender = Sender::new("127.0.0.1", 5555);
            sender.send_pose_message(&filter_valid_poses(&stroke))?;
        } else {
            println!("Projection failed.");
        }
    } else {
        // flat projection
        if let Ok(stroke) =
            projector.project_flat_path_fitted(&mesh, &path, Axis::X, RayDirection::FromNegative)
        {
            println!("Projection succeeded, {} poses.", stroke.len());
            let sender = Sender::new("127.0.0.1", 5555);
            sender.send_pose_message(&filter_valid_poses(&stroke))?;
        } else {
            println!("Projection failed.");
        }
    }
        }

    Ok(())
}

pub fn filter_valid_poses(poses: &Vec<AnnotatedPose>) -> Vec<Isometry3<f64>> {
    poses
        .iter()
        .filter(|pose| {
            // Extract translation and rotation components
            let translation = pose.pose.translation.vector;
            let rotation = pose.pose.rotation;

            // Check for NaN values in translation and rotation
            if translation.x.is_nan()
                || translation.y.is_nan()
                || translation.z.is_nan()
                || rotation.i.is_nan()
                || rotation.j.is_nan()
                || rotation.k.is_nan()
                || rotation.w.is_nan()
            {
                return false; // NaN values -> invalid
            }

            // Check for zero-length quaternion
            let quaternion_magnitude =
                (rotation.i.powi(2) + rotation.j.powi(2) + rotation.k.powi(2) + rotation.w.powi(2))
                    .sqrt();
            if quaternion_magnitude < 1e-6 {
                // Threshold to consider near zero-length
                return false; // Zero-length quaternion -> invalid
            }

            // Pose passes all checks -> valid
            true
        })
        .map(|pose| pose.pose)
        .collect()
}
