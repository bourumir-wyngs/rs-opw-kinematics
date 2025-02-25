use rs_cxx_ros2_opw_bridge::sender::Sender;
use std::fmt::format;

use bevy_egui::egui::Key::O;
use nalgebra::Isometry3;
use parry3d::bounding_volume::Aabb;
use parry3d::math::Point;
use parry3d::shape::TriMesh;
use rs_opw_kinematics::annotations::AnnotatedPose;
use rs_opw_kinematics::dbscan_r::{closest_centered_object, closest_object};
use rs_opw_kinematics::engraving::{generate_raster_points, project_flat_to_rect_on_mesh};
use rs_opw_kinematics::find_transform::compute_tetrahedron_geometry;
use rs_opw_kinematics::largest_rectangle::largest_rectangle;
use rs_opw_kinematics::mesh_builder::construct_parry_trimesh;
use rs_opw_kinematics::organized_point::{
    filter_points_in_aabb, filter_points_not_in_aabb, OrganizedPoint,
};
use rs_opw_kinematics::plane_builder::PlaneBuilder;
use rs_opw_kinematics::projector::{Axis, Projector, RayDirection};
use rs_opw_kinematics::realsense::{observe_3d_depth, observe_3d_rgb, query_devices};
use rs_opw_kinematics::rect_builder::RectangleEstimator;
use rs_opw_kinematics::ros_bridge::RosSender;
use rs_opw_kinematics::transform_io;
use std::fs::File;
use std::io::Read;

use anyhow::{anyhow, ensure, Result};

pub fn main() -> anyhow::Result<()> {
    let devices = query_devices()?;
    println!("{:?}", devices);
    let mut cloud = Vec::with_capacity(1000 * devices.len());
    for serial in devices {
        println!("**** Reading from {} ****", serial);
        match observe(&serial) {
            Ok(camera_points) => {
                println!("Reading from {} successful", serial);
                cloud.extend(camera_points);
            }
            Err(e) => println!("Reading from {} failed: {}", serial, e),
        }
    }

    let sender = RosSender::default();
    sender.cloud(&cloud, (128, 128, 0), 0.8)?;

    // build a plane from these points

    let plane_builder = PlaneBuilder {
        ..PlaneBuilder::default()
    };

    // Define the rectangle dimensions (width and height) to fit
    let rectangle_width = 0.08;
    let rectangle_height = 0.04;

    // Define the number of RANSAC iterations to try (e.g., 1000)
    let ransac_iterations = 10000;

    let plane = plane_builder
        .build_plane(&cloud, 0.003)?;
    let plane_points = plane.filter(&cloud, 0.006);
    let flattened = plane.flatten(&plane_points);
    //send_cloud(&flattened, (0, 10, 255), 0.2)?;

    println!("Largest rectangle:");
    let largest_rect = largest_rectangle(&flattened);

    // Second iteration - fit rectangle again
    let rectangle_points_2 = RectangleEstimator::ransac_rectangle_fitting(
        &flattened,
        ransac_iterations,
        rectangle_width,
        rectangle_height,
    );

    println!("Plane points: {}", plane_points.len());
    //send_cloud(&plane_points, (0, 0, 255), 1.0)?;
    //send_cloud(&rectangle_points_2, (0, 0, 255), 1.0)?;

    let mesh;
    let rect_available = largest_rect.len() > 3;
    if rect_available {
        mesh = if largest_rect.len() > rectangle_points_2.len() {
            construct_parry_trimesh(largest_rect)
        } else {
            construct_parry_trimesh(rectangle_points_2)
        };

        //let mesh = construct_parry_trimesh(rectangle_points_2);
        //let mesh = construct_parry_trimesh(largest_rect.clone());

        sender.mesh(&mesh, (0, 128, 128), 0.8)?;

        if true {
            let projector = Projector {
                check_points: 64,
                check_points_required: 60,
                radius: 0.0035,
            };

            let path = generate_raster_points(20, 20);

            let a = 0_f32.to_radians();
            let b = 358_f32.to_radians();

            //let stroke = projector.project_cylinder_path_centered(&mesh, &path, a..b, Axis::Z)?;
            let stroke = projector.project_flat_path_fitted(
                &mesh,
                &path,
                plane.most_perpendicular_axis(),
                RayDirection::FromPositive)?;
            let sender = Sender::new("127.0.0.1", 5555);
            sender.send_pose_message(&filter_valid_poses(&stroke))?;
        }

        Ok(())
    } else {
        Err(anyhow::anyhow!("No meshes found"))?
    }
}

pub fn observe(serial: &String) -> anyhow::Result<Vec<OrganizedPoint>> {
    let sender = RosSender::default();
    let file_path = format!("calibrations/{}.json", serial);
    let mut file = File::open(file_path)?;
    let mut json_str = String::new();
    file.read_to_string(&mut json_str)?;

    // Convert JSON string to a Transform3
    let transform = transform_io::json_to_transform(&json_str);

    let points: Vec<OrganizedPoint> = observe_3d_rgb(&serial)?;
    let color_filtered: Vec<OrganizedPoint> = points.iter().cloned().filter(|p| 
        p.color[0] > 64 && p.color[1] > 64 && p.color[2] > 64    
    ).collect();
    //sender.cloud(&color_filtered, (0, 0, 255), 0.2)?;

    let aabb = Aabb::new(
        Point::new(-0.1, -0.1, -0.07), // Min bounds
        Point::new(0.1, 0.1, 0.3),   // Max bounds
    );

    let _aabb = Aabb::new(
        Point::new(-1., -1., 0.0), // Min bounds
        Point::new(1., 1., 1.0),   // Max bounds
    );

    println!("Observed {} points", points.len());

    let transformed_points: Vec<OrganizedPoint> = color_filtered
        .iter()
        .map(|point| OrganizedPoint {
            point: transform.transform_point(&point.point),
            .. *point
        })
        .collect();

    // Discard, another point must be within 5 mm
    //let transformed_points: Vec<Point<f32>> = filter_points_by_distance(transformed_points, 0.005, 4);

    let filtered_points = filter_points_in_aabb(&transformed_points, &aabb);
    let unfiltered_points = filter_points_not_in_aabb(&transformed_points, &aabb);
    let linfa = closest_object(&filtered_points, 0.03, 20, &Point::new(0.0, 0.0, 0.1));

    println!(
        "Observed {}, filtered {}, unfiltered {}, linfa {} points",
        points.len(),
        filtered_points.len(),
        unfiltered_points.len(),
        linfa.len()
    );
    sender.cloud(&unfiltered_points, (255, 0, 0), 0.2)?;

    Ok(linfa)
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