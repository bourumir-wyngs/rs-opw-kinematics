use anyhow::Result;
use nalgebra::Point3;
use parry3d::bounding_volume::Aabb;
use parry3d::math::Point;
use rs_opw_kinematics::colors::ColorId;
use rs_opw_kinematics::find_transform::{base_height, compute_tetrahedron_geometry};
use rs_opw_kinematics::organized_point::{
    filter_points_in_aabb, filter_points_not_in_aabb, OrganizedPoint,
};
use rs_opw_kinematics::realsense::{calibrate_realsense, observe_3d_depth, query_devices};
use rs_opw_kinematics::ros_bridge::RosSender;
use rs_opw_kinematics::transform_io::transform_to_json;
use std::fs::File;
use std::io::Write;

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

pub fn main() -> anyhow::Result<()> {
    let devices = query_devices()?;
    let max_attempts = 4;
    let mut calibrated = 0;
    let total = devices.len();
    for serial in devices {
        println!("**** Calibrating {} ****", serial);
        let mut attempt = 1;
        loop {
            match callibrate(&serial) {
                Ok(_) => {
                    println!("********** Calibration of {} SUCCESSFUL **********", serial);
                    calibrated = calibrated + 1;
                    break;
                }
                Err(e) => {
                    println!(
                        "********** Calibration of {} FAILED: {} **********",
                        serial, e
                    );
                    attempt = attempt + 1;
                    if attempt > max_attempts {
                        println!("********** FAILED all {} attempts **********", max_attempts);
                        return Err(anyhow::anyhow!("Calibration of {} failed", serial));
                    } else {
                        println!(
                            "********** Retrying {} attempt {} out of {} **********",
                            serial, attempt, max_attempts
                        );
                    }
                }
            }
        }
    }
    if calibrated == total {
        println!("********** Calibration of all {} devices successful **********", total);
    } else {
        println!("********** NOT ALL DEVICES OK: {}  out of {} **********", calibrated, total);
    }
    Ok(())
}

fn callibrate(serial: &String) -> Result<()> {
    let sender = RosSender::default();
    let (serial, transform, ests) = calibrate_realsense(&serial)?;
    println!("{:?}", transform);

    // Save to file
    let json_data = transform_to_json(&serial, &transform);

    // Write the JSON string to the specified file
    let mut file = File::create(format!("calibrations/{}.json", serial))?;
    file.write_all(json_data.as_bytes())?;

    let points = observe_3d_depth(&serial)?;

    let aabb = Aabb::new(
        Point::new(-0.1, -1.0, 0.0), // Min bounds
        Point::new(0.1, 0.1, 0.35),  // Max bounds
    );

    let filtered_points = filter_points_in_aabb(&points, &aabb);
    let unfiltered_points = filter_points_not_in_aabb(&points, &aabb);

    let transformed_points: Vec<Point<f32>> = filtered_points
        .iter()
        .map(|point| transform.transform_point(&point.point))
        .collect();

    let red = ests[&ColorId::Red];
    let green = ests[&ColorId::Green];
    let blue = ests[&ColorId::Blue];
    let yellow = ests[&ColorId::Yellow];

    let upper_centroid = Point3::new(
        (red.x + green.x + blue.x) / 3.0,
        (red.y + green.y + blue.y) / 3.0,
        (red.z + green.z + blue.z) / 3.0,
    );

    // This is at the height of the color bubble plane
    let bond = 0.032 + 2.0 * 0.01;
    let t_centroid_c = transform.transform_point(&upper_centroid.into());
    let base_height = base_height(bond);
    let t_centroid = Point3::new(t_centroid_c.x, t_centroid_c.y, t_centroid_c.z - base_height);

    if false {
        //sender.points(&unfiltered_points, (200, 0, 0), 0.01);
        //sender.points(&filtered_points, (200, 200, 200), 0.01);
        sender.points(&transformed_points, (200, 200, 00), 0.5)?;
        sender.points(&vec![red], (255, 0, 0), 1.0)?;
        sender.points(&vec![green], (0, 128, 0), 1.0)?;
        sender.points(&vec![blue], (0, 0, 255), 1.0)?;
        sender.points(&vec![upper_centroid], (255, 255, 255), 1.0)?;
        //sender.points(&vec![t_centroid], (255, 255, 0), 0.5);

        let (rred, rgreen, rblue, ryellow) = compute_tetrahedron_geometry(bond);
        sender.points(&vec![rred, rgreen, rblue, ryellow], (255, 255, 0), 0.2)?;

        sender.points(&vec![transform.transform_point(&red)], (255, 0, 0), 1.0)?;
        sender.points(&vec![transform.transform_point(&green)], (0, 200, 0), 1.0)?;
        sender.points(&vec![transform.transform_point(&blue)], (0, 0, 255), 1.0)?;
        sender.points(
            &vec![transform.transform_point(&yellow)],
            (255, 255, 0),
            1.0,
        )?;

        sender.points(&vec![t_centroid_c], (255, 255, 255), 1.0)?;
    }

    Ok(())
}
