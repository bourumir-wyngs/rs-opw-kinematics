//! This example opens and streams from a sensor, copys the frame data to an OpenCV
//! Mat and shows the frame's contents in an OpenCV Window

use crate::computer_vision::detect_circle_mat;
use crate::find_transform::{compute_tetrahedron_geometry, find_transform};
use crate::colors::{ColorId, DefinedColor};
use anyhow::{anyhow, ensure, Result};
use nalgebra::{Isometry3, Point3, Transform3};
use opencv::{core, prelude::*};
use parry3d::math::{Point as ParryPoint, Point};
use realsense_rust::base::Rs2Intrinsics;
use realsense_rust::pipeline::ActivePipeline;
use realsense_rust::prelude::FrameEx;
use realsense_rust::{
    config::Config,
    context::Context,
    frame::PixelKind,
    frame::{ColorFrame, DepthFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use std::collections::HashMap;
use std::{collections::HashSet, convert::TryFrom, time::Duration};

// We do not need to compute this if we operate in real units.
const BALL_RADIUS: f32 = 0.01;

// Plain bond length surface to surface.
const PLAIN_BOND: f32 = 0.032;

/// Converts a RealSense ColorFrame with BGR8 color to an
/// OpenCV mat also with BGR8 color
fn mat_from_color(color_frame: &ColorFrame) -> core::Mat {
    let mut color_mat = unsafe {
        Mat::new_rows_cols(
            color_frame.height() as i32,
            color_frame.width() as i32,
            core::CV_8UC3,
        )
        .unwrap()
    };

    for (i, rs) in color_frame.iter().enumerate() {
        match rs {
            PixelKind::Bgr8 { b, g, r } => {
                *color_mat.at_mut::<opencv::core::Vec3b>(i as i32).unwrap() = [*b, *g, *r].into();
            }
            _ => panic!("We got our types wrong!"),
        }
    }

    color_mat
}

/// Converts a RealSense DepthFrame with 16-bit depth to a
/// floating point OpenCV mat with depth in meters.
fn mat_from_depth16(depth_frame: &DepthFrame) -> core::Mat {
    let mut depth_mat = unsafe {
        Mat::new_rows_cols(
            depth_frame.height() as i32,
            depth_frame.width() as i32,
            core::CV_32F,
        )
        .unwrap()
    };

    let depth_unit = depth_frame.depth_units().unwrap();
    for (i, rs) in depth_frame.iter().enumerate() {
        match rs {
            PixelKind::Z16 { depth } => {
                let depthf = *depth as f32 * depth_unit;
                *depth_mat.at_mut::<f32>(i as i32).unwrap() = depthf;
            }
            _ => panic!("We got our types wrong!"),
        }
    }

    depth_mat
}

fn get_intrinsics_from_depth(depth_frame: &DepthFrame) -> Option<Rs2Intrinsics> {
    // Get an active stream profile for the depth frame
    let stream_profile = depth_frame.stream_profile();
    let intrinsics = stream_profile.intrinsics(); // Fetch intrinsics for the stream

    if let Ok(intrinsics) = intrinsics {
        // Print out the intrinsics
        println!("Camera Intrinsics:");
        println!("  fx = {}, fy = {}", intrinsics.fx(), intrinsics.fy());
        println!("  cx = {}, cy = {}", intrinsics.ppx(), intrinsics.ppy());
        println!(
            "  Width = {}, Height = {}",
            intrinsics.width(),
            intrinsics.height()
        );
        Some(intrinsics)
    } else {
        None
    }
}

fn depth_pixel_to_3d(u: f32, v: f32, z: f32, intr: &Rs2Intrinsics) -> ParryPoint<f32> {
    // Using the pinhole camera model:
    // X = (u - cx) / fx * z
    // Y = (v - cy) / fy * z
    // Z = z
    let x = (u - intr.ppx()) / intr.fx() * z;
    let y = (v - intr.ppy()) / intr.fy() * z;

    ParryPoint::new(x, y, z)
}

// Internal function to calculate average and standard deviation for scalar vector
// (median, MAD and outlier rejection)
fn scalar_stats(values: &[f32]) -> (f32, f32) {
    let n = values.len();
    if n == 0 {
        return (f32::NAN, f32::NAN); // Handle empty input gracefully
    }

    // Step 1: Calculate the median for central tendency
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let median = if n % 2 == 0 {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
    } else {
        sorted[n / 2]
    };

    // Step 2: Compute the MAD (Median Absolute Deviation)
    let deviations: Vec<f32> = sorted.iter().map(|&v| (v - median).abs()).collect();
    let mad = {
        let mut sorted_deviations = deviations.clone();
        sorted_deviations.sort_by(|a, b| a.partial_cmp(b).unwrap());
        if n % 2 == 0 {
            (sorted_deviations[n / 2 - 1] + sorted_deviations[n / 2]) / 2.0
        } else {
            sorted_deviations[n / 2]
        }
    };

    let scaling_factor = 1.4826; // To approximate standard deviation
    let robust_std_dev = mad * scaling_factor;

    // Step 3: Optional Outlier Filtering (Using MAD)
    let lower_bound = median - 3.0 * robust_std_dev;
    let upper_bound = median + 3.0 * robust_std_dev;

    let filtered: Vec<f32> = values
        .iter()
        .cloned()
        .filter(|v| *v >= lower_bound && *v <= upper_bound)
        .collect();

    // If outliers were filtered, recalculate the mean and robust standard deviation
    if filtered.len() != values.len() {
        let recalculated_mean = filtered.iter().sum::<f32>() / filtered.len() as f32;

        let recalculated_variance = filtered
            .iter()
            .map(|&value| (value - recalculated_mean).powi(2))
            .sum::<f32>()
            / filtered.len() as f32;

        let recalculated_std_dev = recalculated_variance.sqrt();
        return (recalculated_mean, recalculated_std_dev);
    }

    // Return the robust metrics if no filtering was done
    (median, robust_std_dev)
}

// Calculate stats for the full vector of ParryPoints
fn calculate_stats(points: &[ParryPoint<f32>]) -> (ParryPoint<f32>, ParryPoint<f32>) {
    // Extract scalar values for x, y, and z
    let xs: Vec<f32> = points.iter().map(|p| p.x).collect();
    let ys: Vec<f32> = points.iter().map(|p| p.y).collect();
    let zs: Vec<f32> = points.iter().map(|p| p.z).collect();

    // Aggregate stats for each dimension
    let (avg_x, std_dev_x) = scalar_stats(&xs);
    let (avg_y, std_dev_y) = scalar_stats(&ys);
    let (avg_z, std_dev_z) = scalar_stats(&zs);

    // Return as ParryPoint for averages and standard deviations
    (
        ParryPoint::new(avg_x, avg_y, avg_z),
        ParryPoint::new(std_dev_x, std_dev_y, std_dev_z),
    )
}

/// Callibrate Realsense device. This function requires to have RealSense device connected,
/// and calibration target present in the view. The target is the tetrahedron, assembled
/// from plastic balls of molecular modelling kit (kind of methane), standing on the apex,
/// with top 3 apexes making the XY plane. The bottom apex (on what the tetrahedron is standing)
/// is considered the origin of the coordinates. Z axis points upwards from it.
/// The balls on the top must be green, blue and red (clockwise). Green ball is pointing in the
/// direction of Y axis.
///
///  The function computes Transform from the camera system to the system coordinates as
///  described above.
pub fn calibrate_realsense() -> Result<(String, Isometry3<f32>, HashMap<ColorId, ParryPoint<f32>>)> {
    let (serial, mut pipeline) = open_pipeline()?;

    let timeout = Duration::from_millis(500);
    let mut intrinsics = None;
    let colors = [
        DefinedColor::green(),
        DefinedColor::red(),
        DefinedColor::blue(),
    ];
    let mut n = 0;
    let mut stats = HashMap::new();

    'scan: loop {
        if let Ok(frames) = pipeline.wait(Some(timeout)) {
            let color_frames = frames.frames_of_type::<ColorFrame>();
            let depth_frames = frames.frames_of_type::<DepthFrame>();

            // There is only one depth and one color stream configured.
            if let (Some(color_frame), Some(depth_frame)) =
                (color_frames.first(), depth_frames.first())
            {
                let depth_mat = mat_from_depth16(depth_frame);
                if intrinsics.is_none() {
                    intrinsics = get_intrinsics_from_depth(depth_frame);
                }

                let color_mat = mat_from_color(color_frame);
                print!("                          \r{}: ", n);
                for color in colors.iter() {
                    let detection = detect_circle_mat(&color_mat, &color);
                    if let Ok(detection) = detection {
                        if let Ok(depth_in_m) =
                            depth_mat.at_2d::<f32>(detection.y as i32, detection.x as i32)
                        {
                            if let Some(intrinsics) = intrinsics.as_ref() {
                                if *depth_in_m > 0.01 {
                                    let point = depth_pixel_to_3d(
                                        detection.x as f32,
                                        detection.y as f32,
                                        // Account for ball radius, report here center, not surface.
                                        *depth_in_m,
                                        intrinsics,
                                    );
                                    print!(
                                        "{:?}: {:.2}, {:.2}, {:.2}",
                                        color.id(),
                                        point.x,
                                        point.y,
                                        point.z + BALL_RADIUS,
                                    );
                                    stats.entry(color.id()).or_insert(Vec::new()).push(point);
                                }
                            }
                        }
                        //let center_str = format!(" Final {:?} in {:?}", detection, now_detect.elapsed());
                    } else {
                        print!("No detection for {:?}   ", color);
                    }
                    print!(" ")
                }
            } else {
                print!("No depth or color frames available");
                continue 'scan;
            }
        } else {
            print!("No any frames available");
        }
        n = stats.values().map(|v| v.len()).min().unwrap_or(0);
        if n > 100 {
            break 'scan;
        }
    }

    // Estimate result
    let mut ests = HashMap::with_capacity(3);

    for (color, points) in stats.iter() {
        let (avg, std_dev) = calculate_stats(&points);
        println!(
            "{:?}: ({:.4} ± {:.4}, {:.4} ± {:.4}, {:.4} ± {:.4})",
            color, avg.x, std_dev.x, avg.y, std_dev.y, avg.z, std_dev.z
        );
        ests.insert(*color, ParryPoint::new(avg.x, avg.y, avg.z));
    }

    let check_result = rr_check(&ests)?;
    ensure!(check_result, "RR check failed");

    // Prepare calibration points
    let red_obs = ests[&ColorId::Red];
    let green_obs = ests[&ColorId::Green];
    let blue_obs = ests[&ColorId::Blue];

    // Estimates for points path planner and RViz use
    let bond = PLAIN_BOND + 2.0 * BALL_RADIUS; // 32 mm bond surface to survace, balls
    let (red_ref, green_ref, blue_ref) = compute_tetrahedron_geometry(bond);
    let transform = find_transform(red_ref, green_ref, blue_ref, red_obs, green_obs, blue_obs);

    println!("Transform: {:?}", transform);

    Ok((serial, transform, ests))
}

/// Opem pipelinme, returns pipeline and device serial.
fn open_pipeline() -> Result<(String, ActivePipeline)> {
    // Check for depth or color-compatible devices.
    let queried_devices = HashSet::new(); // Query any devices
    let context = Context::new()?;
    let devices = context.query_devices(queried_devices);
    if devices.is_empty() {
        return Err(anyhow!("No devices found"));
    }

    // create pipeline
    let pipeline = InactivePipeline::try_from(&context)?;
    let mut config = Config::new();
    let serial = devices[0].info(Rs2CameraInfo::SerialNumber).unwrap();
    println!("Using device: {:?}", serial);
    config
        .enable_device_from_serial(serial)?
        .disable_all_streams()?
        .enable_stream(Rs2StreamKind::Depth, None, 480, 270, Rs2Format::Z16, 30)?
        .enable_stream(Rs2StreamKind::Color, None, 480, 270, Rs2Format::Rgb8, 30)?;

    // Change pipeline's type from InactivePipeline -> ActivePipeline
    let mut pipeline = pipeline.start(Some(config))?;
    let serial = serial.to_string_lossy().to_string();
    // process frames
    println!("Reading {}", &serial);
    Ok((serial, pipeline))
}

fn rr_check(ests: &HashMap<ColorId, Point3<f32>>) -> Result<bool> {
    if ests.len() < 3 {
        return Err(anyhow!("Not all points detected, only {:?}", ests.keys()));
    }

    println!("Estimated distance between balls");
    use parry3d::math::Point as ParryPoint;

    fn distance(p1: &ParryPoint<f32>, p2: &ParryPoint<f32>) -> f32 {
        ((p1.x - p2.x).powi(2) + (p1.y - p2.y).powi(2) + (p1.z - p2.z).powi(2)).sqrt()
    }

    // Example points
    let point_r = ests[&ColorId::Red];
    let point_g = ests[&ColorId::Green];
    let point_b = ests[&ColorId::Blue];

    // Compute edges
    let rg = distance(&point_r, &point_g);
    let gb = distance(&point_g, &point_b);
    let br = distance(&point_b, &point_r);

    // Print edge lengths
    println!("Edges: rg {:.4}, gb {:.4} , br{:.4},", rg, gb, br);
    let sides = [rg, gb, br];
    let mean = sides.iter().copied().map(|x| x as f64).sum::<f64>() / sides.len() as f64;
    let max_difference = sides
        .iter()
        .copied()
        .map(|x| ((x as f64 - mean).abs()))
        .fold(f64::NAN, f64::max);

    println!("Side max difference {}", max_difference);

    let tolerance = 0.003;

    let result = max_difference <= tolerance;
    if !result {
        return Err(anyhow!(
            "Red, green and blue balls do not make equilateral triangle: R-G {}, G-B {}, B-R {}",
            rg,
            gb,
            br
        ));
    }
    Ok(result)
}

pub fn observe_3d() -> Result<Vec<ParryPoint<f32>>> {
    let (serial, mut pipeline) = open_pipeline()?;
    let timeout = Duration::from_millis(500);
    let mut intrinsics = None;
    let mut n = 0;
    let mut clouds = Vec::with_capacity(2);

    'scan: loop {
        if let Ok(frames) = pipeline.wait(Some(timeout)) {
            let depth_frames = frames.frames_of_type::<DepthFrame>();
            // if let Some(intrinsics) = intrinsics.as_ref() {
            // There is only one depth and one color stream configured.

            if let Some(depth_frame) = depth_frames.first() {
                if intrinsics.is_none() {
                    intrinsics = get_intrinsics_from_depth(depth_frame);
                }
                let depth_mat = mat_from_depth16(depth_frame);
                let mut points = Vec::with_capacity((depth_mat.rows() * depth_mat.cols()) as usize);
                for row in 0..depth_mat.rows() {
                    for col in 0..depth_mat.cols() {
                        if let Ok(depth_in_m) = depth_mat.at_2d::<f32>(row, col) {
                            if let Some(intrinsics) = intrinsics.as_ref() {
                                if *depth_in_m > 0.01 {
                                    let point = depth_pixel_to_3d(
                                        col as f32,
                                        row as f32,
                                        *depth_in_m,
                                        intrinsics,
                                    );
                                    points.push(point);
                                }
                            }
                        }
                    }
                }
                clouds.push(points);
                points = Vec::new();
            } else {
                print!("No depth frames available");
                continue 'scan;
            }            
        } else {
            print!("No any frames available");
        }
        n = clouds.len();
        if n >= 2 {
            break 'scan;
        }
    }

    if clouds.is_empty() {
        Err(anyhow!("Failed to get mesh"))
    } else {
        println!("zeferved {} points, {} clouds from {}", clouds[0].len(), clouds.len(), serial);
        Ok(clouds[0].clone())
    }
}
