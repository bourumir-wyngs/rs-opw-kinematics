//! This example opens and streams from a sensor, copys the frame data to an OpenCV
//! Mat and shows the frame's contents in an OpenCV Window

use crate::colors::{ColorId, DefinedColor};
use crate::computer_vision::{detect_circle_mat, IS_MATHING_MIN_THR};
use crate::find_transform::{compute_tetrahedron_geometry, find_transform};
use crate::organized_point::OrganizedPoint;
use anyhow::{anyhow, ensure, Result};
use nalgebra::{Isometry3, Point3, Transform3, Translation3};
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
use std::ffi::CString;
use num_traits::real::Real;
use crate::plane_builder::PlaneBuilder;
use crate::ros_bridge::RosSender;

// We do not need to compute this if we operate in real units.
const BALL_RADIUS: f32 = 0.01;

// Plain bond length surface to surface.
const PLAIN_BOND: f32 = 0.032;

const BOND_LENGTH: f32 = PLAIN_BOND + 2.0 * BALL_RADIUS; // 32 mm bond surface to survace, balls

// Floor level with the calibration target in use.
const FLOOR_LEVEL: f32 = BOND_LENGTH * 1.5;

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

fn color_stats(values: &[(f32, f32, f32)]) -> [f32; 3] {
    if values.is_empty() {
        return [f32::NAN, f32::NAN, f32::NAN]; // Handle empty input gracefully
    }

    // Pre-size vectors to avoid reallocations
    let mut reds = Vec::with_capacity(values.len());
    let mut greens = Vec::with_capacity(values.len());
    let mut blues = Vec::with_capacity(values.len());

    // Separate channels into respective vectors
    for &(r, g, b) in values {
        reds.push(r);
        greens.push(g);
        blues.push(b);
    }

    // Process each channel independently using scalar_stats
    let (median_red, _) = scalar_stats(&reds);
    let (median_green, _) = scalar_stats(&greens);
    let (median_blue, _) = scalar_stats(&blues);

    // Return the array containing the robust estimations for each channel
    [median_red, median_green, median_blue]
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
pub fn calibrate_realsense(serial: &String) -> Result<(String, Transform3<f32>, HashMap<ColorId, ParryPoint<f32>>)>
{
    let sender = RosSender::default();
    
    let (serial, mut pipeline) = open_pipeline(&serial)?;

    let timeout = Duration::from_millis(100);
    let mut intrinsics = None;
    let colors = [
        DefinedColor::green(),
        DefinedColor::red(),
        DefinedColor::blue(),
        DefinedColor::yellow(),
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
                let mut color_mat = mat_from_color(color_frame);

                // Set to black everything too far to be considered.
                let black = opencv::core::Vec3b::from([0, 0, 0]);
                for row in 0..depth_mat.rows() {
                    for col in 0..depth_mat.cols() {
                        // Access the depth at (row, col)
                        let depth_value = *depth_mat.at_2d::<f32>(row, col)?;

                        // If depth is below the threshold, set color to black
                        if depth_value > 0.5 || depth_value < 0.01 {
                            // Set (row, col) in color_mat to black (0, 0, 0)
                            *color_mat.at_2d_mut::<opencv::core::Vec3b>(row, col)? = black;
                        }
                    }
                }


                if intrinsics.is_none() {
                    intrinsics = get_intrinsics_from_depth(depth_frame);
                }


                print!("                          \r{}: ", n);
                for color in colors.iter() {
                    let detection = detect_circle_mat(&color_mat, &color);
                    if let Ok(detection) = detection {
                        if let Ok(depth_in_m) =
                            depth_mat.at_2d::<f32>(detection.y as i32, detection.x as i32)
                        {
                            if let Some(intrinsics) = intrinsics.as_ref() {
                                if *depth_in_m > 0.0 {
                                    let point = depth_pixel_to_3d(
                                        detection.x as f32,
                                        detection.y as f32,
                                        // Account for ball radius, report here center, not surface.
                                        *depth_in_m,
                                        intrinsics,
                                    );
                                    print!(
                                        "{:?}: {:.2}, {:.2}, {:.2} {:.3} ",
                                        color.id(),
                                        point.x,
                                        point.y,
                                        point.z + BALL_RADIUS,
                                        depth_in_m
                                    );
                                    stats.entry(color.id()).or_insert(Vec::new()).push(point);
                                } else {
                                    print!(
                                        "{:?}: [{:}, {:}], {:.3} ",
                                        color.id(),
                                        detection.x,
                                        detection.y,
                                        depth_in_m
                                    );
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
                print!("No depth or color frames available         \r");
                continue 'scan;
            }
        } else {
            print!("No any frames available        \r");
        }
        n = stats.values().map(|v| v.len()).min().unwrap_or(0);
        if n > 200 {
            break 'scan;
        }
    }

    pipeline.stop();

    // Estimate result
    let mut ests = HashMap::with_capacity(3);
    println!("\nEstimates:");
    for (color, points) in stats.iter() {
        let (avg, std_dev) = calculate_stats(&points);
        println!(
            "{:?}: ({:.4} ± {:.4}, {:.4} ± {:.4}, {:.4} ± {:.4})",
            color, avg.x, std_dev.x, avg.y, std_dev.y, avg.z, std_dev.z
        );
        ests.insert(*color, ParryPoint::new(avg.x, avg.y, avg.z));
    }

    let check_result = rr_check(&ests, &serial)?;
    ensure!(check_result, "RR check failed");

    // Prepare calibration points
    let red_obs = ests[&ColorId::Red];
    let green_obs = ests[&ColorId::Green];
    let blue_obs = ests[&ColorId::Blue];
    let yellow_obs = ests[&ColorId::Yellow];

    // Estimates for points path planner and RViz use
    let (red_ref, green_ref, blue_ref, yellow_ref) = compute_tetrahedron_geometry(BOND_LENGTH);
    let transform = find_transform(red_ref, green_ref, blue_ref, yellow_ref, red_obs, green_obs, blue_obs, yellow_obs)?;

    // Adjust zero to be the stand
    let translation = Translation3::new(0.0, 0.0, FLOOR_LEVEL);
    let transform = translation * transform;    

    Ok((serial, transform, ests))
}

pub fn query_devices() -> Result<Vec<String>> {
    let queried_devices = HashSet::new(); // Query any devices
    let context = Context::new()?;
    let devices = context.query_devices(queried_devices);
    Ok(devices.iter().map(|d| d.info(Rs2CameraInfo::SerialNumber).unwrap().to_string_lossy().to_string()).collect())
}

/// Opem pipelinme, returns pipeline and device serial.
fn open_pipeline(serial: &String) -> Result<(String, ActivePipeline)> {
    // Check for depth or color-compatible devices.
    let queried_devices = HashSet::new(); // Query any devices
    let context = Context::new()?;
    let devices = context.query_devices(queried_devices);
    if devices.is_empty() {
        return Err(anyhow!("No devices found"));
    }

    // Create pipeline
    let pipeline = InactivePipeline::try_from(&context)?;

    // Inner function to configure streams and start the pipeline
    fn configure_pipeline(
        serial: &str,
        mut pipeline: InactivePipeline,
    ) -> Result<(String, ActivePipeline)> {
        let mut config = Config::new();
        config
            .enable_device_from_serial(&CString::new(serial)?)?
            .disable_all_streams()?
            .enable_stream(Rs2StreamKind::Depth, None, 480, 270, Rs2Format::Z16, 30)?
            .enable_stream(Rs2StreamKind::Color, None, 480, 270, Rs2Format::Rgb8, 30)?;

        // Change pipeline's type from InactivePipeline -> ActivePipeline
        let active_pipeline = pipeline.start(Some(config))?;
        println!("Reading {}", serial);
        Ok((serial.to_string(), active_pipeline))
    }

    if !serial.is_empty() {
        // Attempt to find and use the device matching the provided serial
        if let Some(device) = devices.iter().find(|device| {
            device
                .info(Rs2CameraInfo::SerialNumber)
                .map(|s| s.to_string_lossy().to_string() == *serial)
                .unwrap_or(false)
        }) {
            println!("Using specified device serial: {:?}", serial);
            configure_pipeline(serial, pipeline)
        } else {
            Err(anyhow!(
                "No device found with the specified serial: {}",
                serial
            ))
        }
    } else {
        // Use the first detected device as default
        let serial = devices[0]
            .info(Rs2CameraInfo::SerialNumber)
            .ok_or_else(|| anyhow!("Failed to retrieve serial number for the default device"))?
            .to_string_lossy()
            .to_string();
        println!("Using first found device: {:?}", serial);
        configure_pipeline(&serial, pipeline)
    }
}

fn rr_check(ests: &HashMap<ColorId, Point3<f32>>, serial: &String) -> Result<bool> {
    if ests.len() < 4 {
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
    let point_y = ests[&ColorId::Yellow];

    // Compute edges
    let rg = distance(&point_r, &point_g);
    let gb = distance(&point_g, &point_b);
    let br = distance(&point_b, &point_r);
    let gy = distance(&point_g, &point_y);
    let by = distance(&point_b, &point_y);
    let ry = distance(&point_r, &point_y);

    // Print edge lengths
    println!("Edges: rg {:.4}, gb {:.4} , br {:.4}, gy {:.4}, by {:.4}, ry {:.4}",
             rg, gb, br, gy, by, ry);

    let sides = [rg, gb, br, gy, by, ry];
    let mean = sides.iter().copied().map(|x| x as f64).sum::<f64>() / sides.len() as f64;
    let max_difference = sides
        .iter()
        .copied()
        .map(|x| ((x as f64 - mean).abs()))
        .fold(f64::NAN, f64::max);

    println!("Side max difference {}", max_difference);

    let tolerance = 0.008;

    let result = max_difference <= tolerance;
    if !result {
        return Err(anyhow!(
            "{}: Sides do not make equilateral triangle:\n   \
            R-G {}, G-B {}, B-R {}, G-Y {}, B-Y {}, R-Y {}. Max difference {} tolerance {}",
            serial,
            rg,
            gb,
            br,
            gy,
            by, 
            ry,
            max_difference,
            tolerance
        ));
    }
    Ok(result)
}

fn convert_mats_to_nested_vector(mats: &Vec<Mat>) -> Result<Vec<Vec<Vec<f32>>>> {
    // Ensure the input vector of Mats is not empty
    let reference = mats
        .first()
        .ok_or(anyhow!("Input vector of Mats is empty"))?;
    let rows = reference.rows();
    let cols = reference.cols();

    // Build the nested vector structure
    let mut result = Vec::with_capacity(rows as usize);

    for row in 0..rows {
        let mut row_vec = Vec::with_capacity(cols as usize);
        for col in 0..cols {
            let mut sample_values = Vec::with_capacity(mats.len());

            // Collect values from the same location (row, col) in each Mat
            for depth_mat in mats {
                // Attempt to retrieve the value at (row, col)
                if let Ok(depth_in_m) = depth_mat.at_2d::<f32>(row, col) {
                    // Only push non-zero values
                    if *depth_in_m > 0.0 {
                        sample_values.push(*depth_in_m);
                    }
                }
                // If retrieval fails, simply skip this value
            }
            row_vec.push(sample_values); // Add column data to the row
        }
        result.push(row_vec); // Add row data to the result
    }

    Ok(result)
}

fn convert_color_mat_to_nested_vector(mats: &Vec<Mat>) -> Result<Vec<Vec<Vec<(f32, f32, f32)>>>> {
    // Ensure the input vector of Mats is not empty
    let reference = mats
        .first()
        .ok_or(anyhow!("Input vector of Mats is empty"))?;
    let rows = reference.rows();
    let cols = reference.cols();

    // Build the nested vector structure
    let mut result = Vec::with_capacity(rows as usize);

    for row in 0..rows {
        let mut row_vec = Vec::with_capacity(cols as usize);
        for col in 0..cols {
            let mut sample_values = Vec::with_capacity(mats.len());

            // Collect values from the same location (row, col) in each Mat
            for color_mat in mats {
                // Attempt to retrieve the value at (row, col)
                if let Ok(color) = color_mat.at_2d::<opencv::core::Vec3b>(row, col) {
                    sample_values.push((color[0] as f32, color[1] as f32, color[2] as f32));
                }
                // If retrieval fails, simply skip this value
            }
            row_vec.push(sample_values); // Add column data to the row
        }
        result.push(row_vec); // Add row data to the result
    }

    Ok(result)
}

pub fn observe_3d_rgb(serial: &String) -> Result<Vec<OrganizedPoint>> {
    let min_samples = 10;
    let n_frames = 12;
    let (serial, mut pipeline) = open_pipeline(&serial)?;
    let timeout = Duration::from_millis(500);
    let mut intrinsics = None;
    let mut depths = Vec::with_capacity(n_frames);
    let mut colors = Vec::with_capacity(n_frames);

    'scan: loop {
        if let Ok(frames) = pipeline.wait(Some(timeout)) {
            let depth_frames = frames.frames_of_type::<DepthFrame>();
            if let Some(depth_frame) = depth_frames.first() {
                if intrinsics.is_none() {
                    intrinsics = get_intrinsics_from_depth(depth_frame);
                }
                let depth_mat = mat_from_depth16(depth_frame);
                depths.push(depth_mat);
            }

            let color_frames = frames.frames_of_type::<ColorFrame>();
            if let Some(color_frame) = color_frames.first() {
                let color_mat = mat_from_color(color_frame);
                colors.push(color_mat);
            }

            if depths.len() > n_frames && colors.len() > n_frames {
                break 'scan;
            }
        }
        print!(
            "\rObserved {} depth frames, {} color frames                        ",
            depths.len(),
            colors.len()
        );
    }
    println!("\n");

    let intrinsics = intrinsics.expect("No intrinsics");
    let reference = depths.first().expect("No depth data");
    let rows = reference.rows() as usize;
    let cols = reference.cols() as usize;
    let nested_vectors = convert_mats_to_nested_vector(&depths)?;
    let nested_colors = convert_color_mat_to_nested_vector(&colors)?;

    // Borrow values immutably
    let nested_ref = &nested_vectors;
    let nested_colors_ref = &nested_colors;
    let intrinsics_ref = &intrinsics;

    use rayon::prelude::*;
    let organized: Vec<_> = (0..rows)
        .into_par_iter()
        .flat_map_iter(|row| {
            (0..cols).filter_map(move |col| {
                if nested_ref[row][col].len() >= min_samples {
                    let depth = scalar_stats(&nested_ref[row][col]).0;
                    let point = depth_pixel_to_3d(col as f32, row as f32, depth, intrinsics_ref);
                    let colors_at = &nested_colors_ref[row][col];
                    let color = color_stats(&colors_at);
                    Some(OrganizedPoint {
                        point,
                        row,
                        col,
                        color: [color[0] as u8, color[1] as u8, color[2] as u8],
                    })
                } else {
                    None
                }
            })
        })
        .collect();

    Ok(organized)
}

pub fn observe_3d_depth(serial: &String) -> Result<Vec<OrganizedPoint>> {
    let min_samples = 4;
    let n_frames = 6;
    let (serial, mut pipeline) = open_pipeline(&serial)?;
    let timeout = Duration::from_millis(500);
    let mut intrinsics = None;
    let mut depths = Vec::with_capacity(n_frames);

    'scan: loop {
        if let Ok(frames) = pipeline.wait(Some(timeout)) {
            let depth_frames = frames.frames_of_type::<DepthFrame>();
            if let Some(depth_frame) = depth_frames.first() {
                if intrinsics.is_none() {
                    intrinsics = get_intrinsics_from_depth(depth_frame);
                }
                let depth_mat = mat_from_depth16(depth_frame);
                depths.push(depth_mat);
            } else {
                print!("No depth frames available");
                continue 'scan;
            }
        } else {
            print!("No any frames available");
        }
        if depths.len() > n_frames {
            break 'scan;
        }
    }
    println!("Observed {} frames", depths.len());

    let intrinsics = intrinsics.expect("No intrinsics");
    let reference = depths.first().expect("No depth data");
    let rows = reference.rows() as usize;
    let cols = reference.cols() as usize;
    let nested_vectors = convert_mats_to_nested_vector(&depths)?;

    // Borrow values immutably
    let nested_ref = &nested_vectors;
    let intrinsics_ref = &intrinsics;

    use rayon::prelude::*;
    let organized: Vec<_> = (0..rows)
        .into_par_iter()
        .flat_map_iter(|row| {
            (0..cols).filter_map(move |col| {
                if nested_ref[row][col].len() >= min_samples {
                    let depth = scalar_stats(&nested_ref[row][col]).0;
                    let point = depth_pixel_to_3d(col as f32, row as f32, depth, intrinsics_ref);
                    Some(OrganizedPoint { point, row, col, color: [0, 0, 0] })
                } else {
                    None
                }
            })
        })
        .collect();

    Ok(organized)
}

