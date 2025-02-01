//! This example opens and streams from a sensor, copys the frame data to an OpenCV
//! Mat and shows the frame's contents in an OpenCV Window

use anyhow::{ensure, Error, Result};
use opencv::{core, highgui, imgproc, prelude::*};
use realsense_rust::prelude::FrameEx;
use realsense_rust::{
    config::Config,
    context::Context,
    frame::PixelKind,
    frame::{ColorFrame, DepthFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use rs_opw_kinematics::computer_vision::detect_circle_mat;
use rs_opw_kinematics::hsv::DefinedColor;
use std::time::Instant;
use std::{collections::HashSet, convert::TryFrom, time::Duration};
use realsense_rust::base::Rs2Intrinsics;
use parry3d::math::Point as ParryPoint;

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
            intrinsics.width(), intrinsics.height()
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

fn main() -> Result<()> {
    // Check for depth or color-compatible devices.
    let queried_devices = HashSet::new(); // Query any devices
    let context = Context::new()?;
    let devices = context.query_devices(queried_devices);
    ensure!(!devices.is_empty(), "No devices found");

    // create pipeline
    let pipeline = InactivePipeline::try_from(&context)?;
    let mut config = Config::new();
    config
        .enable_device_from_serial(devices[0].info(Rs2CameraInfo::SerialNumber).unwrap())?
        .disable_all_streams()?
        .enable_stream(Rs2StreamKind::Depth, None, 480, 270, Rs2Format::Z16, 15)?
        .enable_stream(Rs2StreamKind::Color, None, 480, 270, Rs2Format::Rgb8, 15)?;

    // Change pipeline's type from InactivePipeline -> ActivePipeline
    let mut pipeline = pipeline.start(Some(config))?;

    // process frames
    println!("Reading ");
    let timeout = Duration::from_millis(500);
    let mut intrinsics = None;
    let colors = [DefinedColor::green(), DefinedColor::red(), DefinedColor::blue()];
    //let colors = [DefinedColor::blue()];
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
                print!("                          \r");
                for color in colors.iter() {
                    let now_detect = Instant::now();
                    let detection = detect_circle_mat(&color_mat, &color);
                    if let Ok(mut detection) = detection {
                        if let Ok(depth_in_mm) =
                            depth_mat.at_2d::<f32>(detection.y as i32, detection.x as i32)
                        {
                            if let Some(intrinsics) = intrinsics.as_ref() {
                                let point = depth_pixel_to_3d(detection.x as f32, detection.y as f32, *depth_in_mm, intrinsics);
                                print!("{:?}: {:.2}, {:.2}, {:.2}  ", color.id(),   point.x, point.y, point.z);
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
    }

    Ok(())
}
