//! This example opens and streams from a sensor, copys the frame data to an OpenCV
//! Mat and shows the frame's contents in an OpenCV Window

use anyhow::{ensure, Result};
use opencv::{core, highgui, imgproc, prelude::*};
use realsense_rust::{
    config::Config,
    context::Context,
    frame::PixelKind,
    frame::{ColorFrame, DepthFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use std::{collections::HashSet, convert::TryFrom, time::Duration};
use std::time::Instant;
use rs_opw_kinematics::computer_vision::{detect_circle_mat};
use rs_opw_kinematics::hsv::DefinedColor;

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

/// Colorizes a single channel OpenCV mat. The mat's current
/// range will be mapped to a [0..255] range and then a color map
/// is applied.
fn colorized_mat(mat: &core::Mat) -> core::Mat {
    let mut normalized = Mat::default();
    core::normalize(
        &mat,
        &mut normalized,
        0.0,
        255.0,
        core::NORM_MINMAX,
        core::CV_8UC1,
        &core::no_array(),
    )
        .unwrap();

    let mut colorized = Mat::default();
    imgproc::apply_color_map(&normalized, &mut colorized, imgproc::COLORMAP_JET).unwrap();

    colorized
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
        .enable_stream(Rs2StreamKind::Depth, None, 1280, 0, Rs2Format::Z16, 15)?
        .enable_stream(Rs2StreamKind::Color, None, 1280, 0, Rs2Format::Rgb8, 15)?;

    // Change pipeline's type from InactivePipeline -> ActivePipeline
    let mut pipeline = pipeline.start(Some(config))?;

    // process frames
    let timeout = Duration::from_millis(500);
    let color = DefinedColor::green();
    loop {
        let nf = Instant::now();
        if let Ok(frames) = pipeline.wait(Some(timeout)) {
            let color_frames = frames.frames_of_type::<ColorFrame>();
            let depth_frames = frames.frames_of_type::<DepthFrame>();

            // There is only one depth and one color stream configured.
            if let Some(color_frame) = color_frames.first() {
                let now_mat = Instant::now();
                let color_mat = mat_from_color(color_frame);
                let now_detect = Instant::now();
                let detection = detect_circle_mat(&color_mat, &color);
                let center_str = format!("Detection: {:?} in {:?}", detection, now_detect.elapsed());
                println!("{}", center_str);
            } else { 
                println!("No color frames available");
            }

            if let Some(depth_frame) = depth_frames.first() {
                let depth_mat = mat_from_depth16(depth_frame);
            } else {
                println!("No depth frames available");
            }            
        } else {
            println!("No frames available");
        }
    }

    Ok(())
}
