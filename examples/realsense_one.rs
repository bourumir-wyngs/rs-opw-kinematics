//! Configure and stream a 435i sensor.
//!
//! Notice that the streaming configuration changes based on the USB speed of the sensor.
//! If one attemps to set a streaming configuration that is too much for the current USB
//! speed, RealSense will return with an error. However, that error is non-descript and will
//! not help identify the underlying problem, i.e. the bandwidth of the connection.

use anyhow::{ensure, Result};
use realsense_rust::frame::{ColorFrame, PixelKind};
use realsense_rust::{
    config::Config,
    context::Context,
    frame::{DepthFrame, GyroFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2ProductLine, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use std::{
    collections::HashSet,
    convert::TryFrom,
    io::{self, Write},
    time::Duration,
};
use image::{DynamicImage, ImageBuffer, Rgb};

fn main() -> Result<()> {
    // Check for depth or color-compatible devices.
    let mut queried_devices = HashSet::new();
    queried_devices.insert(Rs2ProductLine::D400);
    let context = Context::new()?;
    let devices = context.query_devices(queried_devices);
    ensure!(!devices.is_empty(), "No devices found");

    // Get device information. For now, we only expect device 0 to be present.
    let serial_number = devices[0]
        .info(Rs2CameraInfo::SerialNumber)
        .expect("Failed to get SerialNumber");
    let usb_cstr = devices[0]
        .info(Rs2CameraInfo::UsbTypeDescriptor)
        .expect("Failed to get USB Type Descriptor");

    println!(
        "Device serial {}, USB Type: {}",
        serial_number.to_string_lossy(),
        usb_cstr.to_string_lossy()
    );

    let pipeline = InactivePipeline::try_from(&context)?;
    let mut config = Config::new();

    config
        .enable_device_from_serial(serial_number)?
        .disable_all_streams()?
        .enable_stream(Rs2StreamKind::Depth, None, 1280, 0, Rs2Format::Z16, 15)?
        .enable_stream(Rs2StreamKind::Color, None, 1280, 0, Rs2Format::Rgb8, 15)?;

    // Change pipeline's type from InactivePipeline -> ActivePipeline
    let mut pipeline = pipeline.start(Some(config))?;

    // process frames
    for i in 0..10 {
        let timeout = Duration::from_millis(5000);
        let frames = pipeline.wait(Some(timeout))?;

        let mut dw = 0;
        let mut dh = 0;
        let mut rw = 0;
        let mut rh = 0;

        let mut distance = 0.0;
        let mut rr = 0;
        let mut gg = 0;
        let mut bb = 0;

        // Get depth
        let mut depth_frames = frames.frames_of_type::<DepthFrame>();
        if !depth_frames.is_empty() {
            let depth_frame = depth_frames.pop().unwrap();
            dw = depth_frame.width();
            dh = depth_frame.height();
            distance = depth_frame.distance(depth_frame.width() / 2, depth_frame.height() / 2)?;
        }

        let mut rgb_frames = frames.frames_of_type::<ColorFrame>();
        if !rgb_frames.is_empty() {
            let rbg_frame = rgb_frames.pop().unwrap();
            rw = rbg_frame.width();
            rh = rbg_frame.height();
            let pixel = rbg_frame
                .get(rbg_frame.width() / 2, rbg_frame.height() / 2)
                .unwrap();
            match pixel {
                PixelKind::Rgb8 { b, g, r } => {
                    bb = *b;
                    rr = *r;
                    gg = *g;
                }
                _ => {
                    println!("The pixel is not of type Bgr8.");
                }
            }
        }

        // Print our results
        print!(
            "\rDistance of center pixel: {:<10} m. Field depth {} x {} RGB {} x {}, pixel RGB: {:<3} {:<3} {:<3}",
            distance, dw, dh, rw, rh, rr, gg, bb
        );

        io::stdout().flush().unwrap();
    }

    Ok(())
}
