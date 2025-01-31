use crate::hsv::DefinedColor;
use bevy::render::render_resource::encase::private::RuntimeSizedArray;
use bevy::utils::Instant;
use image::{DynamicImage, GenericImageView};
use opencv::{core, highgui, imgproc, prelude::*};
use rayon::prelude::*;
use std::collections::HashMap;

const MIN_RADIUS: usize = 20;
const MAX_RADIUS: usize = 60;

const IS_MATCHING: u16 = 32000;
const IS_MATHING_MIN_THR: u16 = 16000; // 200;

#[derive(Debug, Clone, Eq, PartialEq, Hash, Copy)]
/// Represents a detected circle in an image.
pub struct Detection {
    pub x: usize, // X coordinate of the circle's center
    pub y: usize, // Y coordinate of the circle's center
    pub r: usize, // Radius of the detected circle
}

/// Applies a simple Gaussian blur to the image mask.
fn gaussian_blur(mask: &Vec<Vec<u16>>, width: usize, height: usize) -> Vec<Vec<u16>> {
    let kernel: [[f64; 5]; 5] = [
        [1.0, 4.0, 7.0, 4.0, 1.0],
        [4.0, 16.0, 26.0, 16.0, 4.0],
        [7.0, 26.0, 41.0, 26.0, 7.0],
        [4.0, 16.0, 26.0, 16.0, 4.0],
        [1.0, 4.0, 7.0, 4.0, 1.0],
    ];

    let kernel_sum: f64 = kernel.iter().flatten().sum();

    let mut blurred = vec![vec![0u16; width]; height];

    for y in 2..height - 2 {
        for x in 2..width - 2 {
            let mut sum = 0.0;
            for ky in 0..5 {
                for kx in 0..5 {
                    sum += kernel[ky][kx] * mask[y + ky - 2][x + kx - 2] as f64;
                }
            }
            blurred[y][x] = (sum / kernel_sum) as u16;
        }
    }

    blurred
}

fn gradient_based_radius(
    mask: &Vec<Vec<u16>>,   // Input mask, typically a grayscale image
    center: (usize, usize), // Known circle center (cx, cy)
    width: usize,           // Width of the mask
    height: usize,          // Height of the mask
) -> usize {
    fn estimate_gradient_threshold(mask: &Vec<Vec<u16>>, width: usize, height: usize) -> f64 {
        let mut gradients: Vec<f64> = Vec::new();

        // Compute gradients for the mask
        for y in 1..(height - 1) {
            for x in 1..(width - 1) {
                let dx = (mask[y][x + 1] as i32 - mask[y][x - 1] as i32) as f64;
                let dy = (mask[y + 1][x] as i32 - mask[y - 1][x] as i32) as f64;

                // Compute gradient magnitude
                let grad_magnitude = (dx * dx + dy * dy).sqrt();
                gradients.push(grad_magnitude);
            }
        }

        // Calculate mean and standard deviation
        let mean: f64 = gradients.iter().sum::<f64>() / gradients.len() as f64;
        let variance: f64 =
            gradients.iter().map(|g| (g - mean).powi(2)).sum::<f64>() / gradients.len() as f64;
        let std_dev = variance.sqrt();

        // Set threshold as mean + standard deviation multiplier (e.g., 1.5)
        let threshold = mean + 1.5 * std_dev;

        threshold
    }

    let (cx, cy) = center;
    let directions = 360; // Number of radial directions to sample (angles in degrees)
    let mut radii: Vec<usize> = Vec::with_capacity(directions); // To store radii where edges are detected
    let threshold = estimate_gradient_threshold(mask, width, height);

    // Loop through radial directions
    for angle in 0..directions {
        let (sin_theta, cos_theta) = (angle as f64).to_radians().sin_cos();

        // Track gradient magnitudes along this ray
        let mut previous_intensity: i32 = mask[cy][cx] as i32;
        for r in MIN_RADIUS..=MAX_RADIUS {
            // Compute the coordinates for this radius and direction
            let x = cx as isize + (cos_theta * r as f64) as isize;
            let y = cy as isize + (sin_theta * r as f64) as isize;

            // Check if x, y are within bounds
            if x < 0 || x >= width as isize || y < 0 || y >= height as isize {
                break;
            }

            let current_intensity = mask[y as usize][x as usize] as i32;
            let gradient_magnitude = (current_intensity - previous_intensity).abs() as f64;

            // Detect an edge: Look for a substantial gradient change
            if gradient_magnitude > threshold {
                // Threshold for edge detection
                radii.push(r);
                break; // Stop once an edge is detected for this direction
            }

            // Update previous intensity for the next step
            previous_intensity = current_intensity;
        }
    }

    // Return the median radius since it's robust to outliers
    if !radii.is_empty() {
        radii.sort(); // Sort radii
        radii[radii.len() / 2] // Median radius
    } else {
        0 // Return 0 if no edges are detected
    }
}

fn process_mask(mask: &Vec<Vec<u16>>) -> HashMap<(i16, i16), u16> {
    fn local_y(y_main:  isize, y_global:  isize) -> usize {
        (y_global - y_main + MAX_RADIUS as isize) as usize
    }

    fn global_y(y_main:  isize, y_local:  isize) -> usize {
        (y_local + y_main  - MAX_RADIUS as isize) as usize
    }    
    
    let (height, width) = (mask.len() as isize, mask[0].len() as isize);
    // Create thread-local accumulators to avoid contention
    let votes: HashMap<(i16, i16), u16> = (0..height)
        .into_par_iter() // Parallelize the outer loop
        .map(|y| {
            let y_float = y as f32;
            // Create a local accumulator for each row (width is a vector, for y it can be array)
            let mut local = vec![[0u16; 2 * MAX_RADIUS + 1]; width as usize];

            for x in 0..width as usize {
                if mask[y as usize][x] > IS_MATHING_MIN_THR {
                    // Highly likely part of a circle
                    let x_float = x as f32;
                    for angle in (0..360).step_by(2) {
                        let (sin_theta, cos_theta) = (angle as f32).to_radians().sin_cos();
                        for r_int in (MIN_RADIUS..=MAX_RADIUS).step_by(1) {
                            let r = r_int as f32;
                            let circle_x = (x_float + r * sin_theta) as isize;
                            let circle_y = (y_float + r * cos_theta) as isize;
                            if (0..width).contains(&circle_x) && (0..height).contains(&circle_y) {
                                local[circle_x as usize][local_y(y, circle_y)] += 1; // width first
                            }
                        }
                    }
                }
            }
            let mut local_map = HashMap::new();
            for xx in 0..width as usize {
                for yl in 0..local[xx].len() as isize
                {
                    if local[xx][yl as usize] > 0 { // width first
                        let yy = global_y(y, yl);
                        *local_map.entry((xx as i16, yy as i16)).or_insert(0) += 1;
                    }
                }
            }
            local_map
        })
        .reduce(
            || HashMap::with_capacity(256),
            |mut global_map, local_map| {
                for (key, value) in local_map {
                    *global_map.entry(key).or_insert(0) += value; // Merge local into global
                }
                global_map
            },
        );
    votes
}

fn hough_circle_detection(mask: &Vec<Vec<u16>>) -> Result<Detection, String> {
    let (height, width) = (mask.len(), mask[0].len());
    let accumulator = process_mask(mask);

    if accumulator.is_empty() {
        return Err("No circle center detected".to_string());
    }

    let now = Instant::now();

    // Assume 0.0 as the best center for start
    let mut best = accumulator.iter().next().unwrap();

    // Find best center
    for point in &accumulator {
        if point.1 > best.1 {
            best = point;
        }
    }

    // Probe for the best radius around the detected center
    let best_center = (best.0 .0 as usize, best.0 .1 as usize);
    let (best_x, best_y) = best_center;
    let now = Instant::now();
    let best_radius = gradient_based_radius(mask, best_center, width, height);

    if best_radius <= 1 {
        return Err("No valid circle diameter detected".to_string());
    }

    // +1 adjustment introduced comparing with the reference image file.
    Ok(Detection {
        x: best_x + 1,
        y: best_y + 1,
        r: best_radius + 1,
    })
}

pub fn detect_circle(img: &DynamicImage, color: &DefinedColor) -> Result<Detection, String> {
    let mut mask = detect_pixels(img, color);
    let detection = hough_circle_detection(&mask)?;
    detect_circle_iter2(&mut mask, &detection)
}

pub fn detect_circle_iter2(
    mask: &mut Vec<Vec<u16>>,
    detection: &Detection,
) -> Result<Detection, String> {
    let height = mask.len() as i32;
    let width = mask[0].len() as i32;
    for y in 0..height {
        for x in 0..width {
            let cx = x - detection.x as i32;
            let cy = y - detection.y as i32;
            if cx * cx + cy * cy >= (2 * MAX_RADIUS * MAX_RADIUS) as i32 {
                if x >= 0 && y >= 0 && x < width && y < height {
                    mask[y as usize][x as usize] = 0; // Reset to none pixels far from the expected center
                }
            }
        }
    }

    hough_circle_detection(mask)
}

pub fn detect_circle_mat(img: &Mat, color: &DefinedColor) -> Result<Detection, String> {
    let np = Instant::now();
    let mut mask = detect_pixels_mat(img, color);
    let detection = hough_circle_detection(&mask)?;
    let result = detect_circle_iter2(&mut mask, &detection);
    result
}

fn detect_pixels(img: &DynamicImage, color: &DefinedColor) -> Vec<Vec<u16>> {
    let (width, height) = img.dimensions();

    // Divide the image pixels into chunks of 256 for parallel processing
    let binding = img.pixels().collect::<Vec<_>>();
    let pixel_chunks: Vec<_> = binding.chunks(256).collect();

    // Process each chunk in parallel
    let high_score_coords: Vec<(u32, u32)> = pixel_chunks
        .par_iter()
        .flat_map(|chunk| {
            let mut coords = Vec::new();

            for (x, y, pixel) in *chunk {
                if color.this_color(pixel) {
                    if coords.is_empty() {
                        coords.reserve(256);
                    }
                    coords.push((*x, *y));
                }
            }

            coords
        })
        .collect();

    // Create a binary mask from the collected coordinates
    let mut mask = vec![vec![0u16; width as usize]; height as usize];
    for (x, y) in high_score_coords {
        mask[y as usize][x as usize] = IS_MATCHING;
    }
    mask
}

fn detect_pixels_mat(mat: &core::Mat, color: &DefinedColor) -> Vec<Vec<u16>> {
    let width = mat.cols();
    let height = mat.rows();

    // Ensure the matrix is continuous
    assert!(
        mat.is_continuous(),
        "Mat must be continuous for processing."
    );
    let data = mat.data_bytes().unwrap();
    let mut coords = Vec::new();

    for y in 0..height {
        for x in 0..width {
            let pixel = mat.at_2d::<core::Vec3b>(y, x).expect("Failed to get pixel");
            if color.this_color_rgb(pixel[2], pixel[1], pixel[0]) {
                // BGR
                coords.push((x, y));
            }
        }
    }
    // Create a binary mask from the collected coordinates
    let mut mask = vec![vec![0u16; width as usize]; height as usize];
    for (x, y) in &coords {
        mask[*y as usize][*x as usize] = IS_MATCHING;
    }
    mask
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detection::detect_circles;

    #[test]
    fn test_detect_all() -> Result<(), String> {
        let image_path = "/home/audrius/opw/calibration/c2_Color.png";

        let img = image::io::Reader::open(image_path)
            .map_err(|e| format!("Failed to open image: {}", e))?
            .decode()
            .map_err(|e| format!("Failed to decode image: {}", e))?;
        let detections = detect_circles(&img);
        for (color, detection) in detections {
            println!("{:?}: {:?}", color, detection);
        }
        Ok(())
    }

    // #[test]
    fn test_detect_red_circle() -> Result<(), String> {
        let image_path = "src/tests/data/vision/rg_e.png";

        let img = image::io::Reader::open(image_path)
            .map_err(|e| format!("Failed to open image: {}", e))?
            .decode()
            .map_err(|e| format!("Failed to decode image: {}", e))?;

        match detect_circle(&img, &DefinedColor::red()) {
            Ok(detection) => {
                let (x, y, radius) = (detection.x, detection.y, detection.r);
                println!("Detected red circle at: ({}, {}), radius: {}", x, y, radius,);
                assert_eq!(x, 332);
                assert_eq!(y, 432);
                assert_eq!(radius, 32);
            }
            Err(e) => {
                eprintln!("Error occurred: {}", e);
                assert!(false, "Error during detection: {}", e);
            }
        };
        Ok(())
    }

    // #[test]
    fn test_detect_green_circle() -> Result<(), String> {
        let image_path = "src/tests/data/vision/rg_e.png";

        let img = image::io::Reader::open(image_path)
            .map_err(|e| format!("Failed to open image: {}", e))?
            .decode()
            .map_err(|e| format!("Failed to decode image: {}", e))?;

        match detect_circle(&img, &DefinedColor::green()) {
            Ok(detection) => {
                let (x, y, radius) = (detection.x, detection.y, detection.r);
                println!("Detected red circle at: ({}, {}), radius: {}", x, y, radius,);
                assert_eq!(x, 534);
                assert_eq!(y, 434);
                assert_eq!(radius, 32);
            }
            Err(e) => {
                eprintln!("Error occurred: {}", e);
                assert!(false, "Error during detection: {}", e);
            }
        };
        Ok(())
    }

    // #[test]
    fn test_detect_blue_circle() -> Result<(), String> {
        let image_path = "src/tests/data/vision/rg_e.png";

        let img = image::io::Reader::open(image_path)
            .map_err(|e| format!("Failed to open image: {}", e))?
            .decode()
            .map_err(|e| format!("Failed to decode image: {}", e))?;

        match detect_circle(&img, &DefinedColor::blue()) {
            Ok(detection) => {
                let (x, y, radius) = (detection.x, detection.y, detection.r);
                println!("Detected red circle at: ({}, {}), radius: {}", x, y, radius,);
                assert_eq!(x, 536);
                assert_eq!(y, 202);
                assert_eq!(radius, 32);
            }
            Err(e) => {
                eprintln!("Error occurred: {}", e);
                assert!(false, "Error during detection: {}", e);
            }
        };
        Ok(())
    }

    // #[test]
    fn test_detect_yellow_circle() -> Result<(), String> {
        let image_path = "src/tests/data/vision/rg_e.png";

        let img = image::io::Reader::open(image_path)
            .map_err(|e| format!("Failed to open image: {}", e))?
            .decode()
            .map_err(|e| format!("Failed to decode image: {}", e))?;

        match detect_circle(&img, &DefinedColor::yellow()) {
            Ok(detection) => {
                let (x, y, radius) = (detection.x, detection.y, detection.r);
                println!("Detected red circle at: ({}, {}), radius: {}", x, y, radius,);
                assert_eq!(x, 332);
                assert_eq!(y, 202);
                assert_eq!(radius, 32);
            }
            Err(e) => {
                eprintln!("Error occurred: {}", e);
                assert!(false, "Error during detection: {}", e);
            }
        };
        Ok(())
    }
}
