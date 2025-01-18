use crate::hsv::DefinedColor;
use image::io::Reader as ImageReader;
use image::{DynamicImage, GenericImageView};
use rayon::prelude::*;

const MIN_RADIUS: usize = 10;
const MAX_RADIUS: usize = 250;

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

fn process_mask(mask: &Vec<Vec<u16>>) -> Vec<Vec<usize>> {
    let (height, width) = (mask.len(), mask[0].len());

    // Create thread-local accumulators to avoid contention
    let thread_local_accumulators: Vec<Vec<Vec<usize>>> = (0..height)
        .into_par_iter() // Parallelize the outer loop
        .filter_map(|y| {
            // Create a local accumulator for each row
            let mut local_accumulator = None;

            for x in 0..width {
                if mask[y][x] > IS_MATHING_MIN_THR {
                    // Highly likely part of a circle
                    for angle in 0..360 {
                        let (sin_theta, cos_theta) = (angle as f64).to_radians().sin_cos();
                        for r in MIN_RADIUS..=MAX_RADIUS {
                            let ur = r as f64;
                            let a = (x as f64 - ur * sin_theta) as isize;
                            let b = (y as f64 - ur * cos_theta) as isize;
                            if a >= 0 && a < width as isize && b >= 0 && b < height as isize {
                                if local_accumulator.is_none() {
                                    local_accumulator = Some(vec![vec![0usize; width]; height])
                                }
                                if let Some(local_accumulator) = local_accumulator.as_mut() {
                                    local_accumulator[b as usize][a as usize] += 1;
                                }
                            }
                        }
                    }
                }
            }

            local_accumulator
        })
        .collect();

    // Combine thread-local accumulators into the shared accumulator
    let mut accumulator: Vec<Vec<usize>> = Vec::new();
    accumulator.resize(height, vec![0usize; width]);
    for local_accumulator in thread_local_accumulators {
        for y in 0..height {
            for x in 0..width {
                accumulator[y][x] += local_accumulator[y][x];
            }
        }
    }
    accumulator
}

fn hough_circle_detection(mask: &Vec<Vec<u16>>) -> Result<Detection, String> {
    let (height, width) = (mask.len(), mask[0].len());
    let accumulator = process_mask(mask);

    // Assume 0.0 as the best center for start
    let mut max_votes = accumulator[0][0];
    let mut best_center = (0, 0);

    // Find best center
    for y in 0..height {
        for x in 0..width {
            if accumulator[y][x] > 0 && accumulator[y][x] > max_votes {
                max_votes = accumulator[y][x];
                best_center = (x, y);
            }
        }
    }

    if max_votes <= 1 {
        return Err("No circle center detected".to_string());
    }

    // Probe for the best radius around the detected center
    let (best_x, best_y) = best_center;

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
    let (width, height) = img.dimensions();
    let mask = detect_pixels(&img, color);

    // Apply Gaussian blur to the binary mask (we will check if it is better or not with it)
    let blurred_mask = gaussian_blur(&mask, width as usize, height as usize);

    // Detect circles in the blurred mask
    hough_circle_detection(&blurred_mask)
    //Ok(hough_circle_detection(&mask ))
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_red_circle() -> Result<(), String> {
        let image_path = "src/tests/data/vision/rg_e.png";

        let img = ImageReader::open(image_path)
            .map_err(|e| format!("Failed to open image: {}", e))?
            .decode()
            .map_err(|e| format!("Failed to decode image: {}", e))?;

        match detect_circle(&img, &DefinedColor::red()) {
            Ok(detection) => {
                let (x, y, radius) = (detection.x, detection.y, detection.r);
                println!(
                    "Detected red circle at: ({}, {}), radius: {}",
                    x, y, radius,
                );
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

    #[test]
    fn test_detect_green_circle() -> Result<(), String> {
        let image_path = "src/tests/data/vision/rg_e.png";

        let img = ImageReader::open(image_path)
            .map_err(|e| format!("Failed to open image: {}", e))?
            .decode()
            .map_err(|e| format!("Failed to decode image: {}", e))?;

        match detect_circle(&img, &DefinedColor::green()) {
            Ok(detection) => {
                let (x, y, radius) = (detection.x, detection.y, detection.r);
                println!(
                    "Detected red circle at: ({}, {}), radius: {}",
                    x, y, radius,
                );
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

    #[test]
    fn test_detect_blue_circle() -> Result<(), String> {
        let image_path = "src/tests/data/vision/rg_e.png";

        let img = ImageReader::open(image_path)
            .map_err(|e| format!("Failed to open image: {}", e))?
            .decode()
            .map_err(|e| format!("Failed to decode image: {}", e))?;

        match detect_circle(&img, &DefinedColor::blue()) {
            Ok(detection) => {
                let (x, y, radius) = (detection.x, detection.y, detection.r);
                println!(
                    "Detected red circle at: ({}, {}), radius: {}",
                    x, y, radius,
                );
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

    #[test]
    fn test_detect_yellow_circle() -> Result<(), String> {
        let image_path = "src/tests/data/vision/rg_e.png";

        let img = ImageReader::open(image_path)
            .map_err(|e| format!("Failed to open image: {}", e))?
            .decode()
            .map_err(|e| format!("Failed to decode image: {}", e))?;

        match detect_circle(&img, &DefinedColor::yellow()) {
            Ok(detection) => {
                let (x, y, radius) = (detection.x, detection.y, detection.r);
                println!(
                    "Detected red circle at: ({}, {}), radius: {}",
                    x, y, radius,
                );
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
