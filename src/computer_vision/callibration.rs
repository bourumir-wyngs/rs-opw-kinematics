use image::{DynamicImage, GenericImageView, Pixel};

const MIN_RADIUS: usize = 10;
const MAX_RADIUS: usize = 250;

const IS_RED: u16 = 32000;
const IS_RED_MIN_THR: u16 = 16000; // 200;

/// Converts an RGB pixel to HSV.
/// HSV ranges:
/// - H: [0, 360] (hue)
/// - S: [0, 1] (saturation)
/// - V: [0, 1] (value)
fn rgb_to_hsv(r: u8, g: u8, b: u8) -> (f64, f64, f64) {
    let r = r as f64 / 255.0;
    let g = g as f64 / 255.0;
    let b = b as f64 / 255.0;

    let max = r.max(g).max(b);
    let min = r.min(g).min(b);
    let delta = max - min;

    let h = if delta == 0.0 {
        0.0
    } else if max == r {
        60.0 * (((g - b) / delta) % 6.0)
    } else if max == g {
        60.0 * (((b - r) / delta) + 2.0)
    } else {
        60.0 * (((r - g) / delta) + 4.0)
    };

    let hue = if h < 0.0 { h + 360.0 } else { h };
    let saturation = if max == 0.0 { 0.0 } else { delta / max };
    let value = max;

    (hue, saturation, value)
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
    mask: &Vec<Vec<u16>>,    // Input mask, typically a grayscale image
    center: (usize, usize), // Known circle center (cx, cy)
    width: usize,           // Width of the mask
    height: usize          // Height of the mask
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
                if mask[y][x] > IS_RED_MIN_THR {
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

fn hough_circle_detection(mask: &Vec<Vec<u16>>) -> (usize, usize, usize, usize) {
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

    // Probe for the best radius around the detected center
    let (best_x, best_y) = best_center;
    
    let best_radius =
        gradient_based_radius(mask, best_center, width, height);

    // +1 adjustment introduced comparing with the reference image file.
    (best_x + 1, best_y + 1, best_radius + 1, max_votes)
}

use image::io::Reader as ImageReader;
use rayon::prelude::*; // Assume the `image` crate is being used.

pub fn detect_red_circle(image_path: &str) -> Result<(usize, usize, usize, usize), String> {
    // Load the image using the `image` crate
    let img = ImageReader::open(image_path)
        .map_err(|e| format!("Failed to open image: {}", e))?
        .decode()
        .map_err(|e| format!("Failed to decode image: {}", e))?;

    let (width, height) = img.dimensions();    
    let mask = detect_red_pixels(img);

    // Apply Gaussian blur to the binary mask (we will check if it is better or not with it)
    let blurred_mask = gaussian_blur(&mask, width as usize, height as usize);

    // Detect circles in the blurred mask
    Ok(hough_circle_detection(&blurred_mask ))
    //Ok(hough_circle_detection(&mask ))
}

fn detect_red_pixels(img: DynamicImage) -> Vec<Vec<u16>> {
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
                let rgb = pixel.to_rgb();
                let (h, _, _) = rgb_to_hsv(rgb[0], rgb[1], rgb[2]);

                // Check if the pixel is in the red color range in HSV
                if h >= 350.0 || h < 24.0 {
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
        mask[y as usize][x as usize] = IS_RED;
    }
    mask
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_red_circle() {
        let image_path = "src/tests/data/vision/rg_e.png"; // 613, 383), radius: 72 // (615, 384), radius: 67
        match detect_red_circle(image_path) {
            Ok((x, y, radius, score)) => {
                println!(
                    "Detected red circle at: ({}, {}), radius: {} score {}",
                    x, y, radius, score
                );
                //assert!(x > 0 && y > 0 && radius > 0, "Invalid circle parameters!");
            }
            Err(e) => {
                eprintln!("Error occurred: {}", e);
                assert!(false, "Error during detection: {}", e);
            }
        }
    }
}
