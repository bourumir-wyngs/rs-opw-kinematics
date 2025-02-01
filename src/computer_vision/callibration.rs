use crate::hsv::{ColorId, DefinedColor};
use bevy::render::render_resource::encase::private::RuntimeSizedArray;
use opencv::{core, prelude::*};
use rayon::prelude::*;
use std::collections::HashMap;

const MIN_RADIUS: usize = 5;
const MAX_RADIUS: usize = 60;

pub const BASIC_MULTIPLIER: usize = 100;
pub const MIN_BRIGHTNESS: usize = 80;
pub const IS_MATHING_MIN_THR: u16 = (BASIC_MULTIPLIER as u16 * 300) /(100+300+100); // 200;

#[derive(Debug, Clone, PartialEq, Copy)]
/// Represents a detected circle in an image.
pub struct Detection {
    pub color: ColorId,
    pub x: usize, // X coordinate of the circle's center
    pub y: usize, // Y coordinate of the circle's center
    pub r: usize, // Radius of the detected circle
    pub depth: usize // Depth in "depth pixels" that are adjusted to be the same as for x, y and r.
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
                        *local_map.entry((xx as i16, yy as i16)).or_insert(0) += local[xx][yl as usize];
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

fn hough_circle_detection(mask: &Vec<Vec<u16>>, color: ColorId) -> Result<Detection, String> {
    let (height, width) = (mask.len(), mask[0].len());
    let accumulator = process_mask(mask);

    if accumulator.is_empty() {
        return Err("No circle center detected".to_string());
    }

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
    let best_radius = gradient_based_radius(mask, best_center, width, height);

    if best_radius <= 1 {
        return Err("No valid circle diameter detected".to_string());
    }

    // +1 adjustment introduced comparing with the reference image file.
    Ok(Detection {
        color: color,
        x: best_x + 1,
        y: best_y + 1,
        r: best_radius + 1,
        depth: 0 // to be populated later from depth frame
    })
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

    hough_circle_detection(mask, detection.color)
}

pub fn detect_circle_mat(img: &Mat, color: &DefinedColor) -> Result<Detection, String> {
    let mut mask = detect_pixels_mat(img, color);
    let detection = hough_circle_detection(&mask, color.id())?;
    let result = detect_circle_iter2(&mut mask, &detection);
    result
}

fn detect_pixels_mat(mat: &core::Mat, color: &DefinedColor) -> Vec<Vec<u16>> {
    let width = mat.cols();
    let height = mat.rows();

    // Ensure the matrix is continuous
    assert!(
        mat.is_continuous(),
        "Mat must be continuous for processing."
    );
    let mut mask = vec![vec![0u16; width as usize]; height as usize];

    for y in 0..height {
        for x in 0..width {
            let pixel = mat.at_2d::<core::Vec3b>(y, x).expect("Failed to get pixel");
            mask[y as usize][x as usize] = color.color_score(pixel[2], pixel[1], pixel[0]);
        }
    }
    //print_mask_to_ascii(&mask, "mask.txt").expect("Failed to print mask to ASCII");
    mask
}


use std::fs::File;
use std::io::{self, Write};

fn print_mask_to_ascii(mask: &Vec<Vec<u16>>, output_file: &str) -> io::Result<()> {
    // Step 1: Find the min and max values in the mask
    let mut min_value = u16::MAX;
    let mut max_value = u16::MIN;

    for row in mask.iter() {
        for &value in row.iter() {
            if value < min_value {
                min_value = value;
            }
            if value > max_value {
                max_value = value;
            }
        }
    }

    // Step 2: Calculate the mapping range
    let ascii_chars: Vec<char> = ('a'..='z').collect();
    let num_chars = ascii_chars.len() as u16;

    let map_to_ascii = |value: u16| -> char {
        if max_value == min_value {
            // Handle edge case where all mask values are the same
            ascii_chars[0]
        } else {
            let normalized = ((value - min_value) as f64 / (max_value - min_value) as f64)
                * (num_chars as f64 - 1.0);
            ascii_chars[normalized as usize]
        }
    };

    // Step 3: Write ASCII art to a file
    let mut file = File::create(output_file)?;

    for row in mask.iter() {
        for &value in row.iter() {
            let v = map_to_ascii(value);
            if v >= 'o' {
                write!(file, "{}", v)?;
            } else {
                write!(file, "{}", ' ')?;
            }
        }
        writeln!(file)?; // Newline after each row
    }

    // Step 4: Write the min and max values at the end
    writeln!(file, "\nMin Value: {}", min_value)?;
    writeln!(file, "Max Value: {}", max_value)?;

    Ok(())
}
