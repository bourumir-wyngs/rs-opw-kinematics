use rayon::prelude::*;

fn find_best_fit_sphere(
    depth_map: &Vec<Vec<i32>>,
    guess_x: i32,
    guess_y: i32,
    guess_r: i32,
) -> (i32, i32, i32, i32) {
    let guess_z = depth_map[guess_y as usize][guess_x as usize] + guess_r;
    let delta = 6;

    // Create a Vec of all combinations of (x, y, z, r) in the range
    let search_space = (guess_x - delta..=guess_x + delta)
        .flat_map(|x| {
            (guess_y - delta..=guess_y + delta).flat_map(move |y| {
                (guess_z - delta..=guess_z + delta).flat_map(move |z| {
                    (guess_r - delta..=guess_r + delta).map(move |r| (x, y, z, r))
                })
            })
        })
        .collect::<Vec<_>>();

    // Use rayon to parallelize the search over (x, y, z, r)
    let (best_x, best_y, best_z, best_r, best_error) = search_space
        .par_iter()
        .map(|&(x, y, z, r)| {
            let error = calculate_error(depth_map, x, y, z, r);
            (x, y, z, r, error)
        })
        .min_by_key(|&(_, _, _, _, error)| error)
        .unwrap();

    (best_x, best_y, best_z, best_r)
}


fn calculate_error(depth_map: &Vec<Vec<i32>>, x: i32, y: i32, z: i32, r: i32) -> i32 {
    let mut total_error = 0;

    for dx in -r..=r {
        for dy in -r..=r {
            if dx * dx + dy * dy > r * r {
                continue;
            }
            let sx = x + dx;
            let sy = y + dy;
            let actual_z = depth_map[sy as usize][sx as usize];
            // Skip pixels with invalid or zero depth
            if actual_z == 0 {
                continue;
            }

            // sx, sy and sz now represent the center of the sphere of radius r
            // Predicted z-coordinate based on the sphere equation
            let predicted_z =
                z - ((r.pow(2) - (sx - x).pow(2) - (sy - y).pow(2)) as f64).sqrt() as i32;

            // Compute the residual using the sphere equation
            let residual = (actual_z - predicted_z).abs();

            // Compute absolute error, summing all residuals
            total_error += residual;
        }
    }

    total_error
}

#[cfg(test)]
mod tests {
    use super::*; // Import the function find_best_fit_sphere

    #[test]
    fn test_find_best_fit_sphere() {
        // Step 1: Set up the depth map (50x50) and populate with a synthetic sphere
        let width = 200;
        let height = 200;
        let mut depth_map = vec![vec![0; width]; height]; // All depths initialized to zero

        // Known sphere parameters
        let true_x = 100 as i32;
        let true_y = 100 as i32;
        let true_r = 30 as i32;
        let true_z = 100 as i32;

        // Populate the depth map for a synthetic sphere
        for dx in -true_r..=true_r {
            for dy in -true_r..=true_r {
                let squared_sum = dx * dx + dy * dy;
                if squared_sum <= true_r * true_r {
                    depth_map[(true_y + dy) as usize][(true_x + dx) as usize] =
                        true_z - ((true_r * true_r - squared_sum) as f64).sqrt() as i32;
                }
            }
        }

        // Step 2: Define the initial guesses for the sphere parameters
        let guess_x = 104; // Slightly offset x-center
        let guess_y = 104; // Slightly offset y-center

        // Step 3: Call the fitting function to find the best-fit sphere
        let (fitted_x, fitted_y, fitted_z, fitted_r) =
            find_best_fit_sphere(&depth_map, guess_x, guess_y, true_r);
        println!(
            "Best-fit sphere: x={}, y={}, z={}, r={}",
            fitted_x, fitted_y, fitted_z, fitted_r
        );
    }
}
