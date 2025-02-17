use std::collections::HashMap;
use crate::organized_point::OrganizedPoint;

/// Finds the largest rectangle inside the structured grid and returns its points.
pub fn largest_rectangle(points: &[OrganizedPoint]) -> Vec<OrganizedPoint> {
    let mut grid: HashMap<(usize, usize), OrganizedPoint> = HashMap::with_capacity(points.len());
    let mut max_row = 0;
    let mut max_col = 0;

    // Fill the grid map and track the largest row/col indices
    for p in points {
        grid.insert((p.row, p.col), *p);
        max_row = max_row.max(p.row);
        max_col = max_col.max(p.col);
    }

    let mut binary_grid = vec![vec![0; max_col + 1]; max_row + 1];

    // Convert points into a 2D binary grid
    for p in points {
        binary_grid[p.row][p.col] = 1;
    }

    // Use DP to compute the largest rectangle
    let ((row_start, col_start), (row_end, col_end)) = maximal_rectangle(&binary_grid);

    // Extract the points belonging to the largest rectangle
    let rect_width = col_end - col_start + 1;
    let rect_height = row_end - row_start + 1;
    let mut largest_rect_points = Vec::with_capacity(rect_width * rect_height);

    for row in row_start..=row_end {
        for col in col_start..=col_end {
            if let Some(&point) = grid.get(&(row, col)) {
                largest_rect_points.push(point);
            }
        }
    }

    largest_rect_points
}

/// Finds the largest rectangle using a dynamic programming approach
fn maximal_rectangle(binary_grid: &[Vec<i32>]) -> ((usize, usize), (usize, usize)) {
    let rows = binary_grid.len();
    let cols = if rows == 0 { 0 } else { binary_grid[0].len() };

    let mut height = vec![0; cols];
    let mut left = vec![0; cols];
    let mut right = vec![cols; cols];
    let mut max_area = 0;
    let mut best_rect = ((0, 0), (0, 0));

    for r in 0..rows {
        let mut curr_left = 0;
        let mut curr_right = cols;

        // Compute height
        for c in 0..cols {
            if binary_grid[r][c] == 1 {
                height[c] += 1;
            } else {
                height[c] = 0;
            }
        }

        // Compute left boundaries
        for c in 0..cols {
            if binary_grid[r][c] == 1 {
                left[c] = left[c].max(curr_left);
            } else {
                left[c] = 0;
                curr_left = c + 1;
            }
        }

        // Compute right boundaries (from right to left)
        for c in (0..cols).rev() {
            if binary_grid[r][c] == 1 {
                right[c] = right[c].min(curr_right);
            } else {
                right[c] = cols;
                curr_right = c;
            }
        }

        // Compute max area
        for c in 0..cols {
            let area = height[c] * (right[c] - left[c]);
            if area > max_area {
                max_area = area;
                best_rect = ((r - height[c] + 1, left[c]), (r, right[c] - 1));
            }
        }
    }

    best_rect
}
