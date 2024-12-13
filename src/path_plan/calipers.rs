// Helper functions to work with the mesh (computes rectangle of the largest area and size)
use geo::{Contains, Point, Polygon, Simplify};
use std::f32;

fn generate_interpolated_points(
    corners: Vec<Point<f32>>
) -> Vec<Point<f32>> {
    let mut result = Vec::with_capacity(corners.len() * 4);

    for i in 0..corners.len() {
        // Original point with delta applied
        let current = corners[i];
        let next = corners[(i + 1) % corners.len()]; // Wrap around to form a loop

        // Add the current corner to the result
        result.push(Point::new(
            current.x(),
            current.y(),
        ));

        // Add 3 intermediate points
        for step in 1..=3 {
            let t = step as f32 / 4.0; // 1/4, 2/4, 3/4 for 3 intermediate points
            let interpolated_x = current.x() + t * (next.x() - current.x());
            let interpolated_y = current.y() + t * (next.y() - current.y());
            result.push(Point::new(interpolated_x, interpolated_y));
        }
    }

    result
}

pub fn largest_fitting_rectangle(polygon: &Polygon<f32>) -> Option<(Point<f32>, Point<f32>)> {
    let delta = 1E-5;
    let tolerance = 0.0002; // Only points more apart than 2 mm are taken into consideration
    let n_was = polygon.exterior().points().len();
    let polygon_shape = if n_was < 200 {
        polygon.clone()
    } else {
        polygon.simplify(&tolerance)
    };
   
    let points: Vec<Point<f32>> = polygon_shape.exterior().points().collect();
    let n = points.len();
    if n != n_was {
        println!("Simplified from {} to {} points", n_was, n);
    }
    if n < 3 {
        return None; // Not enough points for a valid polygon
    }    

    let mut min_area = 0.0; // any non zero area rectangle will replace
    let mut best_rectangle = None;

    // Iterate over all pairs of points as potential opposite corners
    for i in 0..n {
        for j in i + 1..n {
            let p1 = &points[i];
            let p2 = &points[j];

            // Compute the bounding box defined by these two points
            let min_x = &p1.x().min(p2.x());
            let max_x = &p1.x().max(p2.x());
            let min_y = &p1.y().min(p2.y());
            let max_y = &p1.y().max(p2.y());

            let width = max_x - min_x;
            let height = max_y - min_y;
            
            // area is much cheaper to compute than to check if all points are in polygon
            let area = width * height;
            if area > min_area && width > height {
                let rectangle_corners = vec![
                    Point::new(min_x + delta, min_y + delta),
                    Point::new(min_x + delta, max_y - delta),
                    Point::new(max_x - delta, min_y + delta),
                    Point::new(max_x - delta, max_y - delta),
                ];
                
                let outline = generate_interpolated_points(rectangle_corners);
                // Check if all rectangle corners are inside the polygon
                if outline
                    .iter()
                    .all(|&corner| polygon_shape.contains(&corner))
                {
                    min_area = area;
                    best_rectangle = Some((Point::new(*min_x, *min_y), Point::new(*max_x, *max_y)));
                }
            }
        }
    }

    // Return the smallest valid bounding box with its area
    best_rectangle
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::{LineString, Point};

    #[test]
    fn test_largest_fitting_rectangle_square() {
        let square = LineString::from(vec![(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]);
        let result = largest_fitting_rectangle(&to_polygon(&square)).expect("Failed to compute rectangle");

        // Just the same rectangle
        let expected_bottom_left = Point::new(0.0, 0.0);
        let expected_top_right = Point::new(4.0, 4.0);
        assert_eq!(
            result.0, expected_bottom_left,
            "Bottom-left corner mismatch"
        );
        assert_eq!(result.1, expected_top_right, "Top-right corner mismatch");
    }

    #[test]
    fn test_largest_rectangle_axis_aligned_right_triangle() {
        // Define the right triangle
        let triangle = LineString::from(vec![
            (0.0, 0.0),
            (1.0, 0.0),
            (0.5, 0.5), // Hypotenuse midpoint
            (0.0, 1.0),
        ]);

        // Compute the largest rectangle
        let result = largest_fitting_rectangle(&to_polygon(&triangle)).expect("Failed to compute rectangle");

        let expected_bottom_left = Point::new(0.0, 0.0);
        let expected_top_right = Point::new(0.5, 0.5);
        assert_eq!(
            result.0, expected_bottom_left,
            "Bottom-left corner mismatch"
        );
        assert_eq!(result.1, expected_top_right, "Top-right corner mismatch");
    }

    #[test]
    fn test_largest_rectangle_with_horizontal_hypotenuse() {
        // Define the triangle with the hypotenuse aligned to the X-axis
        let triangle_with_horizontal_hypotenuse = LineString::from(vec![
            (-1.0, 0.0),
            (-0.5, 0.0),
            (0.0, 0.0),
            (0.5, 0.0),
            (1.0, 0.0),
            (0.5, 0.5),
            (0.0, 1.0),
            (-0.5, 0.5),
            (-1.0, 0.0),
        ]);

        // Compute the largest rectangle
        let result = largest_fitting_rectangle(&to_polygon(&triangle_with_horizontal_hypotenuse))
            .expect("Failed to compute rectangle");

        // Print the generated rectangle
        println!("Generated Rectangle:");
        println!("Bottom-left: {:?}", result.0);
        println!("Top-right: {:?}", result.1);

        // Expected rectangle
        let expected_bottom_left = Point::new(-0.5, 0.0);
        let expected_top_right = Point::new(0.5, 0.5);

        // Assertions
        assert_eq!(
            result.0, expected_bottom_left,
            "Bottom-left corner mismatch"
        );
        assert_eq!(result.1, expected_top_right, "Top-right corner mismatch");
    }

    /// Custom assertion for comparing two `Point<f32>` objects with a tolerance.
    /// Panics if the difference between any coordinate exceeds the specified tolerance.
    ///
    /// # Arguments
    /// * `a` - First point.
    /// * `b` - Second point.
    /// * `tolerance` - Maximum allowable difference between corresponding coordinates.
    /// * `msg` - Optional message to display on assertion failure.
    pub fn assert_points_eq(a: Point<f32>, b: Point<f32>, tolerance: f32, msg: &str) {
        if (a.x() - b.x()).abs() > tolerance || (a.y() - b.y()).abs() > tolerance {
            panic!(
                "assertion failed: `|Point({}, {}) - Point({}, {})| <= {}`\n{}",
                a.x(),
                a.y(),
                b.x(),
                b.y(),
                tolerance,
                msg
            );
        }
    }

    #[test]
    fn test_octagon() {
        use std::f32::consts::PI;

        // Define the radius of the circle
        let r = 100.0;
        let start_angle = PI / 8.0; // Rotate the circle to start at π/8

        // Generate 8 equally spaced points around the circle
        let mut points = Vec::new();
        for i in 0..8 {
            let angle = start_angle + 2.0 * PI * (i as f32) / 8.0;
            points.push((r * angle.cos(), r * angle.sin()));
        }

        // Add midpoints
        let mut all_points = points.clone();
        println!("Midpoints:");
        for i in 0..8 {
            let next_i = (i + 1) % 8;
            let midpoint = (
                (points[i].0 + points[next_i].0) / 2.0,
                (points[i].1 + points[next_i].1) / 2.0,
            );
            all_points.push(midpoint);
            println!("Midpoint {}: {:?}", i + 1, midpoint);
        }

        // Create a LineString from all points
        let polygon = LineString::from(all_points);

        // Compute the largest rectangle
        let result = largest_fitting_rectangle(&to_polygon(&polygon)).expect("Failed to compute rectangle");

        // Print the generated rectangle
        println!("Generated Rectangle:");
        println!("Bottom-left: {:?}", result.0);
        println!("Top-right: {:?}", result.1);

        // Expected rectangle
        let expected_bottom_left = Point::new(-65.32815, -65.32815);
        let expected_top_right = Point::new(65.32815, 65.32815);

        // Assertions with tolerance
        assert_points_eq(
            result.0,
            expected_bottom_left,
            1e-4,
            "Bottom-left corner mismatch",
        );
        assert_points_eq(
            result.1,
            expected_top_right,
            1e-4,
            "Top-right corner mismatch",
        );
    }

    #[test]
    fn test_ellipsoid_rectangle_approximation() {
        use std::f32::consts::PI;

        // Ellipsoid parameters
        let a = 0.1; // Semi-axis along X
        let b = 0.1; // Semi-axis along Y
        let num_points = 10000; // Total points to generate

        // Generate points of ellipse
        let mut points = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let v = 2.0 * PI * (i as f32) / (num_points as f32);
            let x = a * v.cos();
            let y = b * v.sin();
            points.push((x, y)); // Only project X and Y for 2D rectangle computation
        }

        // Create a LineString from the projected points
        let polygon = LineString::from(points);

        // Compute the largest rectangle
        let result = largest_fitting_rectangle(&to_polygon(&polygon)).expect("Failed to compute rectangle");

        // Print the results
        println!("Generated Rectangle:");
        println!("Bottom-left: {:?}", result.0);
        println!("Top-right: {:?}", result.1);

        // Approximate expected rectangle dimensions
        // Approximate expected rectangle dimensions
        let denom = (a * a + b * b).sqrt();
        let expected_bottom_left = Point::new(-a * a / denom, -b * b / denom);
        let expected_top_right = Point::new(a * a / denom, b * b / denom);

        // Assertions with tolerance
        assert_points_eq(
            result.0,
            expected_bottom_left,
            0.1,
            "Bottom-left corner mismatch",
        );
        assert_points_eq(
            result.1,
            expected_top_right,
            0.1,
            "Top-right corner mismatch",
        );
    }

    fn to_polygon(linestring: &LineString<f32>) -> Polygon<f32> {
        // Create a Polygon from the LineString
        Polygon::new(linestring.clone(), vec![])
    }

    
}
