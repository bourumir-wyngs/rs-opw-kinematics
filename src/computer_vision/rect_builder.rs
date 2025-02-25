use nalgebra::{Matrix3, Vector2, Vector3};
use crate::organized_point::OrganizedPoint;
use crate::plane::{ProjectedPoint, Plane};

// Represent a rectangle in 2D space
struct Rectangle {
    pub center: Vector2<f32>,
    pub width: f32,
    pub height: f32,
    pub cos: f32,
    pub sin: f32,
}

impl Rectangle {
    // Check if a projected point is inside the rectangle
    fn contains(&self, point: &ProjectedPoint) -> bool {
        let translated_x = point.x - self.center.x;
        let translated_y = point.y - self.center.y;

        // Use sincos for simultaneous calculation of sine and cosine
        let rotated_x = translated_x * self.cos + translated_y * self.sin;
        let rotated_y = -translated_x * self.sin + translated_y * self.cos;

        rotated_x.abs() <= self.width / 2.0 && rotated_y.abs() <= self.height / 2.0
    }
}

// Represent a full model of a rectangle fitted to a plane
pub struct RectangleModel {
    pub plane: Plane,
    rectangle: Rectangle,
}

// RANSAC Estimator for Rectangle Fitting
pub struct RectangleEstimator;

impl RectangleEstimator {
    pub fn ransac_rectangle_fitting(
        points: &Vec<OrganizedPoint>,
        iterations: usize,
        width: f32,
        height: f32,
    ) -> Vec<OrganizedPoint> {
        if let Ok(plane) = Plane::fit(points) {
            let projected_points = plane.project(points); // Project points onto the plane

            let mut best_inliers: Vec<u32> = Vec::new();
            for _ in 0..iterations {
                // Generate a candidate rectangle and retrieve inliers
                let inliers = Self::generate_candidate_rectangle(&projected_points, width, height);

                // Check if this candidate has more inliers
                if inliers.len() > best_inliers.len() {
                    best_inliers = inliers;
                }
            }

            let filtered_points: Vec<OrganizedPoint> = best_inliers
                .iter()
                .map(|&id| points[id as usize].clone()) // Retrieve the original points by ID
                .collect();

            filtered_points
        } else {
            Vec::new()
        }
    }

    fn generate_candidate_rectangle(
        projected_points: &Vec<ProjectedPoint>,
        width: f32,
        height: f32,
    ) -> Vec<u32> {
        use nalgebra::Vector2;
        use rand::seq::SliceRandom;

        let mut rng = rand::thread_rng();

        // Add a maximum number of attempts to avoid infinite loops
        let max_attempts = 100;

        for _attempt in 0..max_attempts {
            let sampled_points: Vec<&ProjectedPoint> =
                projected_points.choose_multiple(&mut rng, 2).collect();

            if sampled_points.len() == 2 {
                let p1 = sampled_points[0];
                let p2 = sampled_points[1];
                let dx = p2.x - p1.x;
                let dy = p2.y - p1.y;

                // Check if the points are within the allowable distance
                if dx * dx + dy * dy <= width * width + height * height {
                    // Compute the center between the sampled points
                    let center_x = (p1.x + p2.x) / 2.0;
                    let center_y = (p1.y + p2.y) / 2.0;
                    let center = Vector2::new(center_x, center_y);

                    // Calculate the angle of the rectangle based on the line between the points
                    let angle = dy.atan2(dx).sin_cos();

                    // Construct the candidate rectangle
                    let candidate_rectangle = Rectangle {
                        center,
                        width,
                        height,
                        sin: angle.0,
                        cos: angle.1,
                    };

                    // Filter projected points that fit into this rectangle
                    let filtered_ids: Vec<u32> = projected_points
                        .iter()
                        .filter(|point| candidate_rectangle.contains(point)) // Check if the point is within the rectangle
                        .map(|point| point.id) // Collect the IDs of the points
                        .collect();

                    return filtered_ids;
                }
            }
        }

        // If no suitable rectangle could be generated, return an empty vector
        Vec::new()
    }
}
