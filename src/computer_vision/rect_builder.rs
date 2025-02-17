use crate::organized_point::OrganizedPoint;
use nalgebra::{Matrix3, Vector2, Vector3};

// Represent a plane in 3D space
#[derive(Clone, Copy, Debug)]
pub struct Plane {
    pub normal: Vector3<f32>,
    pub d: f32, // Distance from origin
}

// Represent a projected point in 2D space
#[derive(Clone, Copy, Debug)]
struct ProjectedPoint {
    x: f32,
    y: f32,
    id: u32,
}

impl PartialEq for ProjectedPoint {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for ProjectedPoint {}

// Represent a rectangle in 2D space
pub struct Rectangle {
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
    pub rectangle: Rectangle,
}

// Project all points onto the plane and generate 2D projections
fn project_to_plane(points: &Vec<OrganizedPoint>, plane: &Plane) -> Vec<ProjectedPoint> {
    // Ensure a robust selection of a reference vector
    let reference = if plane.normal.x.abs() < 0.9 {
        Vector3::x_axis() // Prefer X unless normal is mostly X
    } else {
        Vector3::z_axis() // Otherwise, use Z
    };

    // Compute two orthonormal basis vectors spanning the plane
    let u = plane.normal.cross(&reference).normalize();
    let v = plane.normal.cross(&u).normalize();

    points
        .iter()
        .enumerate()
        .map(|(id, point)| {
            // Project onto the plane
            let distance = point.point.coords.dot(&plane.normal) + plane.d;
            let projected_point_3d = point.point.coords - distance * plane.normal;

            // Convert to 2D using the plane's basis vectors
            let x = projected_point_3d.dot(&u);
            let y = projected_point_3d.dot(&v);

            ProjectedPoint {
                x,
                y,
                id: id as u32,
            }
        })
        .collect()
}

fn fit_plane_least_squares(points: &[OrganizedPoint]) -> Plane {
    let centroid = points
        .iter()
        .fold(Vector3::zeros(), |acc, p| acc + p.point.coords)
        / points.len() as f32;

    let mut covariance_matrix = Matrix3::zeros();
    for point in points {
        let diff = point.point.coords - centroid;
        covariance_matrix += diff * diff.transpose();
    }

    let svd = covariance_matrix.svd(true, true);
    if let Some(v_t) = svd.v_t {
        let normal = v_t.row(2).transpose();
        Plane {
            normal: normal.normalize(),
            d: -normal.dot(&centroid),
        }
    } else {
        panic!("SVD failed, covariance matrix is singular.");
    }
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
        let plane = fit_plane_least_squares(points); // Fit a plane from the points
        let projected_points = project_to_plane(points, &plane); // Project points onto the plane

        let mut best_inliers: Vec<u32> = Vec::new();
        for _ in 0..iterations {
            // Generate a candidate rectangle and retrieve inliers
            let inliers = generate_candidate_rectangle(&projected_points, width, height);

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
