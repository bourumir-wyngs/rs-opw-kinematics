use arrsac::Arrsac;
use bevy::prelude::Or;
use geo::CoordsIter;
use nalgebra::{Point3, Unit, Vector3};
use parry3d::shape::TriMesh;
use rand::rngs::SmallRng;
use rand::SeedableRng;
use sample_consensus::{Consensus, Estimator, Model};
use crate::organized_point::OrganizedPoint;

/// Represents a plane in 3D space defined by its normal vector and distance from the origin.
struct Plane {
    /// The normal vector (A, B, C) of the plane equation Ax + By + Cz + D = 0
    normal: Vector3<f32>,

    /// The offset (D) in the plane equation    
    d: f32,
}

#[derive(Clone, Copy, Debug)]
struct ProjectedPoint {
    x: f32,
    y: f32,
    id: usize,
}

impl PartialEq for ProjectedPoint {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for ProjectedPoint {}

/// Implements the `Model` trait for `Plane`, enabling it to be used in ARRSAC.
impl Model<OrganizedPoint> for Plane {
    /// The absolute distance of the point from the plane
    fn residual(&self, point: &OrganizedPoint) -> f64 {
        // Compute the signed distance from the plane using the plane equation:
        // Distance = |(A*x + B*y + C*z + D)|
        let distance = self.normal.dot(&point.point.coords) + self.d;
        distance.abs() as f64
    }
}

struct PlaneEstimator;

fn generate_nearby_samples(points: &[OrganizedPoint], radius: f32) -> Vec<OrganizedPoint> {
    use rand::seq::SliceRandom;

    if points.is_empty() {
        return vec![];
    }

    // Pick a random center point
    let center = points.choose(&mut rand::thread_rng()).unwrap();

    // Select points within the specified radius
    let nearby_points: Vec<OrganizedPoint> = points
        .iter()
        .filter(|p| p.distance(center) < radius)
        .cloned()
        .collect();

    nearby_points
}

impl Estimator<OrganizedPoint> for PlaneEstimator {
    type Model = Plane;
    type ModelIter = Option<Self::Model>;
    const MIN_SAMPLES: usize = 3;

    fn estimate<I: Iterator<Item = OrganizedPoint>>(&self, mut data: I) -> Self::ModelIter {
        let points: Vec<_> = data.take(Self::MIN_SAMPLES).collect();
        if points.len() < Self::MIN_SAMPLES {
            return None;
        }

        // Select a subset of nearby points to prevent steep plane selection
        let nearby_points = generate_nearby_samples(&points, 0.05);

        if nearby_points.len() < 3 {
            return None;
        }

        let p1 = nearby_points[0];
        let p2 = nearby_points[1];
        let p3 = nearby_points[2];

        let normal = (p2.point - p1.point).cross(&(p3.point - p1.point)).normalize();
        let d = -normal.dot(&p1.point.coords);

        Some(Plane { normal, d })
    }
}


fn fit_plane_least_squares(points: &[OrganizedPoint]) -> Plane {
    // Compute the centroid by summing up all coordinates and dividing by the total number of points.
    let centroid_coords = points
        .iter()
        .map(|p| p.point.coords) // Extract coords as Vector3<f32>
        .reduce(|a, b| a + b) // Sum all vectors
        .unwrap()
        / (points.len() as f32); // Divide by the number of points to get the mean coords.

    // Convert the centroid back into a Point3
    let centroid = Point3::from(centroid_coords);

    let mut covariance = nalgebra::Matrix3::<f32>::zeros();
    for &p in points {
        let diff = p.point - centroid; // Compute the difference vector
        covariance += diff * diff.transpose(); // Accumulate the outer product of the difference vector
    }

    // Perform eigenvalue decomposition to find the best normal vector
    let eigen = covariance.symmetric_eigen();

    // Find the index of the smallest eigenvalue
    let min_eigen_idx = eigen.eigenvalues.iter().enumerate()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(idx, _)| idx)
        .unwrap();

    let normal: Vector3<f32> = eigen.eigenvectors.column(min_eigen_idx).into();
    let d = -normal.dot(&centroid.coords);

    Plane { normal, d }
}

pub fn build_plane(points_to_fit: &Vec<OrganizedPoint>, filter_target: &Vec<OrganizedPoint>, max_distance_till_plane: f32) -> Vec<OrganizedPoint> {
    if points_to_fit.len() < 3 {
        return points_to_fit.clone(); // Any 3 points fit into one plane
    }

    let rng = SmallRng::seed_from_u64(42);
    let mut arrsac = Arrsac::new(max_distance_till_plane as f64, rng)
        .initialization_hypotheses(512)
        .max_candidate_hypotheses(128)
        .block_size(128);

    let mut best_inliers = Vec::new();
    let mut best_plane = Plane {
        normal: Default::default(),
        d: 0.0,
    };

    for _ in 0..32 {
        if let Some((plane, inliers)) = arrsac.model_inliers(&PlaneEstimator, points_to_fit.iter().copied()) {
            if inliers.len() > best_inliers.len() {
                best_inliers = inliers;
                best_plane = plane;
            }
        }
    }

    let filtered_vertices: Vec<_> = best_inliers.iter().map(|&i| points_to_fit[i]).collect();
    let best_plane = fit_plane_least_squares(filtered_vertices.as_slice());

    let distance_threshold = max_distance_till_plane as f64; // Use a scaling factor
    let filtered_vertices: Vec<_> = filter_target
        .iter().cloned()
        .filter(|p| best_plane.residual(&p) < 2.0*distance_threshold)
        .collect();

    filtered_vertices
}

fn project_to_plane_2d(points: &[Point3<f32>], plane: &Plane) -> Vec<ProjectedPoint> {
    use nalgebra::{Vector3, Unit};

    // Compute two basis vectors that span the plane
    let u = Unit::new_normalize(Vector3::new(1.0, 0.0, -plane.normal.x / plane.normal.z));
    let v = plane.normal.cross(&u);

    // Convert 3D points to 2D coordinates, with their IDs
    points
        .iter()
        .enumerate() // To include the index (id)
        .map(|(id, p)| {
            let local = p.coords; // Convert Point3 to Vector3
            let x = u.dot(&local);
            let y = v.dot(&local);
            ProjectedPoint { x, y, id }
        })
        .collect()
}

fn find_max_filled_rectangle_and_filter(
    points_2d: &[ProjectedPoint],
    width: f32,
    height: f32,
) -> Option<Vec<usize>> {
    if points_2d.is_empty() {
        return None;
    }

    // Create a vector of references to the points and sort by x-coordinate
    let mut sorted_points: Vec<&ProjectedPoint> = points_2d.iter().collect();
    sorted_points.sort_by(|a, b| a.x.partial_cmp(&b.x).unwrap());

    let mut max_count = 0;
    let mut best_ids = Vec::new();
    let mut y_values = Vec::new();

    // Slide the rectangle along the x-axis
    for i in 0..sorted_points.len() {
        let x_start = sorted_points[i].x;
        let x_end = x_start + width;

        // Collect references to valid points within the x-axis range
        let in_strip: Vec<&ProjectedPoint> = sorted_points
            .iter()
            .filter(|p| p.x >= x_start && p.x <= x_end)
            .copied()
            .collect();

        // Precompute y-values of points in the strip and sort once
        y_values.clear();
        y_values.extend(in_strip.iter().map(|p| p.y));
        y_values.sort_by(|a, b| a.partial_cmp(b).unwrap());

        // Apply a sliding window to find the optimal rectangle in the y-dimension
        let mut start_idx = 0;
        for end_idx in 0..y_values.len() {
            let y_start = y_values[start_idx];
            let y_end = y_start + height;

            // Shrink window if points are outside height constraints
            while y_values[end_idx] > y_end && start_idx < end_idx {
                start_idx += 1;
            }

            // Calculate the number of points in the current rectangle
            let count = end_idx - start_idx + 1;
            if count > max_count {
                max_count = count;

                // Collect IDs of points within the rectangle boundaries
                let valid_ids: Vec<usize> = in_strip
                    .iter()
                    .filter(|p| p.y >= y_start && p.y <= y_end)
                    .map(|p| p.id)
                    .collect();

                best_ids = valid_ids; // Update the best IDs
            }
        }
    }

    Some(best_ids)
}

pub fn find_best_rectangle_inliers(points: Vec<Point3<f32>>, plane: &Plane, width: f32, height: f32) -> Vec<Point3<f32>> {
    if points.len() < 3 {
        return points; // Not enough points for a rectangle
    }

    // Step 1: Project points onto the plane
    let points_2d = project_to_plane_2d(&points, plane);

    // Step 2: Find the best-fitting rectangle and the filtered 2D IDs
    if let Some(filtered_ids) = find_max_filled_rectangle_and_filter(&points_2d, width, height) {
        // Step 3: Map the filtered IDs back to original 3D points
        return filtered_ids
            .into_iter()
            .map(|id| points[id]) // Use point IDs to get the original Point3<f32>
            .collect();
    }

    points
}

