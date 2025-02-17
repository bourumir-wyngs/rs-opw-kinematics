use arrsac::Arrsac;
use nalgebra::{Point3, Vector3};
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

    fn estimate<I: Iterator<Item = OrganizedPoint>>(&self, data: I) -> Self::ModelIter {
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
    for _ in 0..32 {
        if let Some((_plane, inliers)) = arrsac.model_inliers(&PlaneEstimator, points_to_fit.iter().copied()) {
            if inliers.len() > best_inliers.len() {
                best_inliers = inliers;
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



