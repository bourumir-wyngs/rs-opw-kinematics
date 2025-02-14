use arrsac::Arrsac;
use geo::CoordsIter;
use nalgebra::{Point3, Unit, Vector3};
use parry3d::shape::TriMesh;
use rand::rngs::SmallRng;
use rand::SeedableRng;
use sample_consensus::{Consensus, Estimator, Model};

/// Represents a plane in 3D space defined by its normal vector and distance from the origin.
struct Plane {
    /// The normal vector (A, B, C) of the plane equation Ax + By + Cz + D = 0
    normal: Vector3<f32>,

    /// The offset (D) in the plane equation    
    d: f32,
}

/// Implements the `Model` trait for `Plane`, enabling it to be used in ARRSAC.
impl Model<Point3<f32>> for Plane {
    /// The absolute distance of the point from the plane
    fn residual(&self, point: &Point3<f32>) -> f64 {
        // Compute the signed distance from the plane using the plane equation:
        // Distance = |(A*x + B*y + C*z + D)|
        let distance = self.normal.dot(&point.coords) + self.d;

        // Return the absolute distance, ensuring a non-negative residual value.
        let r = distance.abs() as f64;
        if r < 0.005 {
            r
        } else {
            f64::INFINITY // discourage points anywhere more outside the plane
        }
    }
}

struct PlaneEstimator;

fn generate_nearby_samples(points: &[Point3<f32>], radius: f32) -> Vec<Point3<f32>> {
    use rand::seq::SliceRandom;

    if points.is_empty() {
        return vec![];
    }

    // Pick a random center point
    let center = points.choose(&mut rand::thread_rng()).unwrap();

    // Select points within the specified radius
    let nearby_points: Vec<Point3<f32>> = points
        .iter()
        .filter(|p| (*p - center).norm() < radius)
        .cloned()
        .collect();

    nearby_points
}

impl Estimator<Point3<f32>> for PlaneEstimator {
    type Model = Plane;
    type ModelIter = Option<Self::Model>;
    const MIN_SAMPLES: usize = 64;

    fn estimate<I: Iterator<Item = Point3<f32>>>(&self, mut data: I) -> Self::ModelIter {
        let points: Vec<_> = data.take(Self::MIN_SAMPLES).collect();
        if points.len() < Self::MIN_SAMPLES {
            return None;
        }

        // Select a subset of nearby points to prevent steep plane selection
        let nearby_points = generate_nearby_samples(&points, 0.01); 

        if nearby_points.len() < 3 {
            return None;
        }

        let p1 = nearby_points[0];
        let p2 = nearby_points[1];
        let p3 = nearby_points[2];

        let normal = (p2 - p1).cross(&(p3 - p1)).normalize();
        let d = -normal.dot(&p1.coords);

        Some(Plane { normal, d })
    }
}


fn fit_plane_least_squares(points: &[Point3<f32>]) -> Plane {
    // Compute the centroid by summing up all coordinates and dividing by the total number of points.
    let centroid_coords = points
        .iter()
        .map(|p| p.coords) // Extract coords as Vector3<f32>
        .reduce(|a, b| a + b) // Sum all vectors
        .unwrap()
        / (points.len() as f32); // Divide by the number of points to get the mean coords.

    // Convert the centroid back into a Point3
    let centroid = Point3::from(centroid_coords);

    let mut covariance = nalgebra::Matrix3::<f32>::zeros();
    for &p in points {
        let diff = p - centroid; // Compute the difference vector
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

pub fn build_plane(mesh: &TriMesh, max_distance_till_plane: f32) -> Vec<Point3<f32>> {
    let points: Vec<Point3<f32>> = mesh
        .vertices()
        .iter()
        .map(|v| Point3::new(v.x, v.y, v.z))
        .collect();

    if points.len() < 3 {
        return points; // Any 3 points fit into one plane
    }

    let mut rng = SmallRng::seed_from_u64(42);
    let mut arrsac = Arrsac::new(max_distance_till_plane as f64, rng)
        .initialization_hypotheses(512)
        .max_candidate_hypotheses(128)
        .block_size(128);

    let mut best_inliers = Vec::new();
    let mut best_plane = Plane {
        normal: Default::default(),
        d: 0.0,
    };

    for _ in 0..8 {
        if let Some((plane, inliers)) = arrsac.model_inliers(&PlaneEstimator, points.iter().copied()) {
            if inliers.len() > best_inliers.len() {
                best_inliers = inliers;
                best_plane = plane;
            }
        }
    }

    let filtered_vertices: Vec<_> = best_inliers.iter().map(|&i| mesh.vertices()[i]).collect();
    let best_plane = fit_plane_least_squares(filtered_vertices.as_slice());

    let distance_threshold = max_distance_till_plane as f64; // Use a scaling factor
    let filtered_vertices: Vec<_> = points
        .into_iter()
        .filter(|p| best_plane.residual(p) < 2.0 * distance_threshold)
        .collect();

    //filtered_vertices
    find_best_rectangle_inliers(filtered_vertices, &best_plane, 0.04, 0.08) // w/h swapped
}

fn project_to_plane_2d(points: &[Point3<f32>], plane: &Plane) -> Vec<(f32, f32)> {
    use nalgebra::{Vector3, Unit};

    // Compute two basis vectors that span the plane
    let u = Unit::new_normalize(Vector3::new(1.0, 0.0, -plane.normal.x / plane.normal.z));
    let v = plane.normal.cross(&u);

    // Convert 3D points to 2D coordinates
    points
        .iter()
        .map(|p| {
            let local = p.coords; // Convert Point3 to Vector3
            let x = u.dot(&local);
            let y = v.dot(&local);
            (x, y)
        })
        .collect()
}

fn find_max_filled_rectangle(points_2d: &[(f32, f32)], width: f32, height: f32) -> Option<((f32, f32), (f32, f32), (f32, f32), (f32, f32))> {
    if points_2d.is_empty() {
        return None;
    }

    // Sort points by x-coordinate ahead of time
    let mut sorted_points = points_2d.to_vec();
    sorted_points.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    let mut max_count = 0;
    let mut best_rect = None;
    let mut y_values = Vec::new();

    // Slide the rectangle along the x-axis
    for i in 0..sorted_points.len() {
        let (x_start, _) = sorted_points[i];
        let x_end = x_start + width;

        // Collect valid points within the x-axis range
        let in_strip = sorted_points
            .iter()
            .filter(|&&(x, _)| x >= x_start && x <= x_end)
            .cloned()
            .collect::<Vec<_>>();

        // Precompute y-values of points in the strip and sort once
        y_values.clear();
        y_values.extend(in_strip.iter().map(|&(_, y)| y));
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
                best_rect = Some((
                    (x_start, y_start),
                    (x_end, y_start),
                    (x_end, y_end),
                    (x_start, y_end),
                ));
            }
        }
    }

    best_rect
}

fn filter_points_in_rectangle(points_2d: &[(f32, f32)], rect: &((f32, f32), (f32, f32), (f32, f32), (f32, f32))) -> Vec<(f32, f32)> {
    let (p1, p2, p3, p4) = *rect;

    points_2d
        .iter()
        .filter(|&&(x, y)| x >= p1.0 && x <= p2.0 && y >= p1.1 && y <= p3.1)
        .cloned()
        .collect()
}

fn transform_to_3d(points_2d: &[(f32, f32)], plane: &Plane) -> Vec<Point3<f32>> {
    use nalgebra::Vector3;

    // Compute two basis vectors for the plane
    let u = Unit::new_normalize(Vector3::new(1.0, 0.0, -plane.normal.x / plane.normal.z));
    let v = Unit::new_normalize(plane.normal.cross(&u));

    points_2d
        .iter()
        .map(|&(x, y)| {
            let pos = (*u.as_ref()) * x + (*v.as_ref()) * y + plane.normal * (-plane.d);
            Point3::from(pos)
        })
        .collect()
}

pub fn find_best_rectangle_inliers(points: Vec<Point3<f32>>, plane: &Plane, width: f32, height: f32) -> Vec<Point3<f32>> {
    if points.len() < 3 {
        return points; // Not enough points for a rectangle
    }

    // Step 1: Project points onto plane
    let points_2d = project_to_plane_2d(&points, plane);

    // Step 2: Find best-fitting rectangle
    if let Some(rect) = find_max_filled_rectangle(&points_2d, width, height) {
        // Step 3: Extract points inside the rectangle
        let filtered_2d = filter_points_in_rectangle(&points_2d, &rect);

        // Step 4: Convert points back to 3D
        return transform_to_3d(&filtered_2d, plane);
    }

    vec![] // No valid rectangle found
}



