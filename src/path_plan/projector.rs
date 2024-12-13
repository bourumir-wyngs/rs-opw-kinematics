use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};
use parry3d::math::{Point as ParryPoint};
use parry3d::query::{Ray, RayCast};
use parry3d::shape::TriMesh;
use std::f32::consts::PI;

pub(crate) struct Projector {
    pub(crate) check_points: usize,
    pub(crate) radius: f32,
}

/// Enum representing the direction from which a ray originates along an axis.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum RayDirection {
    FromNegative,
    FromPositive,
}

impl RayDirection {
    /// Converts the enum variant into an integer value (-1 or +1).
    pub fn to_sign(self) -> f32 {
        match self {
            RayDirection::FromNegative => -1.0,
            RayDirection::FromPositive => 1.0,
        }
    }
}

/// Enum representing the axis of movement.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    /// Compute the ray's origin point based on the given direction, ray side, and distance.
    pub fn compute_ray_origin(
        self,
        point: &ParryPoint<f32>,
        ray_side: RayDirection,
        far: f32,
    ) -> ParryPoint<f32> {
        match self {
            Axis::X => ParryPoint::new(point.x + (far * ray_side.to_sign()), point.y, point.z),
            Axis::Y => ParryPoint::new(point.x, point.y + (far * ray_side.to_sign()), point.z),
            Axis::Z => ParryPoint::new(point.x, point.y, point.z + (far * ray_side.to_sign())),
        }
    }

    /// Compute the ray's direction vector based on the axis and ray side.
    pub fn compute_ray_direction(self, ray_side: RayDirection) -> Vector3<f32> {
        match self {
            Axis::X => Vector3::new(-ray_side.to_sign(), 0.0, 0.0),
            Axis::Y => Vector3::new(0.0, -ray_side.to_sign(), 0.0),
            Axis::Z => Vector3::new(0.0, 0.0, -ray_side.to_sign()),
        }
    }

    /// Generates a point on a circle around the given point along the specified axis.
    pub fn circle_point(
        &self,
        point: &ParryPoint<f32>,
        radius: f32,
        angle: f32,
    ) -> ParryPoint<f32> {
        match self {
            Axis::X => ParryPoint::new(
                point.x,
                point.y + radius * angle.cos(),
                point.z + radius * angle.sin(),
            ),
            Axis::Y => ParryPoint::new(
                point.x + radius * angle.cos(),
                point.y,
                point.z + radius * angle.sin(),
            ),
            Axis::Z => ParryPoint::new(
                point.x + radius * angle.cos(),
                point.y + radius * angle.sin(),
                point.z,
            ),
        }
    }
}

impl Projector {
    pub fn project_point(
        mesh: &TriMesh,
        point: &ParryPoint<f32>,
        direction: RayDirection,
        axis: Axis,
    ) -> Option<ParryPoint<f32>> {
        const FAR: f32 = 100.0; // Unit is assumed to be meters, enough for the robotic cell

        // Compute ray origin and direction using Direction methods
        let ray_origin = axis.compute_ray_origin(point, direction, FAR);
        let ray_direction = axis.compute_ray_direction(direction);
        let ray = Ray::new(ray_origin, ray_direction);

        // Perform ray casting
        if let Some(toi) = mesh.cast_ray(&Isometry3::identity(), &ray, 2.0 * FAR, true) {
            // Compute the intersection point using the ray origin and direction
            let intersection_point = ray_origin + ray_direction * toi;

            // Adjust only the projected coordinate for the specified axis
            match axis {
                Axis::X => Some(ParryPoint::new(intersection_point.x, point.y, point.z)),
                Axis::Y => Some(ParryPoint::new(point.x, intersection_point.y, point.z)),
                Axis::Z => Some(ParryPoint::new(point.x, point.y, intersection_point.z)),
            }
        } else {
            None
        }
    }

    /// Projects a point and performs planar regression to compute the normal.
    pub fn project(
        &self,
        mesh: &TriMesh,
        point: &ParryPoint<f32>,
        direction: RayDirection,
        axis: Axis,
    ) -> Option<Isometry3<f32>> {
        // Step 1: Project the central point
        let central_point = Self::project_point(mesh, point, direction, axis)?;
        println!("Central point projection: {:?}", central_point);

        // Step 2: Generate points on a circle in the XZ plane
        let mut points: Vec<ParryPoint<f32>> = vec![];
        let mut valid_points: Vec<Point3<f32>> = vec![
            Point3::new(central_point.x, central_point.y, central_point.z), // Include the central point
        ];

        for i in 0..self.check_points {
            let angle = 2.0 * PI * (i as f32) / (self.check_points as f32);
            let circle_point = axis.circle_point(&central_point, self.radius, angle);

            if let Some(intersection) = Self::project_point(mesh, &circle_point, direction, axis) {
                if false {
                    println!(
                        "Intersection at angle {}: {:?}",
                        angle.to_degrees(),
                        intersection
                    );
                }
                points.push(intersection);
                valid_points.push(Point3::new(intersection.x, intersection.y, intersection.z));
            } else {
                println!("No intersection at angle {}", angle.to_degrees());
            }
        }

        // Ensure enough valid points exist
        if valid_points.len() < 4 {
            println!("Not enough valid points for regression.");
            return None;
        }

        compute_plane_isometry(central_point, valid_points, axis, direction)
    }
}


/// Computes the average orientation of a plane from multiple points.
///
/// Ensures that the resulting orientation aligns as closely as possible
/// with the given `Axis` and `RayDirection`.
///
/// # Arguments
/// * `points` - A slice of points defining the plane.
/// * `axis` - The axis along which the preferred orientation is defined.
/// * `direction` - The direction (positive or negative) for the preferred orientation.
///
/// # Returns
/// * `Isometry3<f32>` - The isometry representing the average plane orientation.
fn average_plane_orientation(
    points: &[Vector3<f32>],
    axis: Axis,
    direction: RayDirection,
) -> Isometry3<f32> {
    if points.len() < 3 {
        panic!("At least three points are required to define a plane");
    }

    // Accumulate normals
    let mut normal_sum = Vector3::zeros();
    for i in 0..points.len() {
        for j in (i + 1)..points.len() {
            for k in (j + 1)..points.len() {
                // Compute vectors on the plane
                let v1 = points[j] - points[i];
                let v2 = points[k] - points[i];

                // Compute normal of the triangle
                let normal = v1.cross(&v2);

                // Accumulate normals (ignoring magnitude)
                if normal.norm() > 0.0 {
                    normal_sum += normal.normalize();
                }
            }
        }
    }

    // Average the normals
    let mut average_normal = normal_sum.normalize();

    // Ensure the normal vector is valid
    if average_normal.norm() == 0.0 {
        panic!("The points are collinear or do not define a valid plane");
    }

    // Convert `axis` and `direction` into a preferred orientation vector
    let preferred_direction = match axis {
        Axis::X => Vector3::x(),
        Axis::Y => Vector3::y(),
        Axis::Z => Vector3::z(),
    } * direction.to_sign();

    // Check orientation consistency with the preferred vector
    if average_normal.dot(&preferred_direction) < 0.0 {
        // Flip the normal if it's pointing away from the preferred direction
        average_normal = -average_normal;
    }

    // Create a quaternion to rotate the Z-axis (0, 0, 1) to the average normal
    let z_axis = Vector3::x();
    let rotation_quat = UnitQuaternion::rotation_between(&z_axis, &average_normal)
        .unwrap_or_else(|| UnitQuaternion::identity()); // Fallback if already aligned

    // Construct the Isometry3 with the rotation
    Isometry3::from_parts(Default::default(), rotation_quat)
}

fn compute_plane_isometry(
    centroid: ParryPoint<f32>,
    points: Vec<Point3<f32>>,
    axis: Axis,
    direction: RayDirection,
) -> Option<Isometry3<f32>> {

    // Convert Point3<f32> to Vector3<f32> for average_plane_orientation
    let vectors: Vec<Vector3<f32>> = points.into_iter().map(|p| p.coords).collect();

    // Call average_plane_orientation to compute the plane's orientation
    let orientation = average_plane_orientation(&vectors, axis, direction);

    // Combine the rotation with the translation (centroid) into an Isometry3
    Some(Isometry3::from_parts(centroid.coords.into(), orientation.rotation))
}

#[cfg(test)]
mod tests {
    use super::*;
    use parry3d::shape::TriMesh;

    const EDGE_POS: f32 = 1.2;
    const EDGE_NEG: f32 = -1.5;

    // The cube spans -1 to 1 with 0 in the center
    fn create_test_cube() -> TriMesh {
        // Create an axis-aligned cube as a TriMesh
        let vertices = vec![
            // Bottom face vertices
            ParryPoint::new(EDGE_NEG, 3.0 * EDGE_NEG, 2.0 * EDGE_NEG),
            ParryPoint::new(EDGE_POS, 3.0 * EDGE_NEG, 2.0 * EDGE_NEG),
            ParryPoint::new(EDGE_POS, 3.0 * EDGE_POS, 2.0 * EDGE_NEG),
            ParryPoint::new(EDGE_NEG, 3.0 * EDGE_POS, 2.0 * EDGE_NEG),
            // Top face vertices
            ParryPoint::new(EDGE_NEG, 3.0 * EDGE_NEG, 2.0 * EDGE_POS),
            ParryPoint::new(EDGE_POS, 3.0 * EDGE_NEG, 2.0 * EDGE_POS),
            ParryPoint::new(EDGE_POS, 3.0 * EDGE_POS, 2.0 * EDGE_POS),
            ParryPoint::new(EDGE_NEG, 3.0 * EDGE_POS, 2.0 * EDGE_POS),
        ];

        let indices = vec![
            [0, 1, 2],
            [0, 2, 3], // Bottom face
            [4, 5, 6],
            [4, 6, 7], // Top face
            [0, 1, 5],
            [0, 5, 4], // Front face
            [1, 2, 6],
            [1, 6, 5], // Right face
            [2, 3, 7],
            [2, 7, 6], // Back face
            [3, 0, 4],
            [3, 4, 7], // Left face
        ];
        TriMesh::new(vertices, indices)
    }

    // The points span -0.5 to 0.5. x value should not be accounted for
    fn create_test_points(axis: Axis) -> Vec<ParryPoint<f32>> {
        create_expected_results(0.92, axis)
    }

    fn create_expected_results(value: f32, axis: Axis) -> Vec<ParryPoint<f32>> {
        match axis {
            Axis::X => vec![
                ParryPoint::new(value, -0.5, -0.51),
                ParryPoint::new(value, -0.5, 0.51),
                ParryPoint::new(value, 0.5, -0.51),
                ParryPoint::new(value, 0.5, 0.51),
            ],
            Axis::Y => vec![
                ParryPoint::new(-0.5, value, -0.51),
                ParryPoint::new(-0.5, value, 0.51),
                ParryPoint::new(0.5, value, -0.51),
                ParryPoint::new(0.5, value, 0.51),
            ],
            Axis::Z => vec![
                ParryPoint::new(-0.5, -0.51, value),
                ParryPoint::new(-0.5, 0.51, value),
                ParryPoint::new(0.5, -0.51, value),
                ParryPoint::new(0.5, 0.51, value),
            ],
        }
    }

    const TOLERANCE: f32 = 0.002;

    // Helper function to compare two points with tolerance.
    fn points_are_close(a: &ParryPoint<f32>, b: &ParryPoint<f32>, tol: f32) -> bool {
        (a.x - b.x).abs() <= tol && (a.y - b.y).abs() <= tol && (a.z - b.z).abs() <= tol
    }

    #[test]
    fn test_pp_xpos() {
        let cube = create_test_cube();
        let test_points = create_test_points(Axis::X);
        let expected_results = create_expected_results(EDGE_POS, Axis::X);

        for i in 0..test_points.len() {
            let test_point = &test_points[i];
            let expected = &expected_results[i];
            let result =
                Projector::project_point(&cube, test_point, RayDirection::FromPositive, Axis::X);

            // Print both expected and received results for each test point.
            println!(
                "Test Point: {:?}, Expected: {:?}, Result: {:?}",
                test_point, expected, result
            );

            assert!(
                result
                    .map(|res| points_are_close(&res, expected, TOLERANCE))
                    .unwrap_or(false),
                "Projection failed for point {:?}: expected {:?}, got {:?}",
                test_point,
                expected,
                result
            );
        }
    }

    #[test]
    fn test_pp_x_neg() {
        let cube = create_test_cube();
        let test_points = create_test_points(Axis::X);
        let expected_results = create_expected_results(EDGE_NEG, Axis::X);

        for i in 0..test_points.len() {
            let test_point = &test_points[i];
            let expected = &expected_results[i];
            let result =
                Projector::project_point(&cube, test_point, RayDirection::FromNegative, Axis::X);

            // Print both expected and received results for each test point.
            println!(
                "Test Point: {:?}, Expected: {:?}, Result: {:?}",
                test_point, expected, result
            );

            assert!(
                result
                    .map(|res| points_are_close(&res, &expected, TOLERANCE))
                    .unwrap_or(false),
                "Projection failed for point {:?}: expected {:?}, got {:?}",
                test_point,
                expected,
                result
            );
        }
    }

    #[test]
    fn test_pp_ypos() {
        let cube = create_test_cube();
        let test_points = create_test_points(Axis::Y);
        let expected_results = create_expected_results(3.0 * EDGE_POS, Axis::Y);

        for i in 0..test_points.len() {
            let test_point = &test_points[i];
            let expected = &expected_results[i];
            let result =
                Projector::project_point(&cube, test_point, RayDirection::FromPositive, Axis::Y);

            // Print both expected and received results for each test point.
            println!(
                "Test Point: {:?}, Expected: {:?}, Result: {:?}",
                test_point, expected, result
            );

            assert!(
                result
                    .map(|res| points_are_close(&res, expected, TOLERANCE))
                    .unwrap_or(false),
                "Projection failed for point {:?}: expected {:?}, got {:?}",
                test_point,
                expected,
                result
            );
        }
    }

    #[test]
    fn test_pp_y_neg() {
        let cube = create_test_cube();
        let test_points = create_test_points(Axis::Y);
        let expected_results = create_expected_results(3.0 * EDGE_NEG, Axis::Y);

        for i in 0..test_points.len() {
            let test_point = &test_points[i];
            let expected = &expected_results[i];
            let result =
                Projector::project_point(&cube, test_point, RayDirection::FromNegative, Axis::Y);

            // Print both expected and received results for each test point.
            println!(
                "Test Point: {:?}, Expected: {:?}, Result: {:?}",
                test_point, expected, result
            );

            assert!(
                result
                    .map(|res| points_are_close(&res, &expected, TOLERANCE))
                    .unwrap_or(false),
                "Projection failed for point {:?}: expected {:?}, got {:?}",
                test_point,
                expected,
                result
            );
        }
    }

    #[test]
    fn test_pp_from_above() {
        let cube = create_test_cube();
        let test_points = create_test_points(Axis::Z);
        let expected_results = create_expected_results(2.0 * EDGE_POS, Axis::Z);

        for i in 0..test_points.len() {
            let test_point = &test_points[i];
            let expected = &expected_results[i];
            let result =
                Projector::project_point(&cube, test_point, RayDirection::FromPositive, Axis::Z);

            // Print both expected and received results for each test point.
            println!(
                "Test Point: {:?}, Expected: {:?}, Result: {:?}",
                test_point, expected, result
            );

            assert!(
                result
                    .map(|res| points_are_close(&res, expected, TOLERANCE))
                    .unwrap_or(false),
                "Projection failed for point {:?}: expected {:?}, got {:?}",
                test_point,
                expected,
                result
            );
        }
    }

    #[test]
    fn test_pp_from_below() {
        let cube = create_test_cube();
        let test_points = create_test_points(Axis::Z);
        let expected_results = create_expected_results(2.0 * EDGE_NEG, Axis::Z);

        for i in 0..test_points.len() {
            let test_point = &test_points[i];
            let expected = &expected_results[i];
            let result =
                Projector::project_point(&cube, test_point, RayDirection::FromNegative, Axis::Z);

            // Print both expected and received results for each test point.
            println!(
                "Test Point: {:?}, Expected: {:?}, Result: {:?}",
                test_point, expected, result
            );

            assert!(
                result
                    .map(|res| points_are_close(&res, expected, TOLERANCE))
                    .unwrap_or(false),
                "Projection failed for point {:?}: expected {:?}, got {:?}",
                test_point,
                expected,
                result
            );
        }
    }

}
