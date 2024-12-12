use nalgebra::{
    DMatrix, Isometry3, Matrix3, Point3, Rotation3, Unit, UnitQuaternion, Vector3,
};
use parry3d::math::Point as ParryPoint;
use parry3d::query::{Ray, RayCast};
use parry3d::shape::TriMesh;
use std::f32::consts::PI;

struct Projector {
    check_points: usize,
    radius: f32,
}

use nalgebra::DVector;

/// Perform regression for z = ax + by + c, detect degeneracy, and handle alternative regressions.
fn detect_and_regress(
    x: &[f32],
    y: &[f32],
    z: &[f32],
) -> Option<(
    Option<(f32, f32, f32)>,
    Option<(f32, f32, f32)>,
    Option<(f32, f32, f32)>,
)> {
    let n = x.len();
    if n != y.len() || n != z.len() || n < 3 {
        return None; // Ensure valid input
    }

    // Helper function to calculate variance
    let variance = |data: &[f32]| -> f32 {
        let mean = data.iter().copied().sum::<f32>() / data.len() as f32;
        data.iter().map(|v| (v - mean).powi(2)).sum::<f32>() / data.len() as f32
    };

    // Step 1: Compute variances
    let var_x = variance(x);
    let var_y = variance(y);
    let var_z = variance(z);

    println!(
        "Variance: x = {:.5}, y = {:.5}, z = {:.5}",
        var_x, var_y, var_z
    );

    // Threshold for near-degeneracy
    let epsilon = 1e-6;

    // Step 2: Perform the primary regression z = ax + by + c
    let primary_regression = plane_regression(x, y, z);

    // Step 3: Perform alternative regressions if necessary
    let alternative_regressions = if var_x < epsilon {
        println!("Degenerate case detected for x (near-vertical plane). Performing x = dy + ez + f regression.");
        let alt = plane_regression(y, z, x); // x = dy + ez + f
        (alt, None)
    } else if var_y < epsilon {
        println!("Degenerate case detected for y (near-vertical plane). Performing y = gx + hz + q regression.");
        let alt = plane_regression(x, z, y); // y = gx + hz + q
        (None, alt)
    } else {
        (None, None)
    };

    Some((
        primary_regression,
        alternative_regressions.0,
        alternative_regressions.1,
    ))
}

/// Perform regression to find a, b, c in the equation z = ax + by + c
fn plane_regression(x: &[f32], y: &[f32], z: &[f32]) -> Option<(f32, f32, f32)> {
    let n = x.len();
    if n != y.len() || n != z.len() || n < 3 {
        return None; // Ensure valid input
    }

    let mut a_matrix = DMatrix::zeros(n, 3);
    let mut z_vector = DVector::zeros(n);

    for i in 0..n {
        a_matrix[(i, 0)] = x[i];
        a_matrix[(i, 1)] = y[i];
        a_matrix[(i, 2)] = 1.0; // Constant term for c
        z_vector[i] = z[i];
    }

    let a_t = a_matrix.transpose();
    let normal_matrix = a_t.clone() * a_matrix; // A^T A
    let pseudo_inverse = normal_matrix.try_inverse()?; // (A^T A)^-1
    let coefficients = pseudo_inverse * a_t * z_vector; // (A^T A)^-1 A^T Z

    Some((coefficients[0], coefficients[1], coefficients[2])) // Return a, b, and c
}

/// Convert plane coefficients to Isometry3 rotation and translation
fn plane_to_isometry(normal: Vector3<f32>, centroid: Point3<f32>) -> Isometry3<f32> {

    fn select_best_arbitrary_vector(z_axis: Vector3<f32>) -> Vector3<f32> {
        // Compute dot products with the standard basis vectors
        let dot_x = z_axis.dot(&Vector3::x()).abs();
        let dot_y = z_axis.dot(&Vector3::y()).abs();
        let dot_z = z_axis.dot(&Vector3::z()).abs();

        // Select the basis vector with the smallest absolute dot product
        if dot_x <= dot_y && dot_x <= dot_z {
            Vector3::x() // Least aligned with z_axis
        } else if dot_y <= dot_z {
            Vector3::y()
        } else {
            Vector3::z()
        }
    }
    
    // Step 1: Normalize the normal vector (Z-axis of the plane)
    let z_axis = Unit::new_normalize(normal);

    // Step 2: Determine an arbitrary perpendicular vector
    let arbitrary_vector = select_best_arbitrary_vector(z_axis.into_inner());

    // Step 3: Compute the X-axis (perpendicular to Z-axis and arbitrary vector)
    let x_axis = Unit::new_normalize(arbitrary_vector.cross(&z_axis.into_inner()));

    // Step 4: Compute the Y-axis as cross product of Z and X
    let y_axis = Unit::new_normalize(z_axis.cross(&x_axis));

    // Step 5: Construct the rotation matrix
    let rotation = Matrix3::from_columns(&[
        x_axis.into_inner(),
        y_axis.into_inner(),
        z_axis.into_inner(),
    ]);

    let r3 = Rotation3::from_matrix_unchecked(rotation);
    let quaternion = UnitQuaternion::from_rotation_matrix(&r3);

    // Step 6: Create Isometry3 with rotation and centroid as translation
    Isometry3::from_parts(centroid.coords.into(), quaternion)
}

impl Projector {
    const FROM_NEGATIVE: i32 = -1;
    const FROM_POSITIVE: i32 = 1;
    
    /// Projects a 2D pose onto the surface of the mesh along the X-axis, adjusting
    /// its `x` coordinate to lie on the mesh surface while keeping the `y` and `z`
    /// coordinates unchanged.
    ///
    /// The ray used for projection starts far along the X-axis, either on the
    /// negative side (`direction = -1`) or the positive side (`direction = +1`),
    /// and travels in the opposite direction. The ray intersects the mesh to find
    /// the closest point on the surface.
    ///
    /// # Parameters
    /// - `mesh`: The triangular mesh (`TriMesh`) representing the surface onto which
    ///   the point is projected.
    /// - `point`: The 2D pose represented as a `ParryPoint<f32>`. Its `y` and `z`
    ///   coordinates remain unchanged after projection, while `x` is adjusted.
    /// - `direction`: An integer value (`-1` or `+1`) specifying the direction of the ray:
    ///   - `-1` or `FROM_NEGATIVE`: The ray starts far on the negative X-axis and moves towards positive X.
    ///   - `+1` or `FROM_POSITIVE`: The ray starts far on the positive X-axis and moves towards negative X.
    ///
    /// # Returns
    /// - `Some(ParryPoint<f32>)`: The point projected onto the mesh surface, with the
    ///   adjusted `x` coordinate.
    /// - `None`: If the ray does not intersect the mesh.
    ///
    /// # Panics
    /// - Panics if `direction` is not `-1` or `+1`.
    /// ```
    pub fn project_point(
        mesh: &TriMesh,
        point: &ParryPoint<f32>,
        direction: i32,
    ) -> Option<ParryPoint<f32>> {
        const FAR: f32 = 100.0; // Unit is assumed to be meters, 100 meters is more than enough for the robotic cell
        assert!(
            direction == -1 || direction == 1,
            "Direction must be -1 or +1"
        );

        // Start the ray based on the direction parameter
        let ray_origin = ParryPoint::new(point.x + (FAR * direction as f32), point.y, point.z);
        let ray_direction = Vector3::new(-direction as f32, 0.0, 0.0); // Shoot ray in opposite of the origin side
        let ray = Ray::new(ray_origin, ray_direction);

        // Perform ray casting
        if let Some(toi) = mesh.cast_ray(&Isometry3::identity(), &ray, 2.0 * FAR, true) {
            // Compute the intersection point using the ray origin and direction
            let intersection_point = ray_origin + ray_direction * toi;
            Some(ParryPoint::new(intersection_point.x, point.y, point.z))
        } else {
            None
        }
    }

    fn compute_plane_isometry(points: &[Point3<f32>]) -> Option<Isometry3<f32>> {
        if points.len() < 3 {
            return None; // Not enough points to define a plane
        }

        // Extract x, y, z coordinates from the points
        let x: Vec<f32> = points.iter().map(|p| p.x).collect();
        let y: Vec<f32> = points.iter().map(|p| p.y).collect();
        let z: Vec<f32> = points.iter().map(|p| p.z).collect();

        // Perform regression to get the plane equation
        let regression_result = detect_and_regress(&x, &y, &z)?;

        // Use the primary regression if available
        if let Some((a, b, _c)) = regression_result.0 {
            // z = ax + by + c
            let normal = Vector3::new(a, b, 1.0); // Normal vector for the plane
            let centroid = Point3::new(
                x.iter().copied().sum::<f32>() / x.len() as f32,
                y.iter().copied().sum::<f32>() / y.len() as f32,
                z.iter().copied().sum::<f32>() / z.len() as f32,
            );
            return Some(plane_to_isometry(normal, centroid));
        }

        // Use the first alternative regression if primary is degenerate
        if let Some((d, e, _f)) = regression_result.1 {
            // x = dy + ez + f
            let normal = Vector3::new(1.0, d, e);
            let centroid = Point3::new(
                x.iter().copied().sum::<f32>() / x.len() as f32,
                y.iter().copied().sum::<f32>() / y.len() as f32,
                z.iter().copied().sum::<f32>() / z.len() as f32,
            );
            return Some(plane_to_isometry(normal, centroid));
        }

        // Use the second alternative regression if both primary and first alternative are degenerate
        if let Some((g, h, _q)) = regression_result.2 {
            // y = gx + hz + q
            let normal = Vector3::new(g, 1.0, h);
            let centroid = Point3::new(
                x.iter().copied().sum::<f32>() / x.len() as f32,
                y.iter().copied().sum::<f32>() / y.len() as f32,
                z.iter().copied().sum::<f32>() / z.len() as f32,
            );
            return Some(plane_to_isometry(normal, centroid));
        }
        
        println!("No regression could be done for {:?}", points);

        None
    }

    /// Projects a point and performs planar regression to compute the normal.
    pub fn project(&self, mesh: &TriMesh, point: &ParryPoint<f32>, direction: i32) -> Option<Isometry3<f32>> {
        // Step 1: Project the central point
        let central_point = Self::project_point(mesh, point, direction)?;
        println!("Central point projection: {:?}", central_point);

        // Step 2: Generate points on a circle in the XZ plane
        let mut points: Vec<ParryPoint<f32>> = vec![];
        let mut valid_points: Vec<Point3<f32>> = vec![
            Point3::new(central_point.x, central_point.y, central_point.z), // Include the central point
        ];

        for i in 0..self.check_points {
            let angle = 2.0 * PI * (i as f32) / (self.check_points as f32);
            let circle_point = ParryPoint::new(
                point.x,                
                point.y + self.radius * angle.cos(),
                point.z + self.radius * angle.sin(),
            );

            if let Some(intersection) = Self::project_point(mesh, &circle_point, direction) {
                println!("Intersection at angle {}: {:?}", angle.to_degrees(), intersection);
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
        
        Self::compute_plane_isometry(&*valid_points)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use parry3d::shape::TriMesh;

    const HE: f32 = 1.5;
    const HM: f32 = -1.5;

    // The cube spans -1 to 1 with 0 in the center
    fn create_test_cube() -> TriMesh {
        // Create an axis-aligned cube as a TriMesh
        let vertices = vec![
            ParryPoint::new(HM, HM, HM),
            ParryPoint::new(HE, HM, HM),
            ParryPoint::new(HE, HE, HM),
            ParryPoint::new(HM, HE, HM),
            ParryPoint::new(HM, HM, HE),
            ParryPoint::new(HE, HM, HE),
            ParryPoint::new(HE, HE, HE),
            ParryPoint::new(HM, HE, HE),
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
    fn create_test_points() -> Vec<ParryPoint<f32>> {
        vec![
            ParryPoint::new(90.1, -0.5, -0.51),
            ParryPoint::new(90.2, -0.5, 0.51),
            ParryPoint::new(90.3, 0.5, -0.51),
            ParryPoint::new(90.4, 0.5, 0.51),
        ]
    }

    fn create_expected_results(x: f32) -> Vec<ParryPoint<f32>> {
        vec![
            // X value must come from the cube. We approach from + infity side.
            ParryPoint::new(x, -0.5, -0.51),
            ParryPoint::new(x, -0.5, 0.51),
            ParryPoint::new(x, 0.5, -0.51),
            ParryPoint::new(x, 0.5, 0.51),
        ]
    }

    const TOLERANCE: f32 = 0.002;

    // Helper function to compare two points with tolerance.
    fn points_are_close(a: &ParryPoint<f32>, b: &ParryPoint<f32>, tol: f32) -> bool {
        (a.x - b.x).abs() <= tol && (a.y - b.y).abs() <= tol && (a.z - b.z).abs() <= tol
    }

    /// Find the angle between the orientation of an Isometry3 and the global X-axis.
    fn angle_with_x_axis(isometry: &Isometry3<f32>) -> f32 {
        // Define the global X-axis
        let x_axis = Vector3::x();
        println!("Global X-axis: {:?}", x_axis);

        // Apply the rotation of the isometry to the X-axis
        let rotated_x = isometry.rotation.transform_vector(&x_axis);
        println!("Rotated X-axis: {:?}", rotated_x);

        // Compute the dot product of the rotated X-axis and the global X-axis
        let dot_product = rotated_x.dot(&x_axis);
        println!("Dot product with global X-axis: {:?}", dot_product);

        // Clamp the dot product to avoid floating-point precision issues
        let clamped_dot = dot_product.clamp(-1.0, 1.0);
        println!("Clamped dot product: {:?}", clamped_dot);

        // Compute the angle in degrees
        let angle = clamped_dot.acos().to_degrees();
        println!("Computed angle: {:?}", angle);

        angle
    }

    fn assert_normal(expected_normal: Vector3<f32>, description: &str, isometry: Isometry3<f32>) {
        // Check Z-axis alignment (normal)
        let computed_z_axis = isometry.rotation * Vector3::z();
        let normalized_expected_normal = expected_normal.normalize();
        assert!(
            (computed_z_axis - normalized_expected_normal).norm() <= 0.02,
            "Rotation mismatch for {}: got Z-axis {:?}, expected {:?}",
            description,
            computed_z_axis,
            normalized_expected_normal,            
        );

        // Additional orthonormality check
        let computed_x_axis = isometry.rotation * Vector3::x();
        let computed_y_axis = isometry.rotation * Vector3::y();
        assert!(
            (computed_x_axis.dot(&computed_y_axis)).abs() <= 0.01,
            "Rotation matrix is not orthonormal for {}: X-axis and Y-axis are not orthogonal.",
            description
        );
        assert!(
            (computed_z_axis.dot(&computed_x_axis)).abs() <= 0.01,
            "Rotation matrix is not orthonormal for {}: Z-axis and X-axis are not orthogonal.",
            description
        );
    }   

    #[test]
    fn test_project_points_into_cube() {
        let cube = create_test_cube();
        let test_points = create_test_points();
        let expected_results = create_expected_results(HE);

        for i in 0..test_points.len() {
            let test_point = &test_points[i];
            let expected = &expected_results[i];
            let result = Projector::project_point(&cube, test_point, Projector::FROM_POSITIVE);

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
    fn test_project_points_into_cube_neg() {
        let cube = create_test_cube();
        let test_points = create_test_points();
        let expected_results = create_expected_results(HM);

        for i in 0..test_points.len() {
            let test_point = &test_points[i];
            let expected = &expected_results[i];
            let result = Projector::project_point(&cube, test_point, Projector::FROM_NEGATIVE);

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
    fn test_project_circle_into_aac_pos() {
        use nalgebra::Vector3;

        let cube = create_test_cube();
        let test_points = create_test_points();
        let expected_results = create_expected_results(HE);

        let projector = Projector {
            check_points: 8,
            radius: 0.05,
        };

        for (test_point, expected) in test_points.iter().zip(expected_results.iter()) {
            let result = projector.project(&cube, test_point, Projector::FROM_POSITIVE);
            assert!(
                result.is_some(),
                "Projection failed for point {:?}",
                test_point
            );

            let isometry = result.unwrap();
            let projected_point = ParryPoint::new(
                isometry.translation.x,
                isometry.translation.y,
                isometry.translation.z,
            );

            assert!(points_are_close(&projected_point, expected, 0.001),
                    "Projection mismatch for point {:?}",
                    test_point
            );
            assert_normal(Vector3::new(1.0, 0.0, 0.0), "Cube projection", isometry);
        }
    }

    #[test]
    fn test_project_circle_into_aac_neg() {
        use nalgebra::Vector3;

        let cube = create_test_cube();
        let test_points = create_test_points();
        let expected_results = create_expected_results(HM);

        let projector = Projector {
            check_points: 8,
            radius: 0.05,
        };

        for (test_point, expected) in test_points.iter().zip(expected_results.iter()) {
            // Fire in the opposite direction
            let result = projector.project(&cube, test_point, Projector::FROM_NEGATIVE);
            assert!(
                result.is_some(),
                "Projection failed for point {:?}",
                test_point
            );

            let isometry = result.unwrap();
            let projected_point = ParryPoint::new(
                isometry.translation.x,
                isometry.translation.y,
                isometry.translation.z,
            );

            assert_eq!(
                projected_point, *expected,
                "Projection mismatch for point {:?}",
                test_point
            );
            assert_normal(Vector3::new(1.0, 0.0, 0.0), "Cube projection", isometry);

        }
    }

    #[cfg(test)]
    mod normals {
        use nalgebra::{Point3, Vector3};

        /// Helper function to test the plane_to_isometry function
        fn test_plane_to_isometry(normal: Vector3<f32>, centroid: Point3<f32>, description: &str) {
            let isometry = super::plane_to_isometry(normal, centroid);

            // Check translation
            let translation = isometry.translation.vector;
            assert!(
                (translation - centroid.coords).norm() <= 1e-5,
                "Translation mismatch for {}: expected {:?}, got {:?}",
                description,
                centroid.coords,
                translation
            );

            // Check Z-axis alignment (normal)
            let computed_z_axis = isometry.rotation * Vector3::z();
            let normalized_normal = normal.normalize();
            assert!(
                (computed_z_axis - normalized_normal).norm() <= 1e-5,
                "Rotation mismatch for {}: expected Z-axis {:?}, got {:?}",
                description,
                normalized_normal,
                computed_z_axis
            );

            // Additional orthonormality check
            let computed_x_axis = isometry.rotation * Vector3::x();
            let computed_y_axis = isometry.rotation * Vector3::y();
            assert!(
                (computed_x_axis.dot(&computed_y_axis)).abs() <= 1e-5,
                "Rotation matrix is not orthonormal for {}: X-axis and Y-axis are not orthogonal.",
                description
            );
            assert!(
                (computed_z_axis.dot(&computed_x_axis)).abs() <= 1e-5,
                "Rotation matrix is not orthonormal for {}: Z-axis and X-axis are not orthogonal.",
                description
            );
        }

        #[test]
        fn test_xy_plane_to_isometry() {
            // Normal to the XY plane, centroid at origin
            test_plane_to_isometry(
                Vector3::new(0.0, 0.0, 1.0), // Normal
                Point3::new(0.0, 0.0, 0.0),  // Centroid
                "XY plane",
            );
        }

        #[test]
        fn test_xz_plane_to_isometry() {
            // Normal to the XZ plane, centroid at origin
            test_plane_to_isometry(
                Vector3::new(0.0, 1.0, 0.0), // Normal
                Point3::new(0.0, 0.0, 0.0),  // Centroid
                "XZ plane",
            );
        }

        #[test]
        fn test_yz_plane_to_isometry() {
            // Normal to the YZ plane, centroid at origin
            test_plane_to_isometry(
                Vector3::new(1.0, 0.0, 0.0), // Normal
                Point3::new(0.0, 0.0, 0.0),  // Centroid
                "YZ plane",
            );
        }

        #[test]
        fn test_inclined_plane_to_isometry() {
            // Normal to the inclined plane z = x, centroid at (0.5, 0.0, 0.5)
            test_plane_to_isometry(
                Vector3::new(1.0 / 2_f32.sqrt(), 0.0, -1.0 / 2_f32.sqrt()), // Normalized [1, 0, -1]
                Point3::new(0.5, 0.0, 0.5), // Centroid
                "Inclined plane (z = x)",
            );
        }
    }

    #[cfg(test)]
    mod compute_plane {
        use crate::projector::Projector;
        use nalgebra::{Point3, Vector3};
        use crate::projector::tests::{angle_with_x_axis, assert_normal};

        /// Helper function to test compute_plane_isometry with a set of points
        fn test_compute_plane_isometry(
            points: &[Point3<f32>],
            expected_normal: Vector3<f32>,
            description: &str,
        ) {
            let isometry = Projector::compute_plane_isometry(points)
                .expect("Failed to compute plane isometry");

            println!("Axis X angle {}", angle_with_x_axis(&isometry));

            // Compute the centroid from points
            let centroid = points
                .iter()
                .fold(Vector3::zeros(), |acc, p| acc + p.coords)
                / points.len() as f32;

            // Check translation
            let translation = isometry.translation.vector;
            assert!(
                (translation - centroid).norm() <= 1e-5,
                "Translation mismatch for {}: expected {:?}, got {:?}",
                description,
                centroid,
                translation
            );

            assert_normal(expected_normal, description, isometry);
        }

        #[test]
        fn test_i_xy() {
            // Test 1: Points on the XY plane (z = 0)
            test_compute_plane_isometry(
                &[
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                    Point3::new(1.0, 1.0, 0.0),
                ],
                Vector3::new(0.0, 0.0, 1.0), // Normal to the XY plane
                "XY plane",
            );
        }

        #[test]
        fn test_i_xz() {
            // Test 2: Points on the XZ plane (y = 0)
            test_compute_plane_isometry(
                &[
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(0.0, 0.0, 1.0),
                    Point3::new(1.0, 0.0, 1.0),
                ],
                Vector3::new(0.0, 1.0, 0.0), // Normal to the XZ plane
                "XZ plane",
            );
        }

        #[test]
        fn test_i_45() {
            // Test 4: Points on an inclined plane z = x
            test_compute_plane_isometry(
                &[
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 1.0),
                    Point3::new(0.5, 0.5, 0.5),
                    Point3::new(1.0, 1.0, 1.0),
                ],
                Vector3::new(1.0 / 2_f32.sqrt(), 0.0, 1.0 / 2_f32.sqrt()), // Normalized [1, 0, -1]
                "Inclined plane (z = x)",
            );
        }

        #[test]
        fn test_i_45_neg_x() {
            // Points on an inclined plane z = -x
            test_compute_plane_isometry(
                &[
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(-1.0, 0.0, 1.0),
                    Point3::new(-0.5, 0.5, 0.5),
                    Point3::new(-1.0, 1.0, 1.0),
                ],
                Vector3::new(-1.0 / 2_f32.sqrt(), 0.0, 1.0 / 2_f32.sqrt()), // Normalized [-1, 0, -1]
                "Inclined plane (z = -x)",
            );
        }


        #[test]
        fn test_i_inclined_45_y() {
            // Points on an inclined plane z = y
            test_compute_plane_isometry(
                &[
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(0.0, 1.0, 1.0),
                    Point3::new(0.5, 0.5, 0.5),
                    Point3::new(1.0, 1.0, 1.0),
                ],
                Vector3::new(0.0, 1.0 / 2_f32.sqrt(), 1.0 / 2_f32.sqrt()), // Normalized [0, 1, -1]
                "Inclined plane (z = y)",
            );
        }
        

        #[test]
        fn test_i_yz() {
            // Test 3: Points on the YZ plane (x = 0)
            test_compute_plane_isometry(
                &[
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                    Point3::new(0.0, 0.0, 1.0),
                    Point3::new(0.0, 1.0, 1.0),
                ],
                Vector3::new(1.0, 0.0, 0.0), // Normal to the YZ plane
                "YZ plane",
            );
        }

        #[test]
        fn test_i_45_all_directions() {
            // Points on an inclined plane z = x + y
            test_compute_plane_isometry(
                &[
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 1.0),
                    Point3::new(0.0, 1.0, 1.0),
                    Point3::new(1.0, 1.0, 2.0),
                ],
                Vector3::new(1.0 / 3_f32.sqrt(), 1.0 / 3_f32.sqrt(), 1.0 / 3_f32.sqrt()), // Normalized [1, 1, -1]
                "Inclined plane (z = x + y)",
            );
        }

        #[test]
        fn test_i_45_all_directions_opposite() {
            // Points on an inclined plane z = x - y
            test_compute_plane_isometry(
                &[
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 1.0),
                    Point3::new(0.0, 1.0, -1.0),
                    Point3::new(1.0, 1.0, 0.0),
                ],
                Vector3::new(1.0 / 2_f32.sqrt(), -1.0 / 2_f32.sqrt(), 1.0 / 2_f32.sqrt()), // Normalized [1, -1, -1]
                "Inclined plane (z = x - y)",
            );
        }

        #[test]
        fn test_i_30_all_directions() {
            // Points on an inclined plane z = x + y
            test_compute_plane_isometry(
                &[
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 1.0),
                    Point3::new(0.0, 1.0, 1.0),
                    Point3::new(1.0, 1.0, 2.0),
                ],
                Vector3::new(
                    1.0 / 3_f32.sqrt(),
                    1.0 / 3_f32.sqrt(),
                    1.0 / 3_f32.sqrt(),
                ),
                "Inclined plane (z = x + y, 30 degrees)",
            );
        }

        #[test]
        fn test_i_30_all_directions_opposite() {
            // Points on an inclined plane z = x - y
            test_compute_plane_isometry(
                &[
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 1.0),
                    Point3::new(0.0, 1.0, -1.0),
                    Point3::new(1.0, 1.0, 0.0),
                ],
                Vector3::new(
                    1.0 / 3_f32.sqrt(),
                    -1.0 / 3_f32.sqrt(),
                    1.0 / 3_f32.sqrt(),
                ), 
                "Inclined plane (z = x - y, 30 degrees)",
            );
        }
    }
}
