use nalgebra::{Isometry3, Point3, Quaternion, UnitQuaternion, Vector3};
use parry3d::math::Point as ParryPoint;
use parry3d::query::{Ray, RayCast};
use parry3d::shape::TriMesh;
use std::f32::consts::PI;

pub struct Projector {
    pub check_points: usize,

    // If true, normals point inward (as needed for the robot to orient the tool).
    pub normals_inward: bool,

    // Check cylinder radius for finding normals
    pub radius: f32,
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

    pub fn project_point_cylindric(
        mesh: &TriMesh,
        point: &geo::Point<f32>,
        radius: f32,
    ) -> Option<ParryPoint<f32>> {
        const FAR: f32 = 100.0; // Unit is assumed to be meters, enough for the robotic cell

        use parry3d::query::Ray;
        use std::f32::consts::PI;

        // Step 1: Convert cylindrical coordinates to Cartesian
        let theta = point.x();
        let x = radius * theta.cos(); // X coordinate on cylinder's surface
        let y = radius * theta.sin(); // Y coordinate on cylinder's surface
        let z = point.y(); // Z remains unchanged

        // Step 2: Create the ray origin from the cylindrical surface point
        let ray_origin = ParryPoint::new(x, y, z);

        // Step 3: Compute the ray direction (parallel to XY toward Z axis)
        // Pointing inward from the cylinder's surface
        let ray_direction = Vector3::new(-x, -y, 0.0).normalize(); // Normalized direction in XY plane

        // Step 4: Create the ray
        let ray = Ray::new(ray_origin.into(), ray_direction);

        // Step 5: Use mesh.cast_ray to find the intersection
        if let Some(toi) = mesh.cast_ray(&Isometry3::identity(), &ray, FAR, true) {
            let intersection_point = ray_origin + ray_direction * toi;
            return Some(ParryPoint::from(intersection_point));
        }

        // If no intersection is found, return None
        None
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
        if false {
            println!("Central point projection: {:?}", central_point);
        }

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
        if valid_points.len() < 3 {
            println!("Not enough valid points for regression.");
            return None;
        }

        self.compute_plane_isometry(central_point, valid_points, axis, direction)
    }

    pub fn project_cylindric(
        &self,
        mesh: &TriMesh,
        point: &geo::Point<f32>,
        projection_radius: f32,
    ) -> Option<Isometry3<f32>> {
        // Attempt to project the central point.
        if let Some(central_point) = Self::project_point_cylindric(mesh, point, projection_radius) {
            // Log the central point for debugging.
            println!("Central point projection: {:?}", central_point);

            // Create and return an Isometry3 translation using the projected point.
            Some(Isometry3::translation(
                central_point.x,
                central_point.y,
                central_point.z,
            ))
        } else {
            println!("Central point projection NONE");
            None
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
        &self,
        points: &[Vector3<f32>],
        axis: Axis,
        direction: RayDirection,
    ) -> Option<UnitQuaternion<f32>> {
        if points.len() < 3 {
            return None;
        }

        // Accumulate normals
        let normal_sum = Self::compute_normal_sum_parallel(points);

        // Average the normals
        let mut average_normal = normal_sum.normalize();

        // Ensure the normal vector is valid
        if average_normal.norm() == 0.0 {
            return None;
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

        let x_axis = Vector3::z();
        if axis == Axis::Z && direction == RayDirection::FromNegative {
            // Axis would be close to antiparallel to X axis, solutions are unstable here, need spec approach
            average_normal = -average_normal; // Make it instead close to parallel
            let q = UnitQuaternion::rotation_between(&x_axis, &average_normal);
            if let Some(q) = q {
                let swing_twist = self.decompose_swing_twist(q, &x_axis);
                // Flip 180 degrees
                return Some(self.set_twist_y(&swing_twist, PI));
            }
        }
        UnitQuaternion::rotation_between(&x_axis, &average_normal)
    }

    #[allow(dead_code)]
    fn compute_normal_sum_sequential(points: &[Vector3<f32>]) -> Vector3<f32> {
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
        normal_sum
    }

    fn compute_normal_sum_parallel(points: &[Vector3<f32>]) -> Vector3<f32> {
        use nalgebra::Vector3;
        use rayon::iter::{IndexedParallelIterator, ParallelIterator};
        use rayon::prelude::*; // Includes common Rayon traits

        // Use Rayon parallel iterator with a thread-safe accumulation
        let normal_sum = points
            .par_iter() // Parallel iteration over the first loop
            .enumerate()
            .map(|(i, &point_i)| {
                let mut local_sum = Vector3::zeros();

                for j in (i + 1)..points.len() {
                    for k in (j + 1)..points.len() {
                        // Compute vectors on the plane
                        let v1 = points[j] - point_i;
                        let v2 = points[k] - point_i;

                        // Compute normal of the triangle
                        let normal = v1.cross(&v2);

                        // Accumulate normals (ignore magnitude)
                        if normal.norm() > 0.0 {
                            local_sum += normal.normalize();
                        }
                    }
                }

                local_sum // Return this thread's local sum
            })
            .reduce(|| Vector3::zeros(), |acc, local| acc + local);

        normal_sum // Return the accumulated vector
    }

    /// Decomposes a quaternion into its swing and twist components around a specified axis.
    fn decompose_swing_twist(
        &self,
        quaternion: UnitQuaternion<f32>,
        axis: &Vector3<f32>,
    ) -> (UnitQuaternion<f32>, UnitQuaternion<f32>) {
        let axis = axis.normalize(); // Ensure the axis is normalized
        let dot = quaternion.i * axis.x + quaternion.j * axis.y + quaternion.k * axis.z;

        let twist =
            Quaternion::new(quaternion.w, axis.x * dot, axis.y * dot, axis.z * dot).normalize();

        let twist = UnitQuaternion::from_quaternion(twist);
        let swing = quaternion * twist.inverse();

        (swing, twist)
    }

    /// Sets the twist of a quaternion to a fixed angle (in radians) around a specified axis.
    fn set_twist_y(
        &self,
        decomposition: &(UnitQuaternion<f32>, UnitQuaternion<f32>),
        fixed_angle: f32,
    ) -> UnitQuaternion<f32> {
        // Decompose the quaternion into swing and twist
        let axis = Vector3::y_axis();
        let (swing, _twist) = decomposition;
        let fixed_twist = UnitQuaternion::from_axis_angle(&axis, fixed_angle);

        // Combine the swing with the new twist
        swing * fixed_twist
    }

    fn compute_plane_isometry(
        &self,
        centroid: ParryPoint<f32>,
        points: Vec<Point3<f32>>,
        axis: Axis,
        direction: RayDirection,
    ) -> Option<Isometry3<f32>> {
        // Convert Point3<f32> to Vector3<f32> for average_plane_orientation
        let vectors: Vec<Vector3<f32>> = points.into_iter().map(|p| p.coords).collect();

        // Call average_plane_orientation to compute the plane's orientation
        let orientation = self.average_plane_orientation(&vectors, axis, direction);
        if orientation.is_none() {
            return None;
        }

        // Combine the rotation with the translation (centroid) into an Isometry3
        Some(Isometry3::from_parts(
            centroid.coords.into(),
            if self.normals_inward {
                orientation.unwrap().inverse()
            } else {
                orientation.unwrap()
            },
        ))
    }
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
