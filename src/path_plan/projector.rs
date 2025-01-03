use nalgebra::{Isometry3, Point3, Quaternion, Unit, UnitQuaternion, Vector3};
use parry3d::math::Point as ParryPoint;
use parry3d::query::{Ray, RayCast};
use parry3d::shape::TriMesh;
use std::f32::consts::PI;

pub struct Projector {
    pub check_points: usize,

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
    pub fn axis_aligned_circle_point(
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
    /// Project without using Parry isometries
    /// (parry has problems for normals close to Y axis)
    pub fn project_point_flat(
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
        axis: Axis,
    ) -> Option<ParryPoint<f32>> {
        const FAR: f32 = 100.0; // Unit is assumed to be meters, enough for the robotic cell
        let theta = point.x();
        let (sin_theta, cos_theta) = theta.sin_cos();

        // Convert cylindrical coordinates to Cartesian based on the axis.
        let (x, y, z) = match axis {
            Axis::Z => {
                let x = radius * cos_theta; // X coordinate on cylinder's surface
                let y = radius * sin_theta; // Y coordinate on cylinder's surface
                let z = point.y(); // Z remains unchanged
                (x, y, z)
            }
            Axis::X => {
                let y = radius * cos_theta; // Y coordinate on cylinder's surface
                let z = radius * sin_theta; // Z coordinate on the cylinder's surface
                let x = point.y(); // X remains unchanged
                (x, y, z)
            }
            Axis::Y => {
                let x = radius * cos_theta; // X coordinate on cylinder's surface
                let z = radius * sin_theta; // Z coordinate on the cylinder's surface
                let y = point.y(); // Y remains unchanged
                (x, y, z)
            }
        };

        let ray_origin = ParryPoint::new(x, y, z);
        let ray_direction = match axis {
            Axis::Z => Vector3::new(-x, -y, 0.0).normalize(), // Direction in XY toward Z axis
            Axis::X => Vector3::new(0.0, -y, -z).normalize(), // Direction in YZ toward X axis
            Axis::Y => Vector3::new(-x, 0.0, -z).normalize(), // Direction in XZ toward Y axis
        };
        let ray = Ray::new(ray_origin.into(), ray_direction);

        if let Some(toi) = mesh.cast_local_ray(&ray, FAR, true) {
            return Some(ray_origin + ray_direction * toi);
        }
        None
    }

    /// Project computing normals our own way, do not use Parry.
    /// It works well on all 3 axes, while Parry has weakness on normal close to
    /// parallel to Y.
    pub fn project_flat(
        &self,
        mesh: &TriMesh,
        point: &ParryPoint<f32>,
        direction: RayDirection,
        axis: Axis,
    ) -> Option<Isometry3<f32>> {
        // Project the central point
        let central_point = Self::project_point_flat(mesh, point, direction, axis)?;
        if false {
            println!("Central point projection: {:?}", central_point);
        }

        // Generate points on a circle in the XZ plane
        let mut points: Vec<ParryPoint<f32>> = Vec::with_capacity(self.check_points + 1);
        // Do not include the central point, may impact winding

        for i in 0..self.check_points {
            let angle = 2.0 * PI * (i as f32) / (self.check_points as f32);
            let circle_point = axis.axis_aligned_circle_point(&central_point, self.radius, angle);

            if let Some(intersection) =
                Self::project_point_flat(mesh, &circle_point, direction, axis)
            {
                points.push(intersection);
            } else {
                println!("No intersection at angle {}", angle.to_degrees());
            }
        }

        // Ensure enough valid points exist
        if points.len() < 3 {
            println!("Not enough valid points for regression.");
            return None;
        }

        self.compute_plane_isometry_flat(central_point, points, axis, direction)
    }

    /// Cylindrical project with cylinder around an arbitrary axis. This confirmed to work well
    /// except when normal is close to parallel to Y.
    pub fn project_cylindric(
        &self,
        mesh: &TriMesh,
        point: &geo::Point<f32>,
        projection_radius: f32,
        cylinder_axis: Axis,
    ) -> Option<Isometry3<f32>> {
        pub fn circle_point(
            central_point: &geo::Point<f32>, // Center of the circle in 3D
            radius: f32,                     // Circle radius
            theta: f32,                      // Angle (in radians) for parametric circle point
            projection_radius: f32,          // Diameter of the cylinder
        ) -> geo::Point<f32> {
            // Compute the meter_rad_factor
            let meter_rad_factor = projection_radius / 2.0;

            // Compute the Z coordinate (for the axis)
            let z = central_point.y() + radius * theta.sin();

            // Compute the X coordinate, representing angular displacement in meters
            let theta_x = central_point.x() + radius * theta.cos() * meter_rad_factor;

            // Return the 2D point in (theta_x, z)
            geo::Point::new(theta_x, z)
        }

        // Attempt to project the central point using the updated project_point_cylindric_with_axis.
        if let Some(central_point) =
            Self::project_point_cylindric(mesh, point, projection_radius, cylinder_axis)
        {
            use rayon::prelude::*; // Import Rayon traits for parallel iteration

            // Generate points on a circle around the cylinder axis
            let intersection_points = (0..self.check_points + 1)
                .into_par_iter() // Convert to a parallel iterator
                .filter_map(|i| {
                    if i == 0 {
                        // Central point is the first in the list
                        Some(central_point)
                    } else {
                        let angle = 2.0 * PI * ((i - 1) as f32) / (self.check_points as f32);
                        let circle_point =
                            circle_point(&point, self.radius, angle, projection_radius);
                        Self::project_point_cylindric(
                            mesh,
                            &circle_point,
                            projection_radius,
                            cylinder_axis,
                        )
                    }
                });

            let valid_points: Vec<ParryPoint<f32>> = intersection_points.collect();
            if valid_points.len() < 3 {
                println!("Not enough valid points for regression.");
                return None;
            }
            self.compute_plane_isometry(valid_points)
        } else {
            None
        }
    }

    fn average_plane_orientation_flat_axis(
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
        let preferred_direction = -match axis {
            Axis::X => Vector3::x(),
            Axis::Y => Vector3::y(),
            Axis::Z => Vector3::z(),
        } * direction.to_sign();

        // Check orientation consistency with the preferred vector
        if average_normal.dot(&preferred_direction) < 0.0 {
            // Flip the normal if it's pointing away from the preferred direction
            average_normal = -average_normal;
        }

        let z_axis = Vector3::z();
        if axis == Axis::Z && direction == RayDirection::FromNegative {
            // Axis would be close to antiparallel to X axis, solutions are unstable here, need spec approach
            average_normal = -average_normal; // Make it instead close to parallel
            let q = UnitQuaternion::rotation_between(&z_axis, &average_normal);
            if let Some(q) = q {
                let swing_twist = Projector::decompose_swing_twist(q, &z_axis);
                return Some(Projector::twist_y(&swing_twist, PI));
            }
        }
        if axis == Axis::Z && direction == RayDirection::FromPositive {
            let q = UnitQuaternion::rotation_between(&Vector3::y(), &average_normal);
            if let Some(q) = q {
                let swing_twist = Projector::decompose_swing_twist(q, &Vector3::x());
                return Some(Projector::twist_x(&swing_twist, PI));
            }
        }
        UnitQuaternion::rotation_between(&z_axis, &average_normal)
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
        quaternion: UnitQuaternion<f32>,
        axis: &Vector3<f32>,
    ) -> (UnitQuaternion<f32>, UnitQuaternion<f32>) {
        let dot = quaternion.i * axis.x + quaternion.j * axis.y + quaternion.k * axis.z;
        let twist = Quaternion::new(quaternion.w, axis.x * dot, axis.y * dot, axis.z * dot);

        let twist = UnitQuaternion::from_quaternion(twist);
        let swing = quaternion * twist.inverse();

        (swing, twist)
    }

    /// Sets the twist of a quaternion to a fixed angle (in radians) around a specified axis.
    fn twist_y(
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

    fn twist_x(
        decomposition: &(UnitQuaternion<f32>, UnitQuaternion<f32>),
        fixed_angle: f32,
    ) -> UnitQuaternion<f32> {
        // Decompose the quaternion into swing and twist
        let axis = Vector3::x_axis();
        let (swing, _twist) = decomposition;
        let fixed_twist = UnitQuaternion::from_axis_angle(&axis, fixed_angle);

        // Combine the swing with the new twist
        swing * fixed_twist
    }

    fn xy_normalized_angle(normal: &Vector3<f32>) -> f32 {
        // Project the normal onto the XY plane.
        let projected_x = normal.x;
        let projected_y = normal.y;

        // Compute the angle (in radians) of the projection w.r.t. the positive X axis.
        let angle = projected_y.atan2(projected_x).to_degrees();

        // Normalize the angle to a range of 0 to 360 degrees.
        let normalized_angle = if angle < 0.0 { angle + 360.0 } else { angle };
        normalized_angle
    }

    fn xz_normalized_angle(normal: &Vector3<f32>) -> f32 {
        // Project the normal onto the XZ plane
        let projected_x = normal.x;
        let projected_z = normal.z;

        // Compute the angle (in radians) of the projection w.r.t. the positive X axis
        let angle = projected_z.atan2(projected_x).to_degrees();

        // Normalize the angle to a range of 0 to 360 degrees
        let normalized_angle = if angle < 0.0 { angle + 360.0 } else { angle };
        normalized_angle
    }

    /// Determines which axis the vector connecting two points is most perpendicular to.
    fn find_most_perpendicular_axis_between_points(
        point1: &ParryPoint<f32>,
        point2: &ParryPoint<f32>,
    ) -> Axis {
        // Compute the vector connecting the two points
        let vector = point2 - point1;

        // Basis vectors for the X, Y, and Z axes
        let x_axis = Vector3::new(1.0, 0.0, 0.0);
        let y_axis = Vector3::new(0.0, 1.0, 0.0);
        let z_axis = Vector3::new(0.0, 0.0, 1.0);

        // Compute the absolute values of the dot products with the connecting vector
        let dot_x = vector.dot(&x_axis).abs();
        let dot_y = vector.dot(&y_axis).abs();
        let dot_z = vector.dot(&z_axis).abs();

        // Compare the dot products and return the axis with the smallest absolute value
        if dot_x <= dot_y && dot_x <= dot_z {
            Axis::X
        } else if dot_y <= dot_x && dot_y <= dot_z {
            Axis::Y
        } else {
            Axis::Z
        }
    }

    fn compute_plane_isometry(
        &self,
        points: Vec<ParryPoint<f32>>, // First point is center point, others are arround it in
    ) -> Option<Isometry3<f32>> {
        if points.len() < self.check_points {
            return None;
        }
        let from = points[0]; // centroid.0;
        let to = points[1];

        let axis = Self::find_most_perpendicular_axis_between_points(&from, &to);

        let plane_points: Vec<_> = points.iter().map(|p| p.coords).collect();
        let avg_normal = Self::compute_normal_sum_parallel(&plane_points).normalize();
        let quaternion;

        #[derive(Debug, Copy, Clone, PartialEq, Eq)]
        #[allow(dead_code)]
        enum Strategy {
            X,
            Y,
            Z,
            OFF,
            ZSpec,
            ZSpecTwistX,
        }
        // Determine the octant based on the angle and use the approach that is the most
        // robust there
        let angle = Self::xy_normalized_angle(&avg_normal);

        let (strategy, flip) = if axis == Axis::Z {
            (
                match angle {
                    -1.0..=45.0 => Strategy::X, // Y, Z possible
                    45.0..=90.0 => Strategy::X,
                    90.0..=135.0 => Strategy::Z,
                    135.0..=180.0 => Strategy::Z,
                    180.0..=225.0 => Strategy::Z,
                    225.0..=270.0 => Strategy::Y,
                    270.0..=315.0 => Strategy::X,
                    315.0..=361.0 => Strategy::X,
                    _ => unreachable!(),
                },
                false,
            )
        } else if Axis::Y == axis {
            (
                match angle {
                    -1.0..=45.0 => Strategy::X,
                    45.0..=90.0 => Strategy::X,

                    // This case we need to differentiate also by XZ angle
                    90.0..=180.0 => match Self::xz_normalized_angle(&avg_normal) {
                        -1.0..=45.0 => Strategy::X,
                        45.0..=90.0 => Strategy::X,
                        90.0..=135.0 => Strategy::ZSpec,
                        135.0..=185.0 => Strategy::ZSpec,
                        180.0..=225.0 => Strategy::X,
                        225.0..=270.0 => Strategy::X,
                        270.0..=315.0 => Strategy::X,
                        315.0..=361.0 => Strategy::X,
                        _ => unreachable!(),
                    },

                    180.0..=225.0 => Strategy::Z,

                    225.0..=250.0 => Strategy::Z,
                    250.0..=270.0 => Strategy::X,

                    270.0..=315.0 => Strategy::X,
                    315.0..=361.0 => Strategy::X,

                    _ => unreachable!(),
                },
                true,
            )
        } else if Axis::X == axis {
            (
                match angle {
                    -1.0..=45.0 => Strategy::Y,
                    45.0..=90.0 => Strategy::Y,
                    90.0..=135.0 => Strategy::Y,
                    135.0..=180.0 => Strategy::Y,
                    180.0..=225.0 => Strategy::Z,
                    225.0..=270.0 => Strategy::Z,
                    270.0..=315.0 => Strategy::ZSpecTwistX,
                    315.0..=361.0 => Strategy::ZSpecTwistX,
                    _ => unreachable!(),
                },
                false,
            )
        } else {
            unreachable!()
        };

        // https://www.onlogic.com/store/hx310/
        // https://www.onlogic.com/store/hx511/
        quaternion = match strategy {
            // Solutions have broken regions so we must bridge over
            Strategy::X => {
                let r_axis = Vector3::x();
                if let Some(twisted) = UnitQuaternion::rotation_between(&r_axis, &avg_normal) {
                    // Rotate 270 degrees around Y to untwist
                    let decomposition = Projector::decompose_swing_twist(twisted, &r_axis);
                    let twist_by = if flip { PI / 2.0 } else { 3.0 * PI / 2.0 };
                    Some(Projector::twist_y(&decomposition, twist_by))
                } else {
                    None
                }
            }
            Strategy::Y => {
                let r_axis = Vector3::y();
                if let Some(twisted) = UnitQuaternion::rotation_between(&r_axis, &avg_normal) {
                    // Rotate 270 degrees around Y to untwist
                    let decomposition = Projector::decompose_swing_twist(twisted, &r_axis);
                    let twist_by = if flip { 3.0 * PI / 2.0 } else { PI / 2.0 };
                    Some(Projector::twist_x(&decomposition, twist_by))
                } else {
                    None
                }
            }
            Strategy::ZSpec => {
                let r_axis = Vector3::z();
                if let Some(twisted) = UnitQuaternion::rotation_between(&r_axis, &avg_normal) {
                    Some(twisted)
                } else {
                    None
                }
            }
            Strategy::ZSpecTwistX => {
                let r_axis = Vector3::z();
                if let Some(twisted) = UnitQuaternion::rotation_between(&r_axis, &avg_normal) {
                    let decomposition = Projector::decompose_swing_twist(twisted, &r_axis);
                    let twist_by = PI;
                    Some(Projector::twist_x(&decomposition, twist_by))
                } else {
                    None
                }
            }
            Strategy::Z => {
                let r_axis = Vector3::z();
                if let Some(twisted) = UnitQuaternion::rotation_between(&r_axis, &avg_normal) {
                    // Rotate 270 degrees around Y to untwist
                    let decomposition = Projector::decompose_swing_twist(twisted, &r_axis);
                    let twist_by = if flip { 0. } else { PI };
                    Some(Projector::twist_y(&decomposition, twist_by))
                } else {
                    None
                }
            }
            Strategy::OFF => None,
        };

        if let Some(quaternion) = quaternion {
            let quaternion = Self::align_quaternion_x_to_points(quaternion, from, to);

            // Combine the rotation with the translation (centroid) into an Isometry3
            Some(Isometry3::from_parts(from.coords.into(), quaternion))
        } else {
            None
        }
    }

    /// Align a quaternion's X-axis to the given target X-axis with a precise alignment check
    fn _align_quaternion_x_to_points(
        quaternion: UnitQuaternion<f64>,
        target_x: &Vector3<f64>,
    ) -> UnitQuaternion<f64> {
        const ALIGNMENT_THRESHOLD: f64 = 0.01 * std::f64::consts::PI / 180.0;
        let current_x = quaternion * Vector3::x();
        let dot_product = current_x.dot(&target_x).clamp(-1.0, 1.0);
        let angle_misalignment = dot_product.acos();
        if angle_misalignment < ALIGNMENT_THRESHOLD {
            return quaternion;
        }
        let rotation_axis = current_x.cross(&target_x);
        let alignment_quaternion = UnitQuaternion::from_axis_angle(
            &Unit::new_normalize(rotation_axis),
            angle_misalignment,
        );
        alignment_quaternion * quaternion
    }

    fn align_quaternion_x_to_points(
        quaternion: UnitQuaternion<f32>,
        p1: Point3<f32>,
        p2: Point3<f32>,
    ) -> UnitQuaternion<f32> {
        let mut q = quaternion.cast::<f64>();
        let w1 = Point3::new(p1.x as f64, p1.y as f64, p1.z as f64);
        let w2 = Point3::new(p2.x as f64, p2.y as f64, p2.z as f64);
        let vector = Vector3::new(w2.x - w1.x, w2.y - w1.y, w2.z - w1.z).normalize();

        for _i in 0..3 {
            q = Self::_align_quaternion_x_to_points(q, &vector);
        }
        q.cast::<f32>()
    }

    fn compute_plane_isometry_flat(
        &self,
        centroid: ParryPoint<f32>,
        points: Vec<Point3<f32>>,
        axis: Axis,
        direction: RayDirection,
    ) -> Option<Isometry3<f32>> {
        let from = centroid;
        let to = points[0];
        let vectors: Vec<Vector3<f32>> = points.into_iter().map(|p| p.coords).collect();

        // Call average_plane_orientation to compute the plane's orientation
        let orientation = self.average_plane_orientation_flat_axis(&vectors, axis, direction);
        if let Some(orientation) = orientation {
            let orientation = Projector::align_quaternion_x_to_points(orientation, from, to);
            Some(Isometry3::from_parts(centroid.coords.into(), orientation))
        } else {
            None
        }
    }
}
