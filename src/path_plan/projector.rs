use nalgebra::{Isometry3, Point3, Quaternion, Unit, UnitQuaternion, Vector3};
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
    Cylinder,
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
            Axis::Cylinder => unreachable!(),
        }
    }

    /// Compute the ray's direction vector based on the axis and ray side.
    pub fn compute_ray_direction(self, ray_side: RayDirection) -> Vector3<f32> {
        match self {
            Axis::X => Vector3::new(-ray_side.to_sign(), 0.0, 0.0),
            Axis::Y => Vector3::new(0.0, -ray_side.to_sign(), 0.0),
            Axis::Z => Vector3::new(0.0, 0.0, -ray_side.to_sign()),
            Axis::Cylinder => unreachable!(),
        }
    }

    pub fn vector(&self) -> Vector3<f32> {
        match self {
            Axis::X => Vector3::x(),
            Axis::Y => Vector3::y(),
            Axis::Z => Vector3::z(),
            Axis::Cylinder => unreachable!(),
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
            Axis::Cylinder => panic!("Not implemented"),
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

        // Step 5: Use mesh.cast_ray to find the intersection
        if let Some(intersection) =
            mesh.cast_ray_and_get_normal(&Isometry3::identity(), &ray, FAR, true)
        {
            let intersection_point = ray_origin + ray_direction * intersection.time_of_impact;
            return Some(ParryPoint::from(intersection_point));
        }

        // If no intersection is found, return None
        None
    }

    /// Project without using Parry isometries
    /// (parry has problems for normals close to Y axis)
    pub fn project_point_no_iso(
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
                Axis::Cylinder => panic!("Not implemented"),
            }
        } else {
            None
        }
    }

    pub fn project_point_cylindric_with_axis(
        mesh: &TriMesh,
        point: &geo::Point<f32>,
        radius: f32,
        axis: Axis,
    ) -> Option<ParryPoint<f32>> {
        const FAR: f32 = 100.0; // Unit is assumed to be meters, enough for the robotic cell
        let theta = point.x();
        let (sin_theta, cos_theta) = theta.sin_cos(); 
        
        // Step 1: Convert cylindrical coordinates to Cartesian based on the axis.
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
            _ => {
                panic!("Unsupported axis for cylindrical projection");
            }
        };

        // Step 2: Create the ray origin from the cylindrical surface point
        let ray_origin = ParryPoint::new(x, y, z);

        // Step 3: Compute the ray direction (pointing inward to the cylinder's axis)
        let ray_direction = match axis {
            Axis::Z => Vector3::new(-x, -y, 0.0).normalize(), // Direction in XY toward Z axis
            Axis::X => Vector3::new(0.0, -y, -z).normalize(), // Direction in YZ toward X axis
            Axis::Y => Vector3::new(-x, 0.0, -z).normalize(), // Direction in XZ toward Y axis
            _ => {
                panic!("Unsupported axis for cylindrical projection");
            }
        };

        // Step 4: Create the ray
        let ray = Ray::new(ray_origin.into(), ray_direction);

        if let Some(intersection) =
            mesh.cast_ray(&Isometry3::identity(), &ray, FAR, true)
        {
            let intersection_point = ray_origin + ray_direction * intersection;
            return Some(intersection_point);
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

        // Step 2: Generate points on a circle in the XZ plane
        let mut points = Vec::with_capacity(self.check_points + 1);
        points.push(central_point);

        for i in 0..self.check_points {
            let angle = 2.0 * PI * (i as f32) / (self.check_points as f32);
            let circle_point = axis.axis_aligned_circle_point(&central_point, self.radius, angle);

            if let Some(intersection) = Self::project_point(mesh, &circle_point, direction, axis) {
                if false {
                    println!(
                        "Intersection at angle {}: {:?}",
                        angle.to_degrees(),
                        intersection
                    );
                }
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

        self.compute_plane_isometry(points)
    }

    /// Project computing normals our own way, do not use Parry.
    /// It works well on all 3 axes, while Parry has weakness on normal close to
    /// parallel to Y.
    pub fn project_no_iso(
        &self,
        mesh: &TriMesh,
        point: &ParryPoint<f32>,
        direction: RayDirection,
        axis: Axis,
    ) -> Option<Isometry3<f32>> {
        // Project the central point
        let central_point = Self::project_point_no_iso(mesh, point, direction, axis)?;
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
                Self::project_point_no_iso(mesh, &circle_point, direction, axis)
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

        self.compute_plane_isometry_no_iso(central_point, points, axis, direction)
    }

    /// Cylindrical project with cylinder around an arbitrary axis. This confirmed to work well
    /// except when normal is close to parallel to Y.
    pub fn project_cylindric_with_axis(
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
            Self::project_point_cylindric_with_axis(mesh, point, projection_radius, cylinder_axis)
        {
            //println!("Central point projection: {:?}", central_point);

            use rayon::prelude::*; // Import Rayon traits for parallel iteration

            // Generate points on a circle around the cylinder axis
            let intersection_points: Vec<_> = (0..self.check_points)
                .into_par_iter() // Convert to a parallel iterator
                .filter_map(|i| {
                    let angle = 2.0 * PI * (i as f32) / (self.check_points as f32);
                    let circle_point = circle_point(&point, self.radius, angle, projection_radius);
                    Self::project_point_cylindric_with_axis(
                        mesh,
                        &circle_point,
                        projection_radius,
                        cylinder_axis,
                    )
                })
                .collect(); 

            let mut valid_points = Vec::with_capacity(intersection_points.len() + 1); // Pre-allocate space
            valid_points.push(central_point); // Add the central point
            valid_points.extend(intersection_points); // Extend with the valid intersections

            // Ensure enough valid points exist for plane fitting
            if valid_points.len() < 3 {
                println!("Not enough valid points for regression.");
                return None;
            }

            self.compute_plane_isometry(valid_points)
        } else {
            //println!("Central point projection NONE");
            None
        }
    }

    pub fn project_cylindric_with_axis_sequential(
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
            Self::project_point_cylindric_with_axis(mesh, point, projection_radius, cylinder_axis)
        {
            //println!("Central point projection: {:?}", central_point);

            // Generate points on a circle around the cylinder axis
            let mut valid_points = Vec::with_capacity(self.check_points + 1);
            valid_points.push(central_point);

            for i in 0..self.check_points {
                let angle = 2.0 * PI * (i as f32) / (self.check_points as f32);
                let circle_point = circle_point(&point, self.radius, angle, projection_radius);

                // Use project_point_cylindric_with_axis for circle points
                if let Some(intersection) = Self::project_point_cylindric_with_axis(
                    mesh,
                    &circle_point,
                    projection_radius,
                    cylinder_axis,
                ) {
                    if false {
                        println!(
                            "Intersection at angle {}: {:?}",
                            angle.to_degrees(),
                            intersection
                        );
                    }
                    valid_points.push(intersection);
                } else {
                    println!("No intersection at angle {}", angle.to_degrees());
                }
            }

            // Ensure enough valid points exist for plane fitting
            if valid_points.len() < 3 {
                println!("Not enough valid points for regression.");
                return None;
            }

            self.compute_plane_isometry(valid_points)
            // self.compute_plane_isometry_averaging(central_point, valid_points, cylinder_axis)
        } else {
            //println!("Central point projection NONE");
            None
        }
    }

    fn average_plane_orientation_no_iso(
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
            Axis::Cylinder => Vector3::z(),
        } * direction.to_sign();

        // Check orientation consistency with the preferred vector
        if axis != Axis::Cylinder && average_normal.dot(&preferred_direction) < 0.0 {
            // Flip the normal if it's pointing away from the preferred direction
            average_normal = -average_normal;
        }

        let z_axis = Vector3::z(); //  Vector3::z(); //
        if axis == Axis::Z && direction == RayDirection::FromNegative {
            // Axis would be close to antiparallel to X axis, solutions are unstable here, need spec approach
            average_normal = -average_normal; // Make it instead close to parallel
            let q = UnitQuaternion::rotation_between(&z_axis, &average_normal);
            if let Some(q) = q {
                let swing_twist = Projector::decompose_swing_twist(q, &z_axis);
                // Flip 180 degrees
                return Some(Projector::twist_y(&swing_twist, PI));
            }
        }
        UnitQuaternion::rotation_between(&z_axis, &average_normal)
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

    pub fn XY_normalized_angle(normal: &Vector3<f32>) -> f32 {
        // Project the normal onto the XY plane.
        let projected_x = normal.x;
        let projected_y = normal.y;

        // Compute the angle (in radians) of the projection w.r.t. the positive X axis.
        let angle = projected_y.atan2(projected_x).to_degrees();

        // Normalize the angle to a range of 0 to 360 degrees.
        let normalized_angle = if angle < 0.0 { angle + 360.0 } else { angle };
        normalized_angle
    }

    pub fn XZ_normalized_angle(normal: &Vector3<f32>) -> f32 {
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
    pub fn find_most_perpendicular_axis_between_points(
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

        let mut plane_points = Vec::with_capacity(points.len());
        for point in points {
            plane_points.push(point.coords.into());
        }
        let avg_normal = Self::compute_normal_sum_parallel(&plane_points).normalize();
        let quaternion;

        #[derive(Debug, Copy, Clone, PartialEq, Eq)]
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
        let angle = Self::XY_normalized_angle(&avg_normal);

        let (strategy, flip) = if axis == Axis::Z {
            (
                match angle {
                    -1.0..=45.0 =>  Strategy::X, // Y, Z possible
                    45.0..=90.0 => Strategy::X,
                    90.0..=135.0 => Strategy::Z, 
                    135.0..=180.0 => Strategy::Z,
                    180.0..=225.0 => Strategy::Z,
                    225.0..=270.0 => Strategy::Y,
                    270.0..=315.0 => Strategy::X,  
                    315.0..=361.0 => Strategy::X, 
                    _ => unreachable!(),          // The angle is always in the range [0, 360)
                },
                false,
            )
        } else if Axis::Y == axis {
            (
                match angle {
                    -1.0..=45.0 => Strategy::X,
                    45.0..=90.0 => Strategy::X,

                    // This case we need to differentiate also by XZ angle
                    90.0..=180.0 => match Self::XZ_normalized_angle(&avg_normal) {
                        -1.0..=45.0 => Strategy::X,       // Unreachable?
                        45.0..=90.0 => Strategy::X,       // Unreachable?
                        90.0..=135.0 => Strategy::ZSpec,  // Problematic
                        135.0..=185.0 => Strategy::ZSpec, //Strategy::X, // THIS
                        180.0..=225.0 => Strategy::X,
                        225.0..=270.0 => Strategy::X,
                        270.0..=315.0 => Strategy::X,
                        315.0..=361.0 => Strategy::X,
                        _ => unreachable!(), // The angle is always in the range [0, 360)
                    },

                    180.0..=225.0 => Strategy::Z,

                    225.0..=250.0 => Strategy::Z,
                    250.0..=270.0 => Strategy::X,

                    270.0..=315.0 => Strategy::X,
                    315.0..=361.0 => Strategy::X,

                    _ => unreachable!(), // The angle is always in the range [0, 360)
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
                    270.0..=315.0 => Strategy::ZSpecTwistX, //Strategy::Z, // One edge still broken here
                    315.0..=361.0 => Strategy::ZSpecTwistX, // X and Y differently broken, X middle, Z and Y - negative Z zone
                    _ => unreachable!(), // The angle is always in the range [0, 360)
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
        // Minimum angular difference (in radians) to trigger alignment
        const ALIGNMENT_THRESHOLD: f64 = 0.01 * std::f64::consts::PI / 180.0;

        // Step 1: Get the current X-axis of the quaternion
        let current_x = quaternion * Vector3::x();

        // Step 2: Compute the angle between the current and target X-axes
        let dot_product = current_x.dot(&target_x).clamp(-1.0, 1.0);
        let angle_misalignment = dot_product.acos();

        // Step 3: Check if alignment is required
        if angle_misalignment < ALIGNMENT_THRESHOLD {
            // Already aligned within the threshold, return original quaternion
            return quaternion;
        }

        // Step 4: Compute the rotation needed to align the X-axis
        let rotation_axis = current_x.cross(&target_x).normalize();
        let alignment_quaternion = UnitQuaternion::from_axis_angle(
            &Unit::new_normalize(rotation_axis),
            angle_misalignment,
        );

        // Step 5: Apply the rotation to align the quaternion
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

    fn compute_plane_isometry_no_iso(
        &self,
        centroid: ParryPoint<f32>,
        points: Vec<Point3<f32>>,
        axis: Axis,
        direction: RayDirection,
    ) -> Option<Isometry3<f32>> {
        // Convert Point3<f32> to Vector3<f32> for average_plane_orientation
        let vectors: Vec<Vector3<f32>> = points.into_iter().map(|p| p.coords).collect();

        // Call average_plane_orientation to compute the plane's orientation
        let orientation = self.average_plane_orientation_no_iso(&vectors, axis, direction);
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

    // The points span -0.5 to 0.5. X value should not be accounted for
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
            Axis::Cylinder => panic!("Such test is not supported"),
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
