use nalgebra::{Isometry3, Vector3};
use parry3d::bounding_volume::BoundingVolume;
use parry3d::shape::{ConvexPolyhedron, TriMesh};
use std::time::Instant;

pub struct HeadLifter<'a> {
    expected_max_distance: f32,
    safety_distance: f32,
    object_mesh: &'a TriMesh,
    toolhead_mesh: &'a TriMesh,
    toolhead_qhull: ConvexPolyhedron, 
    toolhead_aabb_mesh: TriMesh,    
    tolerance: f32, //  = 0.002 // Precision threshold
    debug: bool,
}

impl<'a> HeadLifter<'a> {
    pub fn new(
        object_mesh: &'a TriMesh,
        toolhead_mesh: &'a TriMesh,
        safety_distance: f32,
        expected_max_distance: f32,
        tolerance: f32,
    ) -> Self {
        let toolhead_aabb = toolhead_mesh.local_aabb().loosened(safety_distance);
        let toolhead_aabb_mesh = crate::collisions::build_trimesh_from_aabb(toolhead_aabb);
        let toolhead_qhull = ConvexPolyhedron::from_convex_hull(toolhead_mesh.vertices())
            .expect("Failed to create QHuConvexPolyhedron for toolhead");

        HeadLifter {
            object_mesh,
            toolhead_mesh,
            toolhead_aabb_mesh,
            toolhead_qhull,
            safety_distance,
            expected_max_distance,
            tolerance,
            debug: true,
        }
    }

    /// Function to lift the toolhead if it intersects with an object, using dichotomy search.
    pub fn lift_toolhead(
        &self,
        toolhead_pose: &Isometry3<f32>, // Mutable to apply changes
        object_pose: &Isometry3<f32>,
    ) -> Option<Isometry3<f32>> {
        let instant = Instant::now();

        let intersection_test = |test_pose: &Isometry3<f32>| {
            // First, use the bounding box for a loose intersection test
            if !parry3d::query::intersection_test(
                test_pose,
                &self.toolhead_aabb_mesh,
                object_pose,
                self.object_mesh,
            )
            .expect(crate::collisions::SUPPORTED)
            {
                // Toolhead and object are safely separated
                false
            } else if !parry3d::query::intersection_test(
                test_pose,
                self.toolhead_mesh, // Collision detector is faster with mesh, not hull.
                //&self.toolhead_qhull,
                object_pose,
                self.object_mesh,
            )
            .expect(crate::collisions::SUPPORTED)
            {
                // Toolhead and object collides
                true
            } else {
                // Boundary case, perform a precise distance check that is a much slower query
                parry3d::query::distance(
                    test_pose,
                    &self.toolhead_qhull,
                    object_pose,
                    self.object_mesh,
                )
                .expect(crate::collisions::SUPPORTED)
                    <= self.safety_distance
            }
        };
        
        /*
        if !parry3d::query::intersection_test(
            toolhead_pose,
            &self.toolhead_aabb_mesh,
            object_pose,
            self.object_mesh,
        )
            .expect(crate::collisions::SUPPORTED)
        {
            println!("Good immediately: No intersection with object.");
            return Some(*toolhead_pose);
        }
       */

        // Save the original pose for testing and restore it afterward
        let mut last_safe_pose = None;
        let mut iteration_count = 0;

        // Compute the local Z-axis in global coordinates (from the toolhead's orientation)
        // Apply the rotation quaternion to the local Z-axis (0, 0, 1)
        let local_z_axis = toolhead_pose.rotation * Vector3::z();

        // The test pose for that we only modify translation
        let mut test_pose = toolhead_pose.clone();
        let mut low = 0.0; // Minimum displacement
        let mut high = self.expected_max_distance; // Maximum safe displacement

        while high - low > self.tolerance {
            iteration_count += 1;
            // Compute the midpoint
            let mid = (low + high) / 2.0;

            // Test at this displacement along the toolhead's local Z-axis
            let displacement = local_z_axis * mid; // Scale the local Z-axis by the distance
            test_pose.translation.vector = toolhead_pose.translation.vector + displacement; // Apply the displacement

            // Run the intersection test at this position
            if intersection_test(&test_pose) {
                // Intersection detected -> move upward (increase low)
                low = mid;
            } else {
                // No intersection -> move downward (decrease high)
                high = mid;
                last_safe_pose = Some(test_pose); // Store this as a valid safe position
            }
        }

        if self.debug {
            let elapsed = instant.elapsed();
            if let Some(safe_pose) = last_safe_pose {
                println!(
                    "Toolhead moved to final position: {:?} in {:?} in {:?} iterations.",
                    safe_pose.translation.vector, elapsed, iteration_count
                );
            } else {
                println!(
                    "No valid position found within the expected range in {:?} in {:?} iterations.",
                    elapsed, iteration_count
                );
            };
        }

        last_safe_pose
    }
}

#[cfg(test)]
mod tests {
    use crate::synthetic_meshes::sphere_mesh;
    use crate::uplifter::HeadLifter;
    use approx::relative_eq;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
    use once_cell::sync::Lazy;
    use parry3d::shape::TriMesh;

    static TOOLHEAD: Lazy<TriMesh> = Lazy::new(|| sphere_mesh(1.0, 128));

    static OBJECT: Lazy<TriMesh> = Lazy::new(|| sphere_mesh(1.0, 128));

    static UPLIFTER: Lazy<HeadLifter> =
        Lazy::new(|| HeadLifter::new(&OBJECT, &TOOLHEAD, 0.2, 4.0, 0.002));

    #[test]
    fn test_lift_toolhead_touching_on_x_axis() {
        // Toolhead's local Z-axis aligned with the global X-axis
        let rotation =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_2); // Rotate 90° around global Y-axis
        let toolhead_pose = Isometry3::from_parts(Translation3::new(1.0, 0.0, 0.0), rotation);
        let object_pose = Isometry3::identity();

        // Perform the lift
        let toolhead_pose = UPLIFTER
            .lift_toolhead(&toolhead_pose, &object_pose)
            .expect("Failed to lift toolhead");

        // Expect movement along the X-axis
        let expected_translation = 1.0 + 1.0 + 0.2; // Sphere radii + safety distance
        println!("Toolhead moved to: {:?}", toolhead_pose.translation.vector);
        assert!(
            toolhead_pose.translation.vector.x >= expected_translation,
            "Toolhead did not move sufficiently along the X-axis. Actual X: {}",
            toolhead_pose.translation.vector.x
        );
    }

    #[test]
    fn test_lift_toolhead_touching_on_y_axis() {
        // Toolhead's local Z-axis aligned with the global Y-axis
        let rotation =
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_2); // Rotate -90° around global X-axis
        let toolhead_pose = Isometry3::from_parts(Translation3::new(0.0, 1.0, 0.0), rotation);
        let object_pose = Isometry3::identity();

        let toolhead_pose = UPLIFTER
            .lift_toolhead(&toolhead_pose, &object_pose)
            .expect("Failed to lift toolhead");

        // Expect movement along the Y-axis
        let expected_translation = 1.0 + 1.0 + 0.2; // Sphere radii + safety distance
        println!("Toolhead moved to: {:?}", toolhead_pose.translation.vector);
        assert!(
            toolhead_pose.translation.vector.y >= expected_translation,
            "Toolhead did not move sufficiently along the Y-axis. Actual Y: {}",
            toolhead_pose.translation.vector.y
        );
    }

    #[test]
    fn test_lift_toolhead_touching_on_z_axis() {
        // Toolhead's default local Z-axis is already aligned with the global Z-axis
        let toolhead_pose = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, -1.0),
            UnitQuaternion::identity(),
        ); // Toolhead starting at Z = -1
        let object_pose =
            Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), UnitQuaternion::identity()); // Object at Z = 0

        let toolhead_pose = UPLIFTER
            .lift_toolhead(&toolhead_pose, &object_pose)
            .expect("Failed to lift toolhead");

        // Expect movement along the Z-axis
        let expected_translation = 1.0 + 1.0 + 0.2; // Sphere radii + safety distance
        println!("Toolhead moved to: {:?}", toolhead_pose.translation.vector);
        assert!(
            toolhead_pose.translation.vector.z >= expected_translation,
            "Toolhead did not move sufficiently upwards along the Z-axis. Actual Z: {}",
            toolhead_pose.translation.vector.z
        );
    }

    #[test]
    fn test_xz_45() {
        // Toolhead's local Z-axis aligned diagonally in the XZ-plane (45 degrees to both X and Z)
        let rotation =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_4); // 45° rotation around global Y-axis
        let toolhead_pose = Isometry3::from_parts(Translation3::new(0.707, 0.0, 0.707), rotation); // Initial position
        let object_pose = Isometry3::identity();

        let toolhead_pose = UPLIFTER
            .lift_toolhead(&toolhead_pose, &object_pose)
            .expect("Failed to lift toolhead");

        assert_displacement(
            toolhead_pose,
            Vector3::new(45_f32.to_radians().sin(), 0.0, 45_f32.to_radians().cos()),
        );
    }

    #[test]
    fn test_xy_45() {
        // The target Z axis direction in the world coordinate space
        let target_z = Vector3::new(1.0, 1.0, 0.0).normalize();

        // Find the rotation that aligns the local Z axis with the target Z axis
        let rotation = UnitQuaternion::face_towards(&target_z, &Vector3::z_axis());

        // Verify the rotation result: Make sure the rotated Z axis aligns correctly
        let rotated_z = rotation.transform_vector(&Vector3::z());
        println!("Rotated Z axis (world space): {:?}", rotated_z);

        // Quaternion representing the required rotation
        println!("Quaternion: {:?}", rotation);

        let toolhead_pose = Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), rotation); // Initial position
        let object_pose = Isometry3::identity();

        let toolhead_pose = UPLIFTER
            .lift_toolhead(&toolhead_pose, &object_pose)
            .expect("Failed to lift toolhead");

        assert_displacement(
            toolhead_pose,
            Vector3::new(45f32.to_radians().sin(), 45f32.to_radians().cos(), 0.0),
        );
    }

    #[test]
    fn test_xz_30() {
        // Toolhead's local Z-axis rotated 30 degrees downward over the horizon (tilted in the XZ-plane)
        let rotation =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_6); // 30° rotation around global Y-axis
        let toolhead_pose = Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), rotation); // Initial position at the origin
        let object_pose = Isometry3::identity(); // Object located at the origin

        let toolhead_pose = UPLIFTER
            .lift_toolhead(&toolhead_pose, &object_pose)
            .expect("Failed to lift toolhead");

        assert_displacement(
            toolhead_pose,
            Vector3::new(30_f32.to_radians().sin(), 0.0, 30_f32.to_radians().cos()),
        );
    }

    fn assert_displacement(toolhead_pose: Isometry3<f32>, direction: Vector3<f32>) {
        let center_to_center_distance = 2.2;
        let displacement_vector = center_to_center_distance * direction;
        let expected_position = Vector3::new(
            displacement_vector.x,
            displacement_vector.y,
            displacement_vector.z,
        );
        let final_position = toolhead_pose.translation.vector;
        println!(
            "Expected Position: ({:.6}, {:.6}, {:.6})",
            expected_position.x, expected_position.y, expected_position.z
        );
        println!(
            "Final Position: ({:.6}, {:.6}, {:.6})",
            final_position.x, final_position.y, final_position.z
        );

        // Assert that the final position aligns with the expected position
        assert!(
            relative_eq!(final_position.x, expected_position.x, epsilon = 0.002),
            "Toolhead final X position mismatch. Expected {:.6}, Actual {:.6}",
            expected_position.x,
            final_position.x
        );
        assert!(
            relative_eq!(final_position.y, expected_position.y, epsilon = 0.002),
            "Toolhead final Y position mismatch. Expected {:.6}, Actual {:.6}",
            expected_position.y,
            final_position.y
        );
        assert!(
            relative_eq!(final_position.z, expected_position.z, epsilon = 0.002),
            "Toolhead final Z position mismatch. Expected {:.6}, Actual {:.6}",
            expected_position.z,
            final_position.z
        );

        // Total displacement assertion
        let expected_total_displacement = center_to_center_distance;
        let actual_total_displacement = (final_position - Vector3::new(0.0, 0.0, 0.0)).norm();

        println!(
            "Expected Total Displacement: {:.6}, Actual Total Displacement: {:.6}",
            expected_total_displacement, actual_total_displacement
        );

        assert!(
            relative_eq!(
                expected_total_displacement,
                actual_total_displacement,
                epsilon = 0.002
            ),
            "Toolhead final total displacement mismatch. Expected {:.6}, Actual {:.6}",
            expected_total_displacement,
            actual_total_displacement
        );
    }
}
