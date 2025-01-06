use crate::collisions::SafetyDistances;
use nalgebra::{Isometry3, Vector3};
use parry3d::bounding_volume::BoundingVolume;
use parry3d::query::intersection_test;
use parry3d::query::QueryDispatcher;
use parry3d::shape::TriMesh;
use std::time::Instant;

/// Function to lift the toolhead if it intersects with an object, using dichotomy search.
fn lift_toolhead(
    toolhead_mesh: &TriMesh,
    toolhead_pose: &Isometry3<f32>, // Mutable to apply changes
    object_mesh: &TriMesh,
    object_pose: &Isometry3<f32>,
    expected_max_distance: f32,
    safety_distance: f32,
) -> Option<Isometry3<f32>> {
    let instant = Instant::now();
    // Step 1: Simplify the toolhead shape to an AABB and loosen it by the safety distance
    let toolhead_aabb = toolhead_mesh.local_aabb().loosened(safety_distance);
    let toolhead_aabb_mesh = crate::collisions::build_trimesh_from_aabb(toolhead_aabb);

    // Step 2: Define a closure for intersection/distance testing
    let intersection_test = |test_pose: &Isometry3<f32>| {
        // First, use the bounding box for a loose intersection test
        if !parry3d::query::intersection_test(
            test_pose,
            &toolhead_aabb_mesh,
            object_pose,
            object_mesh,
        )
            .expect(crate::collisions::SUPPORTED)
        {
            // Toolhead and object are safely separated
            false
        } else if !parry3d::query::intersection_test(
            test_pose,
            toolhead_mesh,
            object_pose,
            object_mesh,
        )
            .expect(crate::collisions::SUPPORTED)
        {
            // Toolhead and object collides
            true
        } else {
            // Boundary case, perform precise distance check that is a much slower query
            parry3d::query::distance(test_pose, toolhead_mesh, object_pose, object_mesh)
                .expect(crate::collisions::SUPPORTED)
                <= safety_distance
        }
    };

    // Step 3: Binary search for the safe position along the toolhead's local Z-axis
    let mut low = 0.0; // Minimum displacement
    let mut high = expected_max_distance; // Maximum safe displacement
    let tolerance = 0.002; // Precision threshold
    let mut mid;

    // Save the original pose for testing and restore it afterward
    let mut last_safe_pose = None;
    let mut found_safe_position = false;
    let mut iteration_count = 0;

    // Compute the local Z-axis in global coordinates (from the toolhead's orientation)
    // Apply the rotation quaternion to the local Z-axis (0, 0, 1)
    let local_z_axis = toolhead_pose.rotation * Vector3::z(); 
    
    // The test pose for that we only modify translation
    let mut test_pose = toolhead_pose.clone();
    
    while high - low > tolerance {
        iteration_count += 1;
        // Compute the midpoint
        mid = (low + high) / 2.0;

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
            found_safe_position = true;
        }
    }

    // Step 4: Move the toolhead to the last safe position
    if let Some(safe_pose) = last_safe_pose {
        println!(
            "Toolhead moved to final position: {:?} in {:?} in {:?} iterations.",
            safe_pose.translation.vector,
            instant.elapsed(),
            iteration_count
        );
        Some(safe_pose)
    } else {
        println!(
            "No valid position found within the expected range in {:?} in {:?} iterations.",
            instant.elapsed(), iteration_count
        );
        None
    }
}

#[cfg(test)]
mod tests {
    use crate::synthetic_meshes::sphere_mesh;
    use crate::uplifter::lift_toolhead;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
    use once_cell::sync::Lazy;
    use parry3d::shape::TriMesh;
    use std::sync::Arc;
    use std::time::Instant;

    static TOOLHEAD: Lazy<Arc<TriMesh>> = Lazy::new(|| {
        let t_start = Instant::now();
        let mesh = sphere_mesh(1.0, 128);
        println!("Sphere mesh built in {:?}", t_start.elapsed());
        Arc::new(mesh) // Thread-safe and reusable
    });

    static OBJECT: Lazy<Arc<TriMesh>> = Lazy::new(|| {
        let t_start = Instant::now();
        let mesh = sphere_mesh(1.0, 128);
        println!("Sphere mesh built in {:?}", t_start.elapsed());
        Arc::new(mesh) // Thread-safe and reusable
    });

    #[test]
    fn test_lift_toolhead_touching_on_x_axis() {
        // Toolhead's local Z-axis aligned with the global X-axis
        let rotation =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_2); // Rotate 90° around global Y-axis
        let toolhead_pose = Isometry3::from_parts(Translation3::new(1.0, 0.0, 0.0), rotation);
        let object_pose = Isometry3::identity();

        // Perform the lift
        let toolhead_pose = lift_toolhead(
            &TOOLHEAD,
            &toolhead_pose,
            &OBJECT,
            &object_pose,
            1.5, // Expected max displacement
            0.2, // Safety distance
        ).unwrap();

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

        // Perform the lift
        let toolhead_pose = lift_toolhead(
            &TOOLHEAD,
            &toolhead_pose,
            &OBJECT,
            &object_pose,
            1.5, // Expected max displacement
            0.2, // Safety distance
        ).unwrap();
        
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
        let mut toolhead_pose = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, -1.0),
            UnitQuaternion::identity(),
        ); // Toolhead starting at Z = -1
        let object_pose =
            Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), UnitQuaternion::identity()); // Object at Z = 0

        // Perform the lift
        let toolhead_pose = lift_toolhead(
            &TOOLHEAD,
            &toolhead_pose,
            &OBJECT,
            &object_pose,
            5.0, // Expected max displacement
            0.2, // Safety distance
        ).unwrap();

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
    fn test_lift_toolhead_touching_45_degree_x_z() {
        // Toolhead's local Z-axis aligned diagonally in the XZ-plane (45 degrees to both X and Z)
        let rotation =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_4); // 45° rotation around global Y-axis
        let toolhead_pose =
            Isometry3::from_parts(Translation3::new(0.707, 0.0, 0.707), rotation); // Initial position
        let object_pose = Isometry3::identity();

        // Perform the lift
        let toolhead_pose = lift_toolhead(
            &TOOLHEAD,
            &toolhead_pose,
            &OBJECT,
            &object_pose,
            1.5, // Default expected max displacement
            0.2, // Safety distance
        ).unwrap();

        // Calculate initial overlap
        let initial_position = Vector3::new(0.707, 0.0, 0.707);
        let initial_distance = initial_position.norm(); // Distance from toolhead center to object center
        let expected_contact_distance = 1.0 + 1.0; // Sum of the toolhead radius + object radius
        let initial_overlap = expected_contact_distance - initial_distance;

        // Expected diagonal distance (hypotenuse considering initial overlap)
        let expected_total_displacement = (1.0 + 1.0 + 0.2) - initial_overlap;

        // Components of the displacement
        let expected_x_displacement = expected_total_displacement * (45f32.to_radians()).cos(); // ≈ 1.556
        let expected_z_displacement = expected_total_displacement * (45f32.to_radians()).sin(); // ≈ 1.556

        // Calculate actual displacement
        let final_position = toolhead_pose.translation.vector;
        let displacement_vector = final_position - initial_position;
        let actual_x_displacement = displacement_vector.x; // Actual X-component
        let actual_z_displacement = displacement_vector.z; // Actual Z-component

        println!(
            "Toolhead moved to: {:?}, X displacement = {:.6}, Z displacement = {:.6}",
            final_position, actual_x_displacement, actual_z_displacement
        );

        // Allow for a small error tolerance (e.g., 0.001) in each component
        assert!(
            (actual_x_displacement - expected_x_displacement).abs() <= 0.002,
            "Toolhead did not move sufficiently along the X-axis. Expected {:.6}, Actual {:.6}",
            expected_x_displacement,
            actual_x_displacement
        );
        assert!(
            (actual_z_displacement - expected_z_displacement).abs() <= 0.002,
            "Toolhead did not move sufficiently along the Z-axis. Expected {:.6}, Actual {:.6}",
            expected_z_displacement,
            actual_z_displacement
        );

        // Validate total displacement
        let total_displacement = displacement_vector.norm(); // Magnitude of displacement vector
        println!(
            "Total displacement = {:.6}, Expected = {:.6}",
            total_displacement, expected_total_displacement
        );
        assert!(
            (total_displacement - expected_total_displacement).abs() <= 0.002,
            "Toolhead did not move sufficiently diagonally. Expected total displacement = {:.6}, Actual = {:.6}",
            expected_total_displacement,
            total_displacement
        );
    }
}
