use nalgebra::{Isometry3, Point3};
use parry3d::query::contact;
use parry3d::shape::{SharedShape, TriMesh};
use crate::joint_body::{CollisionShape, JointBody};
use crate::kinematic_traits::{Joints, Kinematics};

/// Struct representing the robot, which consists of exactly 6 joints
pub struct RobotBody {
    pub joints: [JointBody; 6],    // Fixed-size array of 6 joints
    pub tolerance: f32,
    pub detect_first_collision_only: bool,
}

impl RobotBody {
    /// Constructor to initialize a robot with 6 joints, given tolerance and a flag for early collision detection.
    pub fn new(joints: [JointBody; 6], tolerance: f32, detect_first_collision_only: bool) -> Self {
        RobotBody {
            joints,
            tolerance,
            detect_first_collision_only,
        }
    }

    /// Perform collision detection and return a vector of pairs of shapes that collide
    ///
    /// # Arguments
    ///
    /// * `global_transforms` - A reference to an array of global transforms, where each transform corresponds to a joint
    ///
    /// # Returns
    ///
    /// * A vector of references to pairs of `CollisionShape`s that collide
    ///
    /// If the `detect_first_collision_only` flag is true, the function returns after detecting
    /// the first collision.
    pub fn detect_collisions(
        &self,
        global_transforms: &[Isometry3<f32>; 6],
    ) -> Vec<(&CollisionShape, &CollisionShape)> {
        let mut collisions = Vec::new();

        // Loop through joints and ensure each joint pair is checked only once
        for i in 0..6 {
            for j in (i + 1)..6 {
                // Skip adjacent joints
                if (i as isize - j as isize).abs() == 1 {
                    continue;
                }

                // Get references to the joints
                let joint1 = &self.joints[i];
                let joint2 = &self.joints[j];

                // First, check if the simplified shapes (AABB as TriMesh) collide
                let simplified_contact = contact(
                    &global_transforms[i], 
                    &joint1.simplified_shape,
                    &global_transforms[j],
                    &joint2.simplified_shape,
                    self.tolerance,
                );

                if simplified_contact.is_err() || simplified_contact.unwrap().is_none() {
                    // If the simplified shapes do not collide, skip checking detailed shapes
                    continue;
                }

                // If simplified shapes collide, proceed to detailed collision check
                for shape1 in &joint1.shapes {
                    for shape2 in &joint2.shapes {
                        // Perform collision detection using the combined global and local transforms for each shape
                        let shape_contact = contact(
                            &shape1.combined_transform(&global_transforms[i]), // Combined transform for shape1
                            shape1.shape.as_ref(),
                            &shape2.combined_transform(&global_transforms[j]), // Combined transform for shape2
                            shape2.shape.as_ref(),
                            self.tolerance, // Use the tolerance specified at the robot's construction
                        );

                        if let Ok(Some(_)) = shape_contact {
                            // Add a pair of shapes involved in the collision to the vector
                            collisions.push((shape1, shape2));

                            // If we only need to detect the first collision, stop immediately
                            if self.detect_first_collision_only {
                                return collisions;
                            }
                        }
                    }
                }
            }
        }

        collisions
    }
}

impl CollisionShape {
    /// Calculate the final (global) transform of a shape by combining the joint's global transform
    /// and the shape's local transform
    pub fn combined_transform(&self, joint_global_transform: &Isometry3<f32>) -> Isometry3<f32> {
        joint_global_transform * self.local_transform
    }
}

impl RobotBody {
    pub fn collision_details(&self, qs: &Joints, kinematics: &dyn Kinematics) -> Vec<(&CollisionShape, &CollisionShape)> {
        let joint_poses = kinematics.forward_with_joint_poses(qs);
        let joint_poses_f32: [Isometry3<f32>; 6] = joint_poses.map(|pose| pose.cast::<f32>());
        self.detect_collisions(&joint_poses_f32)
    }

    pub fn collides(&self, qs: &Joints, kinematics: &dyn Kinematics) -> bool {
        !self.collision_details(qs, kinematics).is_empty()
    }    
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_trimesh(x: f32, y: f32, z: f32, name: &str) -> CollisionShape {
        // Define vertices and triangle indices for a triangular pyramid. 
        let trimesh = TriMesh::new(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
            ],
            vec![
                [0, 1, 2],
                [0, 1, 3],
                [0, 2, 3],
                [1, 2, 3],
            ],
        );
        let local_transform = Isometry3::translation(x, y, z);

        CollisionShape {
            name: String::from(name),
            local_transform,
            shape: SharedShape::new(trimesh),
        }
    }

    #[test]
    fn test_collision_detection() {
        // Create joints with attached shapes and corresponding translations
        let joints: [JointBody; 6] = [
            JointBody::new(vec![create_trimesh(0.0, 0.0, 0.0, "Shape 1")]),
            JointBody::new(vec![create_trimesh(0.01, 0.01, 0.01, "Shape 2")]),
            JointBody::new(vec![create_trimesh(0.1, 0.1, 0.1, "Shape 3")]),
            JointBody::new(vec![create_trimesh(20.0, 20.0, 20.0, "Shape 4")]),
            JointBody::new(vec![create_trimesh(30.0, 30.0, 30.0, "Shape 5")]),
            // Place Joint 6 close to joint 1
            JointBody::new(vec![create_trimesh(0.02, 0.02, 0.02, "Shape 6")]),
        ];

        // Create a new robot with exactly 6 joints
        let robot = RobotBody::new(joints, 0.0, false);

        // Perform collision detection and assert that multiple collisions were detected
        let collisions = robot.detect_collisions(&[Isometry3::identity(); 6]);
        assert!(!collisions.is_empty(), "Expected at least one collision, but none were detected.");

        // Now expect 4 collisions due to the placement of joint 6
        assert_eq!(collisions.len(), 4, "Expected exactly 4 collisions.");

        // Ensure that specific collisions are occurring, including the close ones
        let expected_collisions = vec![
            ("Shape 1", "Shape 3"),
            ("Shape 1", "Shape 6"),
            ("Shape 2", "Shape 6"),
            ("Shape 3", "Shape 6"),
        ];

        for (shape_a, shape_b) in &collisions {
            let collision_names = (shape_a.name.as_str(), shape_b.name.as_str());
            assert!(
                expected_collisions.contains(&collision_names)
                    || expected_collisions.contains(&(collision_names.1, collision_names.0)),
                "Unexpected collision between {} and {}",
                shape_a.name,
                shape_b.name
            );
        }

        // Ensure that shape 3 is involved in collisions but not adjacent shapes like Shape 2
        for (shape_a, shape_b) in &collisions {
            assert_ne!(shape_a.name, "Shape 4", "Shape 4 should not be involved in any collision.");
            assert_ne!(shape_b.name, "Shape 4", "Shape 4 should not be involved in any collision.");
        }
    }
    
    
}
