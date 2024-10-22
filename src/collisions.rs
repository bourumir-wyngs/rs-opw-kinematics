use nalgebra::{Isometry3};
use parry3d::query::contact;
use crate::joint_body::JointBody;
use crate::kinematic_traits::{Joints, Kinematics};

/// Struct representing the geometry of a robot, which is composed of exactly 6 joints.
/// Each joint is represented by a `JointBody`, encapsulating the geometrical shape 
/// of a joint. This struct also provides functionality for detecting collisions 
/// between these joints. This structure does not specify the exact pose of the robot,
/// it is used to build a `PositionedRobot` that does.
pub struct RobotBody {
    /// A fixed-size array of 6 `JointBody` instances, each representing 
    /// the geometrical shape of a joint in the robot.
    pub joint_bodies: [JointBody; 6],


    /// The threshold distance used in collision detection. 
    /// If the distance between two geometries is less than this value, they are considered colliding.
    pub collision_tolerance: f32,

    /// A boolean flag indicating whether the collision detection 
    /// should stop after the first detected collision. When set to `true`, the system will halt 
    /// detection as soon as one collision is found; otherwise, all potential collisions will be checked.
    pub detect_first_collision_only: bool,
}


impl RobotBody {
    /// Constructor to initialize a robot with 6 joints, given tolerance and a flag for early collision detection.
    pub fn new(joint_bodies: [JointBody; 6],
               joint_origins:[Isometry3<f32>; 6],
               tolerance: f32, detect_first_collision_only: bool) -> Self {
        RobotBody {
            joint_bodies,
            collision_tolerance: tolerance,
            detect_first_collision_only,
        }
    }

    /// Perform collision detection and return a vector of pairs of joint indices that collide.
    /// In each returned pair, the lower index will always come first.
    ///
    /// # Arguments
    ///
    /// * `transforms` - A reference to an array of global transforms, where each 
    /// transform corresponds to a joint. The local transforms must already be applied.   
    ///
    /// # Returns
    ///
    /// * A vector of tuples, where each tuple contains the indices of the joints that collide.
    /// **The lower index will always be the first element of the tuple.**
    ///
    /// If the `detect_first_collision_only` flag is true, the function returns after detecting
    /// the first collision.
    pub fn detect_collisions(
        &self,
        transforms: &[Isometry3<f32>; 6],
    ) -> Vec<(usize, usize)> {
        let mut collisions = Vec::new();

        // Loop through joints and ensure each joint pair is checked only once
        for i in 0..6 {
            for j in (i + 1)..6 {
                if j - i > 1 {   // Skip adjacent joints

                    // Get references to the joints
                    let joint1 = &self.joint_bodies[i];
                    let joint2 = &self.joint_bodies[j];

                    // First, check if the simplified shapes (AABB as TriMesh) collide
                    let simplified_contact = contact(
                        &transforms[i], &joint1.simplified_shape,
                        &transforms[j], &joint2.simplified_shape,
                        self.collision_tolerance,
                    );

                    // If simplified shapes collide, proceed to detailed collision check
                    if simplified_contact.is_ok() && simplified_contact.unwrap().is_some() {
                        let shape_contact = contact(
                            &transforms[i], &joint1.transformed_shape,
                            &transforms[j], &joint2.transformed_shape,
                            self.collision_tolerance,
                        );

                        if let Ok(Some(_)) = shape_contact {
                            // Ensure lower index comes first in the tuple
                            collisions.push((i.min(j), i.max(j)));
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

impl RobotBody {
    pub fn collision_details(&self, qs: &Joints, kinematics: &dyn Kinematics) -> Vec<(usize, usize)> {
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
    use nalgebra::Point3;
    use parry3d::shape::TriMesh;
    use super::*;

    fn create_trimesh(x: f32, y: f32, z: f32) -> TriMesh {
        // Define vertices and triangle indices for a triangular pyramid. 
        TriMesh::new(
            vec![
                Point3::new(z, y, z),
                Point3::new(x + 1.0, y, z),
                Point3::new(x, y + 1.0, z),
                Point3::new(x, y, z + 1.0),
            ],
            vec![
                [0, 1, 2],
                [0, 1, 3],
                [0, 2, 3],
                [1, 2, 3],
            ],
        )
    }

    #[test]
    fn test_collision_detection() {
        // Create joints with attached shapes and corresponding translations
        // There are 4 collisions between these joints
        let identity = Isometry3::identity();
        let joints: [JointBody; 6] = [
            JointBody::new(create_trimesh(0.0, 0.0, 0.0), identity),
            JointBody::new(create_trimesh(0.01, 0.01, 0.01), identity),
            JointBody::new(create_trimesh(0.1, 0.1, 0.1), identity),
            // Use local transform at places to be sure it works. This joint must be far away.
            JointBody::new(create_trimesh(0.0, 0.0, 0.0), Isometry3::translation(20.0, 20.0, 20.0)),
            JointBody::new(create_trimesh(30.0, 30.0, 30.0), identity),
            // Place Joint 6 close to joint 1
            JointBody::new(create_trimesh(0.0, 0.0, 0.0), Isometry3::translation(0.02, 0.02, 0.02)),
        ];

        let robot = RobotBody::new(joints, [identity; 6], 0.0,false);

        let collisions = robot.detect_collisions(&[identity; 6]);
        assert!(!collisions.is_empty(), "Expected at least one collision, but none were detected.");

        // Now expect 4 collisions due to the placement of joint 6
        assert_eq!(collisions.len(), 4, "Expected exactly 4 collisions.");

        // Ensure that specific collisions are occurring, including the close ones
        let mut expected_collisions = vec![
            (0usize, 2usize), (0usize, 5usize), (1usize, 5usize), (2usize, 5usize),
        ];

        // Normalize the order of expected collisions (sort them)
        for collision in &mut expected_collisions {
            let (a, b) = *collision;
            if a > b {
                *collision = (b, a);  // Ensure smaller index comes first
            }
        }

        for (shape_a, shape_b) in &collisions {
            let mut collision_names = (*shape_a, *shape_b);
            if collision_names.0 > collision_names.1 {
                collision_names = (collision_names.1, collision_names.0);  // Ensure smaller index comes first
            }

            assert!(
                expected_collisions.contains(&collision_names),
                "Unexpected collision between [{}] and [{}]", shape_a, shape_b
            );
        }

        // Ensure that shape 3 is NOT involved in collisions
        for (shape_a, shape_b) in &collisions {
            assert_ne!(*shape_a, 3, "Shape [3] should not be involved in any collision.");
            assert_ne!(*shape_b, 3, "Shape [3] should not be involved in any collision.");
        }
    }
}
