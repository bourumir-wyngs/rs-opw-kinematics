use nalgebra::{Isometry3, Point3};
use parry3d::query::contact;
use parry3d::shape::{SharedShape, TriMesh};

/// Struct representing a shape (using TriMesh here, but it can be other shapes)
pub struct CollisionShape {
    /// Name or identifier for the shape
    pub name: String,
    /// Local transform relative to the joint (position/orientation of the shape within the joint's space)
    pub local_transform: Isometry3<f64>,
    /// The collision shape, can be a TriMesh or any other shape
    pub shape: SharedShape,
}

/// Struct representing a joint, which contains multiple shapes
pub struct Joint {
    /// Name of the joint
    pub name: String,
    /// Global transform (position/orientation of the joint in the world space, after forward kinematics)
    pub global_transform: Isometry3<f64>,
    /// List of shapes attached to the joint
    pub shapes: Vec<CollisionShape>,
}

/// Struct representing the robot, which consists of multiple joints
pub struct Robot {
    /// List of joints in the robot, added in order
    pub joints: Vec<Joint>,
    /// Tolerance for collision detection
    pub tolerance: f32,
    /// Flag to stop after the first collision is detected (if true)
    pub detect_first_collision_only: bool,
}

impl Robot {
    /// Constructor to initialize a robot with a given tolerance and a flag to stop after the first collision
    ///
    /// # Arguments
    ///
    /// * `tolerance` - Tolerance for collision detection (how close two shapes must be to collide)
    /// * `detect_first_collision_only` - If true, stops checking after the first collision is detected
    ///
    /// # Returns
    ///
    /// * A new instance of `Robot`
    pub fn new(tolerance: f32, detect_first_collision_only: bool) -> Self {
        Robot {
            joints: Vec::new(),
            tolerance,
            detect_first_collision_only,
        }
    }

    /// Add a joint to the robot; the added joint is adjacent to the last joint in the list
    ///
    /// # Arguments
    ///
    /// * `joint` - The `Joint` to be added to the robot
    pub fn add_joint(&mut self, joint: Joint) {
        self.joints.push(joint);
    }

    /// Helper function to check if two joints are adjacent (consecutive in the list)
    ///
    /// # Arguments
    ///
    /// * `joint1_idx` - Index of the first joint
    /// * `joint2_idx` - Index of the second joint
    ///
    /// # Returns
    ///
    /// * `true` if the joints are adjacent, otherwise `false`
    fn are_adjacent(&self, joint1_idx: usize, joint2_idx: usize) -> bool {
        // Joints are adjacent if they are consecutive (i.e., the difference in their indices is 1)
        (joint1_idx as isize - joint2_idx as isize).abs() == 1
    }

    /// Perform collision detection and return a vector of pairs of shapes that collide
    ///
    /// # Returns
    ///
    /// * A vector of references to pairs of `CollisionShape`s that collide
    ///
    /// If the `detect_first_collision_only` flag is true, the function returns after detecting
    /// the first collision.
    pub fn detect_collisions(&self) -> Vec<(&CollisionShape, &CollisionShape)> {
        let mut collisions = Vec::new();

        // Loop through joints and ensure each joint pair is checked only once
        for (i, joint1) in self.joints.iter().enumerate() {
            for (j, joint2) in self.joints.iter().enumerate().skip(i + 1) {
                // Skip checking collision for adjacent joints
                if self.are_adjacent(i, j) {
                    continue;
                }

                // Loop through shapes in joint1
                for shape1 in &joint1.shapes {
                    // Loop through shapes in joint2
                    for shape2 in &joint2.shapes {
                        // Perform collision detection using the combined global and local transforms for each shape
                        let contact = contact(
                            &shape1.combined_transform(&joint1.global_transform).cast::<f32>(), // Combined transform for shape1
                            shape1.shape.as_ref(),
                            &shape2.combined_transform(&joint2.global_transform).cast::<f32>(), // Combined transform for shape2
                            shape2.shape.as_ref(),
                            self.tolerance, // Use the tolerance specified at the robot's construction
                        );

                        if let Ok(Some(_)) = contact {
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
    ///
    /// # Arguments
    ///
    /// * `joint_global_transform` - The global transform of the joint
    ///
    /// # Returns
    ///
    /// * The combined transform of the shape in world space
    pub fn combined_transform(&self, joint_global_transform: &Isometry3<f64>) -> Isometry3<f64> {
        // The final global transform is the product of the joint's global transform and the shape's local transform
        joint_global_transform * self.local_transform
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_collision_detection() {
        // Define vertices and triangle indices for a sample mesh
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];

        let indices = vec![
            [0, 1, 2],
            [0, 1, 3],
            [0, 2, 3],
            [1, 2, 3],
        ];

        // Create the TriMesh shape
        let trimesh = TriMesh::new(vertices.clone(), indices.clone());
        let shape1 = CollisionShape {
            name: String::from("Shape 1"),
            local_transform: Isometry3::identity(),
            shape: SharedShape::new(trimesh),
        };

        // Another sample shape using the same mesh, but translated slightly to ensure collision
        let trimesh2 = TriMesh::new(vertices.clone(), indices.clone());
        let shape2 = CollisionShape {
            name: String::from("Shape 2"),
            local_transform: Isometry3::translation(0.1, 0.1, 0.1), // Small translation, ensuring collision
            shape: SharedShape::new(trimesh2),
        };

        // Third shape far away, ensuring no collision
        let trimesh3 = TriMesh::new(vertices.clone(), indices.clone());
        let shape3 = CollisionShape {
            name: String::from("Shape 3"),
            local_transform: Isometry3::translation(10.0, 10.0, 10.0), // Far away, no collision
            shape: SharedShape::new(trimesh3),
        };

        // Create joints and attach the shapes
        let joint1 = Joint {
            name: String::from("Joint 1"),
            global_transform: Isometry3::identity(),
            shapes: vec![shape1],
        };

        let joint2 = Joint {
            name: String::from("Joint 2"),
            global_transform: Isometry3::identity(),
            shapes: vec![shape2],
        };

        let joint3 = Joint {
            name: String::from("Joint 3"),
            global_transform: Isometry3::identity(),
            shapes: vec![shape3],
        };

        // Create a new robot and add the joints
        let mut robot = Robot::new(0.0, false);


        // Joints 1 and 2 are close, joint 3 is far. If we put 1 and 2 adjacent, the 
        // collision between them will not be checked.
        robot.add_joint(joint1);
        robot.add_joint(joint3);  
        robot.add_joint(joint2);        

        // Perform collision detection and assert at least one collision was detected
        let collisions = robot.detect_collisions();
        assert!(!collisions.is_empty(), "Expected at least one collision, but none were detected.");

        // Optionally: Check if the correct shapes were involved in the collision
        assert_eq!(collisions.len(), 1, "Expected exactly 1 collision.");
        assert_eq!(collisions[0].0.name, "Shape 1");
        assert_eq!(collisions[0].1.name, "Shape 2");

        // Ensure shape 3 is not involved in any collision
        for (shape_a, shape_b) in collisions {
            assert_ne!(shape_a.name, "Shape 3", "Shape 3 should not be involved in any collision.");
            assert_ne!(shape_b.name, "Shape 3", "Shape 3 should not be involved in any collision.");
        }
    }
}
