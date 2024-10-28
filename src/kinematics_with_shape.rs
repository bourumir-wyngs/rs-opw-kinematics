use std::sync::Arc;
use nalgebra::Isometry3;
use parry3d::shape::TriMesh;
use crate::collisions::RobotBody;
use crate::joint_body::CollisionBody;
use crate::kinematic_traits::{Kinematics, Joints, Solutions, Pose, Singularity};

/// Struct that combines the kinematic model of a robot with its geometrical shape.
/// This struct provides both the kinematic functionality for computing joint positions and 
/// the physical structure used for collision detection and other geometric calculations.
pub struct KinematicsWithShape {
    /// The kinematic model of the robot, typically used to compute forward and inverse kinematics.
    /// This is an abstract trait (`Kinematics`), allowing for different implementations of kinematic models.
    pub kinematics: Arc<dyn Kinematics>,

    /// The physical structure of the robot, represented by its joint geometries.
    /// This `RobotBody` contains information about the geometrical shapes of the joints 
    /// and provides functionality for collision detection.
    pub body: RobotBody,
}

/// Struct representing a robot with positioned joints.
/// It contains a vector of references to `PositionedJoint`, where each joint has its precomputed global transform.
///
/// This struct is useful when performing operations that require knowing the precomputed positions and orientations
/// of all the joints in the robot, such as forward kinematics, inverse kinematics, and collision detection.
pub struct PositionedRobot<'a> {
    /// A vector of references to `PositionedJoint`, representing the joints and their precomputed transforms.
    pub joints: Vec<PositionedJoint<'a>>,
    pub tool: Option<PositionedJoint<'a>>,
    pub environment: Vec<&'a CollisionBody>,
}

/// Struct representing a positioned joint, which consists of a reference to a `JointBody`
/// and its precomputed combined transform. This is the global transform of the joint combined with
/// the local transforms of the collision shapes within the joint.
///
/// This struct simplifies access to the final world-space transform of the joint's shapes.
pub struct PositionedJoint<'a> {
    /// A reference to the associated `JointBody` that defines the shapes and collision behavior of the joint.
    pub joint_body: &'a TriMesh,

    /// The combined transformation matrix (Isometry3), representing the precomputed global position
    /// and orientation of the joint in the world space.
    pub transform: Isometry3<f32>,
}

impl KinematicsWithShape {
    /// Method to compute and return a `PositionedRobot` from the current `RobotBody` and a set of joint positions.
    ///
    /// This method uses the forward kinematics to compute the global transforms of each joint
    /// and then creates the corresponding `PositionedJoint` instances, which are collected into a `PositionedRobot`.
    ///
    /// # Arguments
    ///
    /// * `joint_positions` - A reference to the joint values (angles/positions) to compute the forward kinematics.
    ///
    /// # Returns
    ///
    /// * A new instance of `PositionedRobot` containing the positioned joints with precomputed transforms.
    pub fn positioned_robot(&self, joint_positions: &Joints) -> PositionedRobot {
        // Compute the global transforms for each joint using forward kinematics
        let global_transforms: [Isometry3<f32>; 6] = self
            .kinematics
            .forward_with_joint_poses(joint_positions)
            .map(|pose| pose.cast::<f32>());

        // Create a vector of PositionedJoints without mut
        let positioned_joints: Vec<PositionedJoint> = self
            .body
            .joint_meshes
            .iter()
            .enumerate()
            .map(|(i, joint_body)| PositionedJoint {
                joint_body,
                transform: global_transforms[i],
            })
            .collect();

        // Return the PositionedRobot with all the positioned joints
        let positioned_tool = if let Some(tool) = self.body.tool.as_ref() {
            Some(
                PositionedJoint {
                    joint_body: tool,
                    transform: global_transforms[5], // TCP pose
                }
            )
        } else {
            None
        };
        
        // Convert to vector of references
        let referenced_environment = 
            self.body.collision_environment.iter().collect();
        
        PositionedRobot {
            joints: positioned_joints,
            tool: positioned_tool,
            environment: referenced_environment,
        }
    }
}

impl KinematicsWithShape {
    fn filter_colliding_solutions(&self, solutions: Solutions) -> Solutions {
        solutions
            .into_iter()
            .filter(|qs| !self.body.collides(qs, self.kinematics.as_ref()))
            .collect()
    }
}

impl Kinematics for KinematicsWithShape {
    /// Delegates call to underlying Kinematics, but will filter away colliding poses
    fn inverse(&self, pose: &Pose) -> Solutions {
        let solutions = self.kinematics.inverse(pose);
        self.filter_colliding_solutions(solutions)
    }

    /// Delegates call to underlying Kinematics, but will filter away colliding poses
    fn inverse_continuing(&self, pose: &Pose, previous: &Joints) -> Solutions {
        let solutions = self.kinematics.inverse_continuing(pose, previous);
        self.filter_colliding_solutions(solutions)
    }

    fn forward(&self, qs: &Joints) -> Pose {
        self.kinematics.forward(qs)
    }

    /// Delegates call to underlying Kinematics, but will filter away colliding poses
    fn inverse_5dof(&self, pose: &Pose, j6: f64) -> Solutions {
        let solutions = self.kinematics.inverse_5dof(pose, j6);
        self.filter_colliding_solutions(solutions)
    }

    /// Delegates call to underlying Kinematics, but will filter away colliding poses
    fn inverse_continuing_5dof(&self, pose: &Pose, prev: &Joints) -> Solutions {
        let solutions = self.kinematics.inverse_continuing_5dof(pose, prev);
        self.filter_colliding_solutions(solutions)
    }

    fn kinematic_singularity(&self, qs: &Joints) -> Option<Singularity> {
        self.kinematics.kinematic_singularity(qs)
    }

    fn forward_with_joint_poses(&self, joints: &Joints) -> [Pose; 6] {
        self.kinematics.forward_with_joint_poses(joints)
    }
}
