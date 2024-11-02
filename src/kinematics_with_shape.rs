use std::sync::Arc;
use nalgebra::{Isometry3};
use parry3d::shape::TriMesh;
use crate::collisions::RobotBody;
use crate::constraints::Constraints;
use crate::joint_body::{BaseBody, CollisionBody};
use crate::kinematic_traits::{Kinematics, Joints, Solutions, Pose, Singularity, J6};
use crate::kinematics_impl::OPWKinematics;
use crate::parameters::opw_kinematics::Parameters;
use crate::tool::{Base, Tool};

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

impl KinematicsWithShape {
    /// Constructs a new `KinematicsWithShape` instance for a 6DOF robot.
    /// This method consumes all parameters, moving them inside the robot.
    /// This is important for meshes that are bulky.
    ///
    /// # Parameters
    ///
    /// * `opw_parameters` - OPW parameters defining this robot
    ///
    /// * `constraints`: joint constraint limits
    ///
    /// * `joint_meshes` - An array of [`TriMesh; 6`] representing the meshes for each joint's body.
    ///
    /// * `base_mesh` - The mesh of the robot base.
    /// * `base_transform` - The transform to apply to the base mesh. This transform brings
    /// the robot into its intended location inside the robotic cell.
    ///
    /// * `tool_mesh` - The mesh of the robot's tool, which will also be checked for collisions.
    /// * `tool_transform` - The transform to apply to the tool mesh. It defines the location
    /// of the "action point" of the robot whose location and rotation (pose) is the pose
    /// for direct and inverse kinematics calls.
    ///
    /// * `collision_environment` - A vector of collision objects represented by `CollisionBody`, 
    /// where each object includes a mesh and its transform.
    ///
    /// * `first_pose_only` - As collision check may be expensive, check if we need multiple
    ///  checked solutions if inverse kinematics, or the first (best) is enough
    ///
    /// # Returns
    ///
    /// A `KinematicsWithShape` instance with defined kinematic structure and shapes for collision detection.
    pub fn new(
        opw_parameters: Parameters,
        constraints: Constraints,
        joint_meshes: [TriMesh; 6],
        base_mesh: TriMesh,
        base_transform: Isometry3<f64>,
        tool_mesh: TriMesh,
        tool_transform: Isometry3<f64>,
        collision_environment: Vec<CollisionBody>,
        first_pose_only: bool,
    ) -> Self {
        KinematicsWithShape {
            kinematics: Arc::new(Self::create_robot_with_base_and_tool(
                base_transform, tool_transform, opw_parameters, constraints)
            ),
            body: RobotBody {
                joint_meshes,
                base: Some(BaseBody {
                    mesh: base_mesh,
                    base_pose: base_transform.cast(),
                }),
                tool: Some(tool_mesh),
                collision_environment,
                first_pose_only: first_pose_only,
            },
        }
    }

    fn create_robot_with_base_and_tool(
        base_transform: Isometry3<f64>,
        tool_transform: Isometry3<f64>,
        opw_parameters: Parameters,
        constraints: Constraints) -> Tool {
        let plain_robot = OPWKinematics::new_with_constraints(
            opw_parameters, constraints);

        let robot_with_base = Base {
            robot: Arc::new(plain_robot),
            base: base_transform.clone(),
        };

        let robot_with_base_and_tool = Tool {
            robot: Arc::new(robot_with_base),
            tool: tool_transform.clone(),
        };

        robot_with_base_and_tool
    }
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
        let positioned_tool =
            if let Some(tool) = self.body.tool.as_ref() {
                Some(
                    PositionedJoint {
                        joint_body: tool,
                        transform: global_transforms[J6], // TCP pose
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
    /// Remove solutions that are reported as result the robot colliding with self or
    /// environment. Only retain non-colliding solutions. As the check may be expensive,
    /// specify if we want to check all solutions, or only the first non-colliding one is
    /// of interest.
    fn filter_colliding_solutions(&self, solutions: Solutions, first_pose_only: bool) -> Solutions {
        if first_pose_only {
            // Find and return the first non-colliding solution as a singleton vector
            solutions
                .into_iter()
                .find(|qs| !self.body.collides(qs, self.kinematics.as_ref()))
                .map(|qs| vec![qs]) // Wrap the solution in a vector
                .unwrap_or_default() // Return an empty vector if no solution is found
        } else {
            // Filter out all colliding solutions as before
            solutions
                .into_iter()
                .filter(|qs| !self.body.collides(qs, self.kinematics.as_ref()))
                .collect()
        }
    }


    /// Check for collisions for the given joint position. Both self-collisions and collisions
    /// with environment are checked. This method simply returns true (if collides) or false (if not) 
    pub fn collides(&self, joints: &Joints) -> bool {
        self.body.collides(joints, self.kinematics.as_ref())
    }

    /// Provide details about he collision, who with whom collides.
    /// Depending on if the RobotBody has been configured for complete check,
    /// either all collisions or only the first found colliding pair
    /// is returned.
    pub fn collision_details(&self, joints: &Joints) -> Vec<(usize, usize)> {
        self.body.collision_details(joints, self.kinematics.as_ref())
    }
}

impl Kinematics for KinematicsWithShape {
    /// Delegates call to underlying Kinematics, but will filter away colliding poses
    fn inverse(&self, pose: &Pose) -> Solutions {
        let solutions = self.kinematics.inverse(pose);
        self.filter_colliding_solutions(solutions, self.body.first_pose_only)
    }

    /// Delegates call to underlying Kinematics, but will filter away colliding poses
    fn inverse_continuing(&self, pose: &Pose, previous: &Joints) -> Solutions {
        let solutions = self.kinematics.inverse_continuing(pose, previous);
        self.filter_colliding_solutions(solutions, self.body.first_pose_only)
    }

    fn forward(&self, qs: &Joints) -> Pose {
        self.kinematics.forward(qs)
    }

    /// Delegates call to underlying Kinematics, but will filter away colliding poses
    fn inverse_5dof(&self, pose: &Pose, j6: f64) -> Solutions {
        let solutions = self.kinematics.inverse_5dof(pose, j6);
        self.filter_colliding_solutions(solutions, self.body.first_pose_only)
    }

    /// Delegates call to underlying Kinematics, but will filter away colliding poses
    fn inverse_continuing_5dof(&self, pose: &Pose, prev: &Joints) -> Solutions {
        let solutions = self.kinematics.inverse_continuing_5dof(pose, prev);
        self.filter_colliding_solutions(solutions, self.body.first_pose_only)
    }

    fn kinematic_singularity(&self, qs: &Joints) -> Option<Singularity> {
        self.kinematics.kinematic_singularity(qs)
    }

    fn forward_with_joint_poses(&self, joints: &Joints) -> [Pose; 6] {
        self.kinematics.forward_with_joint_poses(joints)
    }
}
