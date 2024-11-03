//! Implements collision detection

use std::collections::HashSet;
use nalgebra::Isometry3;
use parry3d::shape::TriMesh;
use rayon::prelude::{IntoParallelRefIterator, ParallelIterator};
use crate::kinematic_traits::{Joints, Kinematics, Solutions, ENV_START_IDX, J1, J5, J6, J_BASE, J_TOOL};

/// Optional structure attached to the robot base joint. It has its own global transform
/// that brings the robot to the location. This structure includes two transforms,
/// one bringing us to the base of the stand supporting the robot (and this is the 
/// pose of the stand itself), and then another defining the point where J1 rotating part
/// begins.
pub struct BaseBody {
    pub mesh: TriMesh,
    pub base_pose: Isometry3<f32>,
}


/// Static object against that we check the robot does not collide.
/// Unlike robot joint, it has the global transform allowing to place it
/// where desired.
pub struct CollisionBody {
    /// Mesh representing this collision object
    pub mesh: TriMesh,
    /// Global transform of this collision object.
    pub pose: Isometry3<f32>,
}

/// Pre-apply local transform for the mesh if needed. This may be needed
/// for the robot joint if it is defined in URDF with some local transform
pub fn transform_mesh(shape: &TriMesh, local_transform: &Isometry3<f32>) -> TriMesh {
    // Apply the local transformation to the shape
    TriMesh::new(shape
                     .vertices()
                     .iter()
                     .map(|v|
                         local_transform.transform_point(v))
                     .collect(),
                 shape.indices().to_vec())
}

/// Struct representing a collision task for detecting collisions
/// between two objects with given transforms and shapes.
struct CollisionTask<'a> {
    i: usize, // reporting index of the first shape
    j: usize, // reporting index of the second shape
    transform_i: &'a Isometry3<f32>,
    transform_j: &'a Isometry3<f32>,
    shape_i: &'a TriMesh,
    shape_j: &'a TriMesh,
}

/// Struct representing the geometry of a robot, which is composed of exactly 6 joints.
pub struct RobotBody {
    /// Joint meshes, one per joint
    pub joint_meshes: [TriMesh; 6],

    /// Tool meshes, optional if the robot has no tool
    pub tool: Option<TriMesh>,

    /// Robot base specification
    pub base: Option<BaseBody>,

    /// Environment objects arround the robot.
    pub collision_environment: Vec<CollisionBody>,

    /// As collision checks are expensive, specify if all possible solutions of inverse 
    /// kinematics are returned, or only non-coliding solution considered best 
    pub first_pose_only: bool,
}

// Public methods
impl RobotBody {
    /// Returns detailed information about all collisions detected in the robot's configuration.
    pub fn collision_details(&self, qs: &Joints, kinematics: &dyn Kinematics) -> Vec<(usize, usize)> {
        let joint_poses = kinematics.forward_with_joint_poses(qs);
        let joint_poses_f32: [Isometry3<f32>; 6] = joint_poses.map(|pose| pose.cast::<f32>());
        self.detect_collisions(&joint_poses_f32, false)
    }

    /// Returns true if any collision is detected in the robot's configuration.
    pub fn collides(&self, qs: &Joints, kinematics: &dyn Kinematics) -> bool {
        let joint_poses = kinematics.forward_with_joint_poses(qs);
        let joint_poses_f32: [Isometry3<f32>; 6] = joint_poses.map(|pose| pose.cast::<f32>());
        !self.detect_collisions(&joint_poses_f32, true).is_empty()
    }

    /// Return non colliding offsets, tweaking each joint plus minus either side, either into
    /// 'to' or into 'from'. This is required for planning algorithms like A*. We can do 
    ///  less collision checks as we only need to check the joint branch of the robot we moved.     
    pub(crate) fn non_colliding_offsets(
        &self,
        initial: &Joints,
        from: &Joints,
        to: &Joints,
        kinematics: &dyn Kinematics,
    ) -> Solutions {
        let mut solutions = Vec::with_capacity(12);

        // Generate 12 neighbors by tweaking each joint in either direction
        for joint_index in 0..6 {
            // Create two possible configurations: one towards 'from', one towards 'to'
            for target in [from, to].iter() {
                let mut new_joints = *initial;
                new_joints[joint_index] = target[joint_index];

                // Generate the full joint poses for collision checking
                let joint_poses = kinematics.forward_with_joint_poses(&new_joints);
                let joint_poses_f32: [Isometry3<f32>; 6] = joint_poses.map(|pose| pose.cast::<f32>());

                // Determine joints that do not require collision checks
                let skip_indices: HashSet<usize> = (0..joint_index).collect();

                // Detect collisions, skipping specified indices
                if self.detect_collisions_with_skips(&joint_poses_f32, false, &skip_indices).is_empty() {
                    solutions.push(new_joints); // Add non-colliding configuration
                }
            }
        }
        solutions
    }
}

const SUPPORTED: &'static str = "Mesh intersection should be supported by Parry3d";

impl RobotBody {
    /// Parallel version with Rayon
    fn process_collision_tasks(tasks: Vec<CollisionTask>, first_collision_only: bool) -> Vec<(usize, usize)> {
        if first_collision_only {
            // Exit as soon as any collision is found
            tasks.par_iter()
                .find_map_any(|task| {
                    let collides = parry3d::query::intersection_test(
                        task.transform_i, task.shape_i, task.transform_j, task.shape_j)
                        .expect(SUPPORTED);
                    if collides {
                        Some((task.i.min(task.j), task.i.max(task.j)))
                    } else {
                        None
                    }
                })
                .into_iter() // Converts the Option result to an iterator
                .collect()
        } else {
            // Collect all collisions
            tasks.par_iter()
                .filter_map(|task| {
                    let collides = parry3d::query::intersection_test(
                        task.transform_i, task.shape_i, task.transform_j, task.shape_j)
                        .expect(SUPPORTED);
                    if collides {
                        Some((task.i.min(task.j), task.i.max(task.j)))
                    } else {
                        None
                    }
                })
                .collect()
        }
    }

    /// Sequential version, currently not in use
    #[allow(dead_code)]
    fn process_collision_tasks_sequential(tasks: Vec<CollisionTask>, first_collision_only: bool) -> Vec<(usize, usize)> {
        let mut collisions = Vec::with_capacity(4);

        for task in tasks {
            if parry3d::query::intersection_test(task.transform_i, task.shape_i, task.transform_j, task.shape_j)
                .expect(SUPPORTED)
            {
                collisions.push((task.i.min(task.j), task.i.max(task.j)));
                if first_collision_only {
                    break;
                }
            }
        }

        collisions
    }

    // Count exact number of tasks so we do not need to reallocate the vector.
    // This may return slightly larger number if skips are active.
    fn count_tasks(&self, skip: &HashSet<usize>) -> usize {
        if skip.len() >= 6 {
            panic!(
                "At most 5 joints can be skipped, but {} were passed: {:?}", skip.len(), skip
            );
        }
        let tool_env_tasks = if self.tool.is_some() {
            self.collision_environment.len()
        } else {
            0
        };

        let joint_joint_tasks = 10; // Fixed number for 6 joints excluding adjacent pairs
        let joint_env_tasks = (6 - skip.len()) * self.collision_environment.len();
        let joint_tool_tasks = if self.tool.is_some() {
            4 // Only 4 tasks for joints (excluding J5 and J6) with the tool
        } else {
            0
        };

        let joint_base_tasks = if self.base.is_some() {
            5 // Only 5 tasks for joints (excluding J1) with the base
        } else {
            0
        };
        let tool_base_task = if self.tool.is_some() && self.base.is_some() {
            1 // Only 1 task for tool vs base if both are present
        } else {
            0
        };

        // Sum all tasks
        tool_env_tasks + joint_joint_tasks + joint_env_tasks +
            joint_tool_tasks + joint_base_tasks + tool_base_task
    }


    fn detect_collisions(
        &self, joint_poses: &[Isometry3<f32>; 6], first_collision_only: bool) -> Vec<(usize, usize)> {
        let empty_set: HashSet<usize> = HashSet::with_capacity(0);
        self.detect_collisions_with_skips(joint_poses, first_collision_only, &empty_set)
    }

    fn detect_collisions_with_skips(
        &self, joint_poses: &[Isometry3<f32>; 6], first_collision_only: bool, skip: &HashSet<usize>,
    ) -> Vec<(usize, usize)> {
        let mut tasks = Vec::with_capacity(self.count_tasks(&skip));

        // Check if the tool does not hit anything in the environment
        if let Some(tool) = &self.tool {
            for (env_idx, env_obj) in self.collision_environment.iter().enumerate() {
                tasks.push(CollisionTask {
                    i: J_TOOL,
                    j: ENV_START_IDX + env_idx,
                    transform_i: &joint_poses[J6],
                    transform_j: &env_obj.pose,
                    shape_i: &tool,
                    shape_j: &env_obj.mesh,
                });
            }
        }

        for i in 0..6 {
            for j in ((i + 1)..6).rev() {
                // If both joints did not move, we do not need to check
                if j - i > 1 && !skip.contains(&i) && !skip.contains(&j) {
                    tasks.push(CollisionTask {
                        i,
                        j,
                        transform_i: &joint_poses[i],
                        transform_j: &joint_poses[j],
                        shape_i: &self.joint_meshes[i],
                        shape_j: &self.joint_meshes[j],
                    });
                }
            }

            // Check if the joint does not hit anything in the environment
            for (env_idx, env_obj) in self.collision_environment.iter().enumerate() {
                // Joints we do not move we do not need to check for collision against objects
                // that also not move.
                if !skip.contains(&i) {
                    tasks.push(CollisionTask {
                        i,
                        j: ENV_START_IDX + env_idx,
                        transform_i: &joint_poses[i],
                        transform_j: &env_obj.pose,
                        shape_i: &self.joint_meshes[i],
                        shape_j: &env_obj.mesh,
                    });
                }
            }

            // Check if there is no collision between joint and tool
            if i != J6 && i != J5 {
                if let Some(tool) = &self.tool {
                    let accessory_pose = &joint_poses[J6];
                    tasks.push(CollisionTask {
                        i,
                        j: J_TOOL,
                        transform_i: &joint_poses[i],
                        transform_j: accessory_pose,
                        shape_i: &self.joint_meshes[i],
                        shape_j: &tool,
                    });
                }
            }

            // Base does not move, we do not need to check for collision against the joint
            // that also did not.
            if i != J1 && !skip.contains(&i) {
                if let Some(base) = &self.base {
                    let accessory = &base.mesh;
                    let accessory_pose = &base.base_pose;
                    tasks.push(CollisionTask {
                        i,
                        j: J_BASE,
                        transform_i: &joint_poses[i],
                        transform_j: accessory_pose,
                        shape_i: &self.joint_meshes[i],
                        shape_j: &accessory,
                    });
                }
            }
        }

        if let (Some(tool), Some(base)) = (&self.tool, &self.base) {
            tasks.push(CollisionTask {
                i: J_TOOL,
                j: J_BASE,
                transform_i: &joint_poses[J6],
                transform_j: &base.base_pose,
                shape_i: &tool,
                shape_j: &base.mesh,
            });
        }
        Self::process_collision_tasks(tasks, first_collision_only)
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
        let joints: [TriMesh; 6] = [
            create_trimesh(0.0, 0.0, 0.0),
            create_trimesh(0.01, 0.01, 0.01),
            create_trimesh(0.1, 0.1, 0.1),
            // Use local transform at places to be sure it works. This joint must be far away.
            transform_mesh(&create_trimesh(0.0, 0.0, 0.0),
                           &Isometry3::translation(20.0, 20.0, 20.0)),
            create_trimesh(30.0, 30.0, 30.0),
            // Place Joint 6 close to joint 1
            transform_mesh(&create_trimesh(0.0, 0.0, 0.0),
                           &Isometry3::translation(0.02, 0.02, 0.02)),
        ];

        let robot = RobotBody {
            joint_meshes: joints,
            tool: None,
            base: None,
            collision_environment: vec![],
            first_pose_only: false,
        };

        let collisions = robot.detect_collisions(&[identity; 6], false);
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
