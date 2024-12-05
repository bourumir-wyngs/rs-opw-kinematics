//! Implements collision detection

use crate::kinematic_traits::{
    Joints, Kinematics, Solutions, ENV_START_IDX, J1, J5, J6, J_BASE, J_TOOL,
};
use nalgebra::Isometry3;
use parry3d::shape::TriMesh;
use rayon::prelude::{IntoParallelRefIterator, ParallelIterator};
use std::collections::{HashMap, HashSet};

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
    TriMesh::new(
        shape
            .vertices()
            .iter()
            .map(|v| local_transform.transform_point(v))
            .collect(),
        shape.indices().to_vec(),
    )
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

impl CollisionTask<'_> {
    fn collides(&self, safety: &SafetyDistances) -> Option<(usize, usize)> {
        let r_min = *safety.min_distance(self.i, self.j);
        let collides = if r_min <= NEVER_COLLIDES {
            false
        } else if r_min == TOUCH_ONLY {
            parry3d::query::intersection_test(
                self.transform_i,
                self.shape_i,
                self.transform_j,
                self.shape_j,
            )
            .expect(SUPPORTED)
        } else {
            parry3d::query::distance(
                self.transform_i,
                self.shape_i,
                self.transform_j,
                self.shape_j,
            )
            .expect(SUPPORTED)
                <= r_min
        };
        if collides {
            Some((self.i.min(self.j), self.i.max(self.j)))
        } else {
            None
        }
    }
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

/// Constant specifying that robot parts never collide so do not need to be checked
/// for collision (so negative value used).
pub const NEVER_COLLIDES: f32 = -1.0;

/// Constant specifying that only interesection test must be done (any non zero distance
/// sufficient).
pub const TOUCH_ONLY: f32 = 0.0;

/// Defines tolerance bounds, how far it should be between any part of the robot,
/// or environment object, or any two parts of the robot. As some robot joints
/// may come very close together, they may require specialized distances.
pub struct SafetyDistances {
    /// Allowed distance between robot and environment objects.
    pub to_environment: f32,

    /// Default allowed distance between any two parts of the robot.
    pub to_robot_default: f32,

    /// Special cases where different (normally shorter) distance is allowed.
    /// Some parts of the robot naturally come very close even if not adjacent, and
    /// these need the shorter distance to be specified. Specify NEVER_COLLIDES as a distance
    /// for parts that cannot collide. Initialize it like this:
    ///
    /// ```
    ///    use std::collections::HashMap;
    ///    use rs_opw_kinematics::collisions::NEVER_COLLIDES;
    ///    use rs_opw_kinematics::kinematic_traits::{J1, J3, J5, J_BASE, J_TOOL};
    ///
    ///    // Always specify less numbered joints first, then
    ///    // the tool, then environment objects.
    ///    HashMap::from([
    ///       ( (J1, J5), 0.1 ), // J1 to J2 min distance 0.1 m
    ///       ( (J_BASE, J5), NEVER_COLLIDES), // J5 never collides with the base
    ///       ( (J_TOOL, J5), 0.2) // J5 to tool max 0.2
    ///    ])
    /// ```
    pub special_distances: HashMap<(usize, usize), f32>,

    /// If true, only checks till the first collision is found.
    pub first_collision_only: bool,
}

impl SafetyDistances {
    /// Returns minimal allowed distance by the specified objects.
    /// The order of objects is not important.
    pub fn min_distance(&self, from: usize, to: usize) -> &f32 {
        if let Some(r) = self.special_distances.get(&(from, to)) {
            return r;
        } else if let Some(r) = self.special_distances.get(&(to, from)) {
            return r;
        } else if from >= ENV_START_IDX || to >= ENV_START_IDX {
            return &self.to_environment;
        } else {
            return &self.to_robot_default;
        }
    }

    fn standard(first_collision_only: bool) -> SafetyDistances {
        SafetyDistances {
            to_environment: 0.0,
            to_robot_default: 0.0,
            special_distances: HashMap::new(),
            first_collision_only: first_collision_only,
        }
    }
}

// Public methods
impl RobotBody {
    /// Returns detailed information about all collisions detected in the robot's configuration.
    /// This method only checks for literally touching objects that limits its application.
    /// Use ```near``` to check if there are no objects closer than the given distance.
    pub fn collision_details(
        &self,
        qs: &Joints,
        kinematics: &dyn Kinematics,
    ) -> Vec<(usize, usize)> {
        let joint_poses = kinematics.forward_with_joint_poses(qs);
        let joint_poses_f32: [Isometry3<f32>; 6] = joint_poses.map(|pose| pose.cast::<f32>());
        self.detect_collisions(&joint_poses_f32, &SafetyDistances::standard(false))
    }

    /// Returns true if any collision is detected in the robot's configuration.
    /// This method only checks for literally touching objects that limits its application.
    /// Use ```near``` to check if there are no objects closer than the given distance.
    pub fn collides(&self, qs: &Joints, kinematics: &dyn Kinematics) -> bool {
        let joint_poses = kinematics.forward_with_joint_poses(qs);
        let joint_poses_f32: [Isometry3<f32>; 6] = joint_poses.map(|pose| pose.cast::<f32>());
        !self
            .detect_collisions(&joint_poses_f32, &SafetyDistances::standard(true))
            .is_empty()
    }

    /// Returns detailed information about all collisions detected in the robot's configuration.
    /// This method only checks for literally touching objects that limits its application.
    /// Use ```near``` to check if there are no objects closer than the given distance.
    pub fn near(
        &self,
        qs: &Joints,
        kinematics: &dyn Kinematics,
        safety_distances: &SafetyDistances,
    ) -> Vec<(usize, usize)> {
        let joint_poses = kinematics.forward_with_joint_poses(qs);
        let joint_poses_f32: [Isometry3<f32>; 6] = joint_poses.map(|pose| pose.cast::<f32>());
        self.detect_collisions(&joint_poses_f32, &safety_distances)
    }

    /// Return non colliding offsets, tweaking each joint plus minus either side, either into
    /// 'to' or into 'from'. This is required for planning algorithms like A*. We can do
    ///  less collision checks as we only need to check the joint branch of the robot we moved.
    ///  Offset generation is accelerated via Rayon.     
    pub fn non_colliding_offsets(
        &self,
        initial: &Joints,
        from: &Joints,
        to: &Joints,
        kinematics: &dyn Kinematics,
    ) -> Solutions {
        // Generate 12 tasks by tweaking each joint in either direction
        let mut tasks = Vec::with_capacity(12);
        for joint_index in 0..6 {
            for &target in &[from, to] {
                tasks.push((joint_index, target));
            }
        }

        // Process each task in parallel, filtering out colliding or out of constraints configurations
        tasks
            .par_iter()
            .filter_map(|&(joint_index, target)| {
                let mut new_joints = *initial;
                new_joints[joint_index] = target[joint_index];

                // Discard perturbations that go out of constraints.
                if let Some(constraints) = kinematics.constraints() {
                    if !constraints.compliant(&new_joints) {
                        return None;
                    }
                }

                // Generate the full joint poses for collision checking
                let joint_poses = kinematics.forward_with_joint_poses(&new_joints);
                let joint_poses_f32: [Isometry3<f32>; 6] =
                    joint_poses.map(|pose| pose.cast::<f32>());

                // Determine joints that do not require collision checks
                let skip_indices: HashSet<usize> = (0..joint_index).collect();

                // Detect collisions, skipping specified indices
                if self
                    .detect_collisions_with_skips(
                        &joint_poses_f32,
                        &SafetyDistances::standard(true),
                        &skip_indices,
                    )
                    .is_empty()
                {
                    return Some(new_joints); // Return non-colliding configuration
                } else {
                    return None;
                }
            })
            .collect() // Collect non-colliding configurations into Solutions
    }
}

const SUPPORTED: &'static str = "Mesh intersection should be supported by Parry3d";

impl RobotBody {
    /// Parallel version with Rayon
    fn process_collision_tasks(
        tasks: Vec<CollisionTask>,
        safety: &SafetyDistances,
    ) -> Vec<(usize, usize)> {
        if safety.first_collision_only {
            // Exit as soon as any collision is found
            tasks
                .par_iter()
                .find_map_any(|task| task.collides(&safety))
                .into_iter() // Converts the Option result to an iterator
                .collect()
        } else {
            // Collect all collisions
            tasks
                .par_iter()
                .filter_map(|task| task.collides(&safety))
                .collect()
        }
    }

    // Count exact number of tasks so we do not need to reallocate the vector.
    // This may return slightly larger number if skips are active.
    fn count_tasks(&self, skip: &HashSet<usize>) -> usize {
        if skip.len() >= 6 {
            panic!(
                "At most 5 joints can be skipped, but {} were passed: {:?}",
                skip.len(),
                skip
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
        tool_env_tasks
            + joint_joint_tasks
            + joint_env_tasks
            + joint_tool_tasks
            + joint_base_tasks
            + tool_base_task
    }

    fn detect_collisions(
        &self,
        joint_poses: &[Isometry3<f32>; 6],
        safety_distances: &SafetyDistances,
    ) -> Vec<(usize, usize)> {
        let empty_set: HashSet<usize> = HashSet::with_capacity(0);
        self.detect_collisions_with_skips(joint_poses, &safety_distances, &empty_set)
    }

    fn detect_collisions_with_skips(
        &self,
        joint_poses: &[Isometry3<f32>; 6],
        safety_distances: &SafetyDistances,
        skip: &HashSet<usize>,
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
        Self::process_collision_tasks(tasks, safety_distances)
    }
}

/// Magnify the TriMesh uniformly outward relative to its center of mass.
/*
fn magnify_trimesh(mesh: &mut TriMesh, magnification_factor: f32) {
    // Nested function to calculate the center of mass (centroid)
    fn calculate_mass_center(mesh: &TriMesh) -> Point3<f32> {
        let sum: Vector3<f32> = mesh
            .vertices()
            .iter()
            .map(|vertex| vertex.coords)
            .sum();

        // Calculate the average of the vertex positions and convert to Point3
        (sum / (mesh.vertices().len() as f32)).into()
    }

    // Compute the center of mass
    let center = calculate_mass_center(mesh);

    // Apply magnification to each vertex
    for vertex in mesh.vertices_mut() {
        let offset = *vertex - center;
        let scaled_offset = offset * magnification_factor;
        *vertex = center + scaled_offset;
    }
}
*/

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;
    use parry3d::shape::TriMesh;

    fn create_trimesh(x: f32, y: f32, z: f32) -> TriMesh {
        // Define vertices and triangle indices for a triangular pyramid.
        TriMesh::new(
            vec![
                Point3::new(z, y, z),
                Point3::new(x + 1.0, y, z),
                Point3::new(x, y + 1.0, z),
                Point3::new(x, y, z + 1.0),
            ],
            vec![[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
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
            transform_mesh(
                &create_trimesh(0.0, 0.0, 0.0),
                &Isometry3::translation(20.0, 20.0, 20.0),
            ),
            create_trimesh(30.0, 30.0, 30.0),
            // Place Joint 6 close to joint 1
            transform_mesh(
                &create_trimesh(0.0, 0.0, 0.0),
                &Isometry3::translation(0.02, 0.02, 0.02),
            ),
        ];

        let robot = RobotBody {
            joint_meshes: joints,
            tool: None,
            base: None,
            collision_environment: vec![],
            first_pose_only: false,
        };

        let collisions = robot.detect_collisions(&[identity; 6], false);
        assert!(
            !collisions.is_empty(),
            "Expected at least one collision, but none were detected."
        );

        // Now expect 4 collisions due to the placement of joint 6
        assert_eq!(collisions.len(), 4, "Expected exactly 4 collisions.");

        // Ensure that specific collisions are occurring, including the close ones
        let mut expected_collisions = vec![
            (0usize, 2usize),
            (0usize, 5usize),
            (1usize, 5usize),
            (2usize, 5usize),
        ];

        // Normalize the order of expected collisions (sort them)
        for collision in &mut expected_collisions {
            let (a, b) = *collision;
            if a > b {
                *collision = (b, a); // Ensure smaller index comes first
            }
        }

        for (shape_a, shape_b) in &collisions {
            let mut collision_names = (*shape_a, *shape_b);
            if collision_names.0 > collision_names.1 {
                collision_names = (collision_names.1, collision_names.0); // Ensure smaller index comes first
            }

            assert!(
                expected_collisions.contains(&collision_names),
                "Unexpected collision between [{}] and [{}]",
                shape_a,
                shape_b
            );
        }

        // Ensure that shape 3 is NOT involved in collisions
        for (shape_a, shape_b) in &collisions {
            assert_ne!(
                *shape_a, 3,
                "Shape [3] should not be involved in any collision."
            );
            assert_ne!(
                *shape_b, 3,
                "Shape [3] should not be involved in any collision."
            );
        }
    }
}
