//! Implements collision detection

use crate::kinematic_traits::{
    Joints, Kinematics, Solutions, ENV_START_IDX, J1, J5, J6, J_BASE, J_TOOL,
};

use nalgebra::Isometry3;
use parry3d::shape::TriMesh;
use rayon::prelude::{IntoParallelRefIterator, ParallelIterator};
use std::collections::{HashMap, HashSet};
use parry3d::bounding_volume::{Aabb, BoundingVolume};
use parry3d::math::Point;

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
    i: u16, // reporting index of the first shape
    j: u16, // reporting index of the second shape
    transform_i: &'a Isometry3<f32>,
    transform_j: &'a Isometry3<f32>,
    shape_i: &'a TriMesh,
    shape_j: &'a TriMesh,
}

impl CollisionTask<'_> {
    fn collides(&self, safety: &SafetyDistances) -> Option<(u16, u16)> {
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
            // Check if the bounding boxe of the object i, enlarged by r_min, touches
            // the object j. If not, objects are more than r_min apart.
            let (sm_shape, sm_transform, bg_shape, bg_transform) = 
                if self.shape_i.vertices().len() < self.shape_j.vertices().len() {
                (self.shape_i, self.transform_i, self.shape_j, self.transform_j)
            } else {
                (self.shape_j, self.transform_j, self.shape_i, self.transform_i)
            };            
            // Small shape is simplified to aabb that is then enlarged. Large shape is used
            // as is (it probably has a complex shape and would result in many false positives
            // if similarly simplified            
            let am_aaabb = sm_shape.local_aabb().loosened(r_min);
            let sm_abb_mesh = build_trimesh_from_aabb(am_aaabb);
            if !parry3d::query::intersection_test(
                sm_transform,
                &sm_abb_mesh,
                bg_transform,
                bg_shape,
            ).expect(SUPPORTED) {
                false
            } else {
                parry3d::query::distance(
                    self.transform_i,
                    self.shape_i,
                    self.transform_j,
                    self.shape_j,
                )
                    .expect(SUPPORTED)
                    <= r_min
            }
        };
        
        if collides {
            Some((self.i.min(self.j), self.i.max(self.j)))
        } else {
            None
        }
    }
}

/// Parry does not support AABB as a "proper" shape so we rewrap it as mesh
pub fn build_trimesh_from_aabb(aabb: Aabb) -> TriMesh {
    let min: Point<f32> = aabb.mins;
    let max: Point<f32> = aabb.maxs;
    // Define the 8 vertices of the AABB
    let vertices = vec![
        min, // 0
        Point::new(max.x, min.y, min.z), // 1
        Point::new(min.x, max.y, min.z), // 2
        Point::new(max.x, max.y, min.z), // 3
        Point::new(min.x, min.y, max.z), // 4
        Point::new(max.x, min.y, max.z), // 5
        Point::new(min.x, max.y, max.z), // 6
        max, // 7
    ];

    // Define the 12 triangles (2 for each face)
    const INDICES: [[u32; 3]; 12] = [
        // Bottom face (min.z)
        [0, 1, 2],
        [2, 1, 3],

        // Top face (max.z)
        [4, 5, 6],
        [6, 5, 7],

        // Front face (max.y)
        [2, 3, 6],
        [6, 3, 7],

        // Back face (min.y)
        [0, 1, 4],
        [4, 1, 5],

        // Left face (min.x)
        [0, 2, 4],
        [4, 2, 6],

        // Right face (max.x)
        [1, 3, 5],
        [5, 3, 7],
    ];

    // Return TriMesh
    TriMesh::new(vertices, INDICES.to_vec())
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

    /// Defines distances, how far the robot must stay from collision objects.
    /// Also specifies if we are interested in first collision only (like for path planning)
    /// or we need a detailed overview (like for diagnostics or visualization)
    pub safety: SafetyDistances,
}

/// Constant specifying that robot parts never collide so do not need to be checked
/// for collision (so negative value used).
pub const NEVER_COLLIDES: f32 = -1.0;

/// Constant specifying that only interesection test must be done (any non zero distance
/// sufficient).
pub const TOUCH_ONLY: f32 = 0.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CheckMode {
    FirstCollisionOnly,
    AllCollsions,
    NoCheck,
}

/// Defines tolerance bounds, how far it should be between any part of the robot,
/// or environment object, or any two parts of the robot. As some robot joints
/// may come very close together, they may require specialized distances.
#[derive(Clone, Debug)]
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
    ///    use rs_opw_kinematics::collisions::{SafetyDistances, NEVER_COLLIDES};
    ///    use rs_opw_kinematics::kinematic_traits::{J1, J2, J3, J4, J5, J6, J_BASE, J_TOOL};
    ///
    ///    // Always specify less numbered joints first, then
    ///    // the tool, then environment objects.
    ///     SafetyDistances::distances(&[
    ///       // Due construction of this robot, these joints are very close, so
    ///       // special rules are needed for them.
    ///       ((J2, J_BASE), NEVER_COLLIDES), // base and J2 cannot collide
    ///       ((J3, J_BASE), NEVER_COLLIDES), // base and J3 cannot collide
    ///       ((J2, J4), NEVER_COLLIDES),
    ///       ((J3, J4), NEVER_COLLIDES),
    ///       ((J4, J_TOOL), 0.02_f32), // reduce distance requirement to 2 cm.
    ///       ((J4, J6), 0.02_f32),     // reduce distance requirement to 2 cm.
    ///       ]);
    /// ```
    pub special_distances: HashMap<(u16, u16), f32>,

    /// Specifies if either first collision only is required, or all must be checked, or off,
    /// or "touch only" mode
    pub mode: CheckMode,
}

impl SafetyDistances {
    // Converts from usize to much more compact and appropriate u16.
    // In Rust, usize is required for indexing.
    pub fn distances(pairs: &[((usize, usize), f32)]) -> HashMap<(u16, u16), f32> {
        let mut result = HashMap::with_capacity(pairs.len());

        for &((a, b), value) in pairs {
            // Cast `usize` to `u16` and insert into the HashMap
            result.insert((a as u16, b as u16), value);
        }
        result
    }

    /// Returns minimal allowed distance by the specified objects.
    /// The order of objects is not important.
    pub fn min_distance(&self, from: u16, to: u16) -> &f32 {
        if let Some(r) = self.special_distances.get(&(from, to)) {
            return r;
        } else if let Some(r) = self.special_distances.get(&(to, from)) {
            return r;
        } else if from as usize >= ENV_START_IDX || to as usize >= ENV_START_IDX {
            return &self.to_environment;
        } else {
            return &self.to_robot_default;
        }
    }

    /// Creates the standard instance of safety distances that always uses touch check only
    /// (no safety margins) but can also disable collision checks completely if you pass
    /// ```CheckMode::NoCheck``` as parameter.
    pub fn standard(mode: CheckMode) -> SafetyDistances {
        SafetyDistances {
            to_environment: TOUCH_ONLY,
            to_robot_default: TOUCH_ONLY,
            special_distances: HashMap::new(),
            mode,
        }
    }
}

// Public methods
impl RobotBody {
    /// Returns detailed information about all collisions detected in the robot's configuration.
    /// This method uses default distance limits specified at creation.
    /// Use ```near``` if you need to change limits frequently as the part of your algorithm.
    pub fn collision_details(
        &self,
        qs: &Joints,
        kinematics: &dyn Kinematics,
    ) -> Vec<(usize, usize)> {
        let joint_poses = kinematics.forward_with_joint_poses(qs);
        let joint_poses_f32: [Isometry3<f32>; 6] = joint_poses.map(|pose| pose.cast::<f32>());
        self.detect_collisions(&joint_poses_f32, &self.safety, None)
    }

    /// Returns true if any collision is detected in the robot's configuration.
    /// This method uses default distance limits specified at creation.
    /// Use ```near``` if you need to change limits frequently as the part of your algorithm.
    pub fn collides(&self, qs: &Joints, kinematics: &dyn Kinematics) -> bool {
        if self.safety.mode == CheckMode::NoCheck {
            return false;
        }
        let joint_poses = kinematics.forward_with_joint_poses(qs);
        let joint_poses_f32: [Isometry3<f32>; 6] = joint_poses.map(|pose| pose.cast::<f32>());
        let safety = &self.safety;
        let override_mode = Some(CheckMode::FirstCollisionOnly);
        let empty_set: HashSet<usize> = HashSet::with_capacity(0);
        !self
            .detect_collisions_with_skips(&joint_poses_f32, &safety, &override_mode, &empty_set)
            .is_empty()
    }

    /// Check for collisions exclusing some links (J_TOOL may need to be excluded
    /// if the robot touches the surface with it while working but now allowed when
    /// relocating
    pub fn collides_except(
        &self,
        qs: &Joints,
        kinematics: &dyn Kinematics,
        skips: &HashSet<usize>,
    ) -> bool {
        if self.safety.mode == CheckMode::NoCheck {
            return false;
        }
        let joint_poses = kinematics.forward_with_joint_poses(qs);
        let joint_poses_f32: [Isometry3<f32>; 6] = joint_poses.map(|pose| pose.cast::<f32>());
        let safety = &self.safety;
        let override_mode = Some(CheckMode::FirstCollisionOnly);
        !self
            .detect_collisions_with_skips(&joint_poses_f32, &safety, &override_mode, &skips)
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
        self.detect_collisions(&joint_poses_f32, &safety_distances, None)
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
                        &self.safety,
                        &Some(CheckMode::FirstCollisionOnly),
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

pub(crate) const SUPPORTED: &'static str = "Mesh intersection should be supported by Parry3d";

impl RobotBody {
    /// Parallel version with Rayon
    fn process_collision_tasks(
        tasks: Vec<CollisionTask>,
        safety: &SafetyDistances,
        override_mode: &Option<CheckMode>,
    ) -> Vec<(u16, u16)> {
        let mode = override_mode.unwrap_or(safety.mode);

        if mode == CheckMode::NoCheck {
            Vec::new()
        } else if mode == CheckMode::FirstCollisionOnly {
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
        safety: &SafetyDistances,
        override_mode: Option<CheckMode>,
    ) -> Vec<(usize, usize)> {
        let empty_set: HashSet<usize> = HashSet::with_capacity(0);
        // Convert to usize
        self.detect_collisions_with_skips(joint_poses, &safety, &override_mode, &empty_set)
            .iter()
            .map(|&col_pair| (col_pair.0 as usize, col_pair.1 as usize))
            .collect()
    }

    fn detect_collisions_with_skips(
        &self,
        joint_poses: &[Isometry3<f32>; 6],
        safety_distances: &SafetyDistances,
        override_mode: &Option<CheckMode>,
        skip: &HashSet<usize>,
    ) -> Vec<(u16, u16)> {
        let mut tasks = Vec::with_capacity(self.count_tasks(&skip));

        // Check if the tool does not hit anything in the environment
        let check_tool = !skip.contains(&J_TOOL);
        if check_tool {
            if let Some(tool) = &self.tool {
                for (env_idx, env_obj) in self.collision_environment.iter().enumerate() {
                    if self.check_required(J_TOOL, (ENV_START_IDX + env_idx) as usize, &skip) {
                        tasks.push(CollisionTask {
                            i: J_TOOL as u16,
                            j: (ENV_START_IDX + env_idx) as u16,
                            transform_i: &joint_poses[J6],
                            transform_j: &env_obj.pose,
                            shape_i: &tool,
                            shape_j: &env_obj.mesh,
                        });
                    }
                }
            }
        }

        for i in 0..6 {
            for j in ((i + 1)..6).rev() {
                // If both joints did not move, we do not need to check
                if j - i > 1 && self.check_required(i, j, &skip) {
                    tasks.push(CollisionTask {
                        i: i as u16,
                        j: j as u16,
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
                if self.check_required(i, ENV_START_IDX + env_idx, &skip) {
                    tasks.push(CollisionTask {
                        i: i as u16,
                        j: (ENV_START_IDX + env_idx) as u16,
                        transform_i: &joint_poses[i],
                        transform_j: &env_obj.pose,
                        shape_i: &self.joint_meshes[i],
                        shape_j: &env_obj.mesh,
                    });
                }
            }

            // Check if there is no collision between joint and tool
            if check_tool && i != J6 && i != J5 && self.check_required(i, J_TOOL, &skip) {
                if let Some(tool) = &self.tool {
                    let accessory_pose = &joint_poses[J6];
                    tasks.push(CollisionTask {
                        i: i as u16,
                        j: J_TOOL as u16,
                        transform_i: &joint_poses[i],
                        transform_j: accessory_pose,
                        shape_i: &self.joint_meshes[i],
                        shape_j: &tool,
                    });
                }
            }

            // Base does not move, we do not need to check for collision against the joint
            // that also did not.
            if i != J1 && !skip.contains(&i) && self.check_required(i, J1, &skip) {
                if let Some(base) = &self.base {
                    let accessory = &base.mesh;
                    let accessory_pose = &base.base_pose;
                    tasks.push(CollisionTask {
                        i: i as u16,
                        j: J_BASE as u16,
                        transform_i: &joint_poses[i],
                        transform_j: accessory_pose,
                        shape_i: &self.joint_meshes[i],
                        shape_j: &accessory,
                    });
                }
            }
        }

        // Check tool-base collision if necessary
        if check_tool || self.check_required(J_TOOL, J_BASE, &skip) {
            if let (Some(tool), Some(base)) = (&self.tool, &self.base) {
                tasks.push(CollisionTask {
                    i: J_TOOL as u16,
                    j: J_BASE as u16,
                    transform_i: &joint_poses[J6],
                    transform_j: &base.base_pose,
                    shape_i: &tool,
                    shape_j: &base.mesh,
                });
            }
        }
        Self::process_collision_tasks(tasks, safety_distances, override_mode)
    }

    fn check_required(&self, i: usize, j: usize, skip: &HashSet<usize>) -> bool {
        !skip.contains(&i) && !skip.contains(&j) &&
            self.safety.min_distance(i as u16, j as u16) > &NEVER_COLLIDES
    }    
}

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
            safety: SafetyDistances::standard(CheckMode::AllCollsions),
        };

        let collisions = robot.detect_collisions(&[identity; 6], &robot.safety, None);
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
