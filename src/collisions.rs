use nalgebra::{Isometry3};
use parry3d::query::contact;
use parry3d::shape::TriMesh;
use crate::joint_body::{BaseBody, CollisionBody};
use crate::kinematic_traits::{Joints, Kinematics, J1, J5, J6};

/// Struct representing the geometry of a robot, which is composed of exactly 6 joints.
/// Each joint is represented by a `JointBody`, encapsulating the geometrical shape 
/// of a joint. This struct also provides functionality for detecting collisions 
/// between these joints. This structure does not specify the exact pose of the robot,
/// it is used to build a `PositionedRobot` that does.
pub struct RobotBody {
    /// A fixed-size array of 6 `JointBody` instances, each representing 
    /// the geometrical shape of a joint in the robot.
    pub joint_meshes: [TriMesh; 6],

    /// Optional tool that, if exists, is attached to the last joint. Being JointBody, it can be
    /// attached with the local transform. It has the same global transform as the joint J6
    /// (that is usually a small flange only)
    pub tool: Option<TriMesh>,

    /// Optional structure attached to the robot base joint. It has its own global transform
    /// that brings the robot to the location. This structure includes two transforms,
    /// one bringing us to the base of the stand supporting the robot (and this is the 
    /// pose of the stand itself), and then another defining the point where J1 rotating part
    /// begins.
    pub base: Option<BaseBody>,

    /// The threshold distance used in collision detection. 
    /// If the distance between two geometries is less than this value, they are considered colliding.
    pub collision_tolerance: f32,

    /// A boolean flag indicating whether the collision detection 
    /// should stop after the first detected collision. When set to `true`, the system will halt 
    /// detection as soon as one collision is found; otherwise, all potential collisions will be checked.
    pub detect_first_collision_only: bool,

    /// Static objects against that we check the robot does not collide.
    pub collision_environment: Vec<CollisionBody>,
}

/// The number for the robot tool in collision report
const J_TOOL: usize = 100;

/// The robot base joint
const J_BASE: usize = 101;

/// Starting index for collision_environment entries in collision pairs
const ENV_START_IDX: usize = 100_000;


impl RobotBody {
    pub fn detect_collisions(
        &self, joint_poses: &[Isometry3<f32>; 6],
    ) -> Vec<(usize, usize)> {
        let mut collisions = Vec::new();

        // Loop through joints and ensure each joint pair is checked only once
        for i in 0..6 {
            for j in (i + 1)..6 {
                if j - i > 1 {   // Skip adjacent joints
                    let joint_1_mesh = &self.joint_meshes[i];
                    let joint_2_mesh = &self.joint_meshes[j];

                    if self.check_collision(
                        i, j,
                        &joint_poses[i], &joint_poses[j],
                        &joint_1_mesh, &joint_2_mesh,
                        &mut collisions,
                    ) {
                        return collisions;
                    }
                }
            }

            // Check each joint against static objects in collision_environment
            for (env_idx, env_obj) in self.collision_environment.iter().enumerate() {
                if self.check_collision(
                    i, ENV_START_IDX + env_idx,
                    &joint_poses[i], &env_obj.pose,
                    &self.joint_meshes[i], &env_obj.mesh,
                    &mut collisions,
                ) {
                    return collisions;
                }
            }

            // Check collisions with `self.tool` if it is defined (tool is same as J6
            // and attached to J5)
            if i != J6 && i != J5 {
                if let Some(tool) = &self.tool {
                    if self.check_accessory_collisions(i, J_TOOL, tool, joint_poses,
                                                       &joint_poses[J6],
                                                       &mut collisions) {
                        return collisions;
                    }
                }
            }

            // Check collisions with `self.base` if it is defined (base is attached to J1)
            if i != J1 {
                if let Some(base) = &self.base {
                    if self.check_accessory_collisions(i, J_BASE, &base.mesh,
                                                       joint_poses, &base.base_pose, &mut collisions) {
                        return collisions;
                    }
                }
            }
        }

        // Additional check for direct collision between tool and base, if both are defined
        if let (Some(tool), Some(base)) = (&self.tool, &self.base) {
            if self.check_collision(
                J_TOOL, J_BASE,
                &joint_poses[J6], &base.base_pose,
                &tool, &base.mesh,
                &mut collisions,
            ) {
                return collisions;
            }
        }

        collisions
    }

    fn check_collision(&self,
                       i: usize, j: usize,
                       transform_i: &Isometry3<f32>, transform_j: &Isometry3<f32>,
                       shape_i: &TriMesh, shape_j: &TriMesh,
                       collisions: &mut Vec<(usize, usize)>,
    ) -> bool {
        // Check for initial collision using simplified shapes
        let simplified_contact = contact(
            transform_i, shape_i,
            transform_j, shape_j,
            self.collision_tolerance,
        );

        if simplified_contact.is_ok() && simplified_contact.unwrap().is_some() {
            // Perform detailed collision check on the full shapes
            let shape_contact = contact(
                transform_i, shape_i,
                transform_j, shape_j,
                self.collision_tolerance,
            );

            if let Ok(Some(_)) = shape_contact {
                // Add collision with ordered indices (lower index first)
                collisions.push((i.min(j), i.max(j)));
                if self.detect_first_collision_only {
                    return true; // Exit if only first collision is needed
                }
            }
        }
        false
    }

    fn check_accessory_collisions(
        &self, joint_idx: usize,
        acc_reporting_code: usize,
        accessory: &TriMesh,
        joint_poses: &[Isometry3<f32>; 6],
        accessory_pose: &Isometry3<f32>,
        collisions: &mut Vec<(usize, usize)>,
    ) -> bool {
        if self.check_collision(
            joint_idx, acc_reporting_code,
            &joint_poses[joint_idx], accessory_pose,
            &self.joint_meshes[joint_idx], &accessory,
            collisions,
        ) {
            return true;
        }

        // Check the tool against static objects in collision_environment
        for (env_idx, env_obj) in self.collision_environment.iter().enumerate() {
            if self.check_collision(
                acc_reporting_code, ENV_START_IDX + env_idx,
                &accessory_pose, &env_obj.pose,
                &accessory, &env_obj.mesh, collisions,
            ) {
                return true;
            }
        }

        false
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
    use crate::joint_body::transform_mesh;
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
            collision_tolerance: 0.0,
            detect_first_collision_only: false,
            collision_environment: vec![],
        };

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
