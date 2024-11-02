use nalgebra::Isometry3;
use parry3d::shape::TriMesh;
use rayon::prelude::{IntoParallelRefIterator, ParallelIterator};
use crate::joint_body::{BaseBody, CollisionBody};
use crate::kinematic_traits::{Joints, Kinematics, ENV_START_IDX, J1, J5, J6, J_BASE, J_TOOL};

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
    pub joint_meshes: [TriMesh; 6],
    pub tool: Option<TriMesh>,
    pub base: Option<BaseBody>,
    pub collision_environment: Vec<CollisionBody>,
}

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
}

impl RobotBody {
    
    /// Parallel version with Rayon
    fn process_collision_tasks(tasks: Vec<CollisionTask>, first_collision_only: bool) -> Vec<(usize, usize)> {
        if first_collision_only {
            // Exit as soon as any collision is found
            tasks.par_iter()
                .find_map_any(|task| {
                    let collides = parry3d::query::intersection_test(
                        task.transform_i, task.shape_i, task.transform_j, task.shape_j)
                        .expect("Mesh intersection must be supported");
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
                        .expect("Mesh intersection must be supported");
                    if collides {
                        Some((task.i.min(task.j), task.i.max(task.j)))
                    } else {
                        None
                    }
                })
                .collect()
        }
    }
    
    /// Sequential version
    fn process_collision_tasks_sequential(tasks: Vec<CollisionTask>, first_collision_only: bool) -> Vec<(usize, usize)> {
        let mut collisions = Vec::with_capacity(4);

        for task in tasks {
            if parry3d::query::intersection_test(task.transform_i, task.shape_i, task.transform_j, task.shape_j)
                .expect("Mesh intersection must be supported")
            {
                collisions.push((task.i.min(task.j), task.i.max(task.j)));
                if first_collision_only {
                    break;
                }
            }
        }

        collisions
    }


    fn detect_collisions(
        &self, joint_poses: &[Isometry3<f32>; 6], first_collision_only: bool,
    ) -> Vec<(usize, usize)> {
        let mut tasks = Vec::with_capacity(64);

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
                if j - i > 1 {
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

            for (env_idx, env_obj) in self.collision_environment.iter().enumerate() {
                tasks.push(CollisionTask {
                    i,
                    j: ENV_START_IDX + env_idx,
                    transform_i: &joint_poses[i],
                    transform_j: &env_obj.pose,
                    shape_i: &self.joint_meshes[i],
                    shape_j: &env_obj.mesh,
                });
            }

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

            if i != J1 {
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

        println!("{} collisions to check", tasks.len());

        Self::process_collision_tasks(tasks, first_collision_only)
    }

    fn check_collision(&self, task: CollisionTask) -> bool {
        let collides = parry3d::query::intersection_test(
            task.transform_i, task.shape_i,
            task.transform_j, task.shape_j,
        );

        if collides.expect("Mesh intersection must be supported") {
            return true;
        }
        false
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
            collision_environment: vec![],
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
