//! Provides visualization window with sliders for angles and pose.
//!
//! <img src="https://github.com/user-attachments/assets/3a0e6293-519e-455f-bf8b-0ff1090103b1" alt="screenshot" width="300"/>
//!
//! The [`crate::kinematics_with_shape::KinematicsWithShape`] structure fully integrates kinematics
//! with 3D meshes representing the robot, making it straightforward to visualize.
//! To display the robot, simply pass this structure to the built-in function
//! [`visualize_robot`].  
//!
//! ```ignore
//! fn main() {
//!     use std::ops::RangeInclusive;
//!     use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
//!     use rs_opw_kinematics::visualization;
//!
//!     // See `complete_visible_robot example` how to build this structure
//!     let robot: KinematicsWithShape = ...
//!
//!     // Define the initial joint angles to display the robot in a specific position
//!     let initial_angles = [173., -8., -94., 6., 83., 207.];
//!
//!     // Define boundaries for XYZ drawbars in the visualization GUI
//!     let tcp_box: [RangeInclusive<f64>; 3] = [-2.0..=2.0, -2.0..=2.0, 1.0..=2.0];
//!
//!     visualization::visualize_robot(robot, initial_angles, tcp_box);
//! }
//! ```
//!
//! ### Purpose
//! Visualization serves as a verification tool to ensure that parameters, meshes, tool, and base setup are correct,
//! rather than as a production feature. Using Bevy, this visualization displays the robot (mounted on its base
//! and equipped with the tool), various environment objects, and manipulable handles for the robot.
//!
//! ### Features
//! In the visualization window, joint positions can be adjusted for forward kinematics, or the tool center point
//! can be set using Cartesian coordinates for inverse kinematics.
//!
//! - **Collision Detection**: Active in both kinematics modes but functions differently:
//!   - **Inverse Kinematics**: Movement is restricted to avoid collisions entirely (the robot will not move if a
//!     collision would occur).
//!   - **Forward Kinematics**: Collisions are permitted, but colliding robot joints and environment objects are highlighted.

use crate::camera_controller::{camera_controller_system, CameraController};
use crate::collisions::SafetyDistances;
use crate::kinematic_traits::{Joints, Kinematics, Pose, ENV_START_IDX, J_BASE, J_TOOL};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::pose::Pose32;
use crate::utils;
use bevy::asset::RenderAssetUsages;
use bevy::mesh::Indices;
use bevy::prelude::*;
use bevy::render::render_resource::PrimitiveTopology;
use bevy::winit::WinitPlugin;
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use glam::{DQuat, DVec3, EulerRot};
use parry3d::shape::TriMesh;
use std::collections::HashSet;
use std::ops::RangeInclusive;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Instant;

// Convert a parry3d `TriMesh` into a bevy `Mesh` for rendering
fn trimesh_to_bevy_mesh(trimesh: &TriMesh) -> Mesh {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    );

    // Step 1: Extract vertices and indices from the TriMesh
    let vertices: Vec<_> = trimesh.vertices().iter().map(|v| [v.x, v.y, v.z]).collect();
    let indices: Vec<_> = trimesh
        .indices()
        .iter()
        .flat_map(|i| i.to_vec())
        .collect::<Vec<_>>();

    // Step 2: Initialize normal vectors for each vertex
    let mut normals: Vec<[f32; 3]> = vec![[0.0, 0.0, 0.0]; vertices.len()];

    // Step 3: Calculate normals for each face (triangle)
    for triangle in trimesh.indices() {
        let i0 = triangle[0] as usize;
        let i1 = triangle[1] as usize;
        let i2 = triangle[2] as usize;

        // Get the three vertices of the triangle
        let v0 = Vec3::from_array(vertices[i0]);
        let v1 = Vec3::from_array(vertices[i1]);
        let v2 = Vec3::from_array(vertices[i2]);

        // Calculate the two edge vectors
        let edge1 = v1 - v0;
        let edge2 = v2 - v0;

        // Calculate the normal using the cross product of the two edges
        let normal = edge1.cross(edge2).normalize_or_zero();

        for &i in &[i0, i1, i2] {
            normals[i][0] += normal.x;
            normals[i][1] += normal.y;
            normals[i][2] += normal.z;
        }
    }

    // Step 4: Normalize all vertex normals
    for normal in normals.iter_mut() {
        let length = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
        if length > 0.0 {
            normal[0] /= length;
            normal[1] /= length;
            normal[2] /= length;
        }
    }

    // Step 5: Insert attributes into the mesh
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vertices);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals); // Add the flipped normals

    // Step 6: Set the indices
    mesh.insert_indices(Indices::U32(indices));

    mesh
}

#[derive(Component)]
struct RenderedRobotPart {
    collision_index: usize,
}

struct RobotPartRenderState {
    collision_index: usize,
    transform: Transform,
    material: Handle<StandardMaterial>,
}

enum VisualizationCommand {
    SetJointAngles([f32; 6]),
    Close,
}

#[derive(Resource)]
struct VisualizationCommands {
    receiver: Mutex<Receiver<VisualizationCommand>>,
}

/// Handle for a non-blocking visualization window.
///
/// The handle can update the displayed robot joint angles and request that the
/// Bevy window closes. Dropping the handle also sends a close request.
pub struct VisualizationHandle {
    sender: Sender<VisualizationCommand>,
    join_handle: Arc<Mutex<Option<JoinHandle<()>>>>,
}

impl VisualizationHandle {
    /// Set the displayed robot joint angles. Angles are degrees, matching the
    /// visualization UI sliders.
    pub fn set_joint_angles(&self, joint_angles: [f32; 6]) -> Result<(), String> {
        self.sender
            .send(VisualizationCommand::SetJointAngles(joint_angles))
            .map_err(|_| "visualization window is not running".to_string())
    }

    /// Request the visualization window to close and wait for its thread to exit.
    pub fn close(&self) -> Result<(), String> {
        let _ = self.sender.send(VisualizationCommand::Close);
        let mut join_handle = self
            .join_handle
            .lock()
            .map_err(|_| "visualization handle lock is poisoned".to_string())?;
        if let Some(join_handle) = join_handle.take() {
            join_handle
                .join()
                .map_err(|_| "visualization thread panicked".to_string())?;
        }
        Ok(())
    }

    /// Returns false after the visualization thread has finished.
    pub fn is_running(&self) -> bool {
        match self.join_handle.lock() {
            Ok(join_handle) => join_handle
                .as_ref()
                .is_some_and(|join_handle| !join_handle.is_finished()),
            Err(_) => false,
        }
    }
}

impl Drop for VisualizationHandle {
    fn drop(&mut self) {
        let _ = self.sender.send(VisualizationCommand::Close);
    }
}

/// Data to store the current joint angles and TCP as they are shown in control panel
#[derive(Resource)]
struct RobotControls {
    joint_angles: [f32; 6],
    tcp: [f64; 6],
    tcp_box: [RangeInclusive<f64>; 3], // Boundaries for sliders in GUI to move the tool center point arround
    initial_joint_angles: [f32; 6],    // Initial angles at the start of visualization
    previous_joint_angles: [f32; 6],   // Store previous joint angles here
    previous_tcp: [f64; 6],
    safety_distance: f32,
    previous_safety_distance: f32,
    joint_angles_changed: bool,
    tcp_changed: bool,
    safety_distance_changed: bool,
}

impl RobotControls {
    fn set_tcp_from_pose(&mut self, pose: &Pose) {
        let (z_angle, y_angle, x_angle) = pose.rotation.to_euler(EulerRot::ZYX);
        self.tcp = [
            pose.translation.x,
            pose.translation.y,
            pose.translation.z,
            z_angle.to_degrees(),
            y_angle.to_degrees(),
            x_angle.to_degrees(),
        ];
    }

    fn pose(&self) -> Pose {
        fn quat_from_euler(this: &RobotControls) -> DQuat {
            let z_angle = this.tcp[3].to_radians();
            let y_angle = this.tcp[4].to_radians();
            let x_angle = this.tcp[5].to_radians();

            // Preserve the existing UI angle order: Z, then Y, then X.
            DQuat::from_euler(EulerRot::ZYX, z_angle, y_angle, x_angle)
        }

        Pose::from_parts(
            DVec3::new(self.tcp[0], self.tcp[1], self.tcp[2]),
            quat_from_euler(self),
        )
    }
}

// Resource to store the current robot instance
#[derive(Resource)]
struct Robot {
    kinematics: KinematicsWithShape,
    safety: SafetyDistances,
    joint_meshes: Option<[Handle<Mesh>; 6]>, // Precomputed and converted meshes
    material: Option<Handle<StandardMaterial>>,
    tool_material: Option<Handle<StandardMaterial>>,
    base_material: Option<Handle<StandardMaterial>>,
    environment_material: Option<Handle<StandardMaterial>>,
    colliding_material: Option<Handle<StandardMaterial>>, // Highlight colliding components in color
    tool: Option<Handle<Mesh>>,
    base: Option<Handle<Mesh>>,
    environment: Vec<Handle<Mesh>>, // Environment objects
}

/// Visualize the given robot, starting from the given initial angles (given in degrees)
/// The sliders for specifying the tool center point location use the boundaries
/// from the tcp_box (given in meters). Bevy will be used for visualization.
pub fn visualize_robot(
    robot: KinematicsWithShape,
    intial_angles: [f32; 6],
    tcp_box: [RangeInclusive<f64>; 3],
) {
    let safety = robot.body.safety.clone();
    visualize_robot_with_safety(robot, intial_angles, tcp_box, &safety)
}

pub fn visualize_robot_with_safety(
    robot: KinematicsWithShape,
    intial_angles: [f32; 6],
    tcp_box: [RangeInclusive<f64>; 3],
    safety_distances: &SafetyDistances,
) {
    App::new()
        .add_plugins((DefaultPlugins, EguiPlugin::default()))
        .insert_resource(RobotControls {
            initial_joint_angles: intial_angles,
            joint_angles: intial_angles,
            tcp: [0.0; 6],
            tcp_box,
            previous_joint_angles: intial_angles,
            previous_tcp: [0.0; 6],
            safety_distance: 0.05,
            previous_safety_distance: 0.0,
            joint_angles_changed: false,
            tcp_changed: false,
            safety_distance_changed: false,
        })
        .insert_resource(Robot {
            kinematics: robot,
            safety: safety_distances.clone(),
            joint_meshes: None,
            tool: None,
            base: None,
            environment: Vec::new(),
            material: None,
            tool_material: None,
            base_material: None,
            environment_material: None,
            colliding_material: None,
        })
        .add_systems(Startup, setup) // Register setup system in Startup phase
        .add_systems(Update, (update_robot, camera_controller_system)) // Add systems without .system()
        .add_systems(EguiPrimaryContextPass, control_panel)
        .run();
}

/// Start visualization on a background thread and return a control handle.
///
/// Joint angles are degrees, matching the visualization UI sliders.
pub fn visualize_robot_async(
    robot: KinematicsWithShape,
    initial_angles: [f32; 6],
    tcp_box: [RangeInclusive<f64>; 3],
) -> VisualizationHandle {
    let safety = robot.body.safety.clone();
    visualize_robot_with_safety_async(robot, initial_angles, tcp_box, &safety)
}

/// Start visualization on a background thread with custom safety distances and
/// return a control handle.
///
/// Joint angles are degrees, matching the visualization UI sliders.
pub fn visualize_robot_with_safety_async(
    robot: KinematicsWithShape,
    initial_angles: [f32; 6],
    tcp_box: [RangeInclusive<f64>; 3],
    safety_distances: &SafetyDistances,
) -> VisualizationHandle {
    let (sender, receiver) = mpsc::channel();
    let safety_distances = safety_distances.clone();
    let join_handle = thread::spawn(move || {
        App::new()
            .add_plugins((
                DefaultPlugins.set(WinitPlugin {
                    // The async visualizer intentionally runs Bevy off the
                    // caller's thread; winit requires this explicit opt-in.
                    run_on_any_thread: true,
                }),
                EguiPlugin::default(),
            ))
            .insert_resource(RobotControls {
                initial_joint_angles: initial_angles,
                joint_angles: initial_angles,
                tcp: [0.0; 6],
                tcp_box,
                previous_joint_angles: initial_angles,
                previous_tcp: [0.0; 6],
                safety_distance: 0.05,
                previous_safety_distance: 0.0,
                joint_angles_changed: false,
                tcp_changed: false,
                safety_distance_changed: false,
            })
            .insert_resource(Robot {
                kinematics: robot,
                safety: safety_distances,
                joint_meshes: None,
                tool: None,
                base: None,
                environment: Vec::new(),
                material: None,
                tool_material: None,
                base_material: None,
                environment_material: None,
                colliding_material: None,
            })
            .insert_resource(VisualizationCommands {
                receiver: Mutex::new(receiver),
            })
            .add_systems(Startup, setup)
            .add_systems(
                Update,
                (
                    process_visualization_commands,
                    update_robot,
                    camera_controller_system,
                )
                    .chain(),
            )
            .add_systems(EguiPrimaryContextPass, control_panel)
            .run();
    });

    VisualizationHandle {
        sender,
        join_handle: Arc::new(Mutex::new(Some(join_handle))),
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut robot_controls: ResMut<RobotControls>,
    mut robot: ResMut<Robot>,
) {
    fn mesh_for_joint(
        meshes: &mut ResMut<Assets<Mesh>>,
        robot: &ResMut<Robot>,
        k: usize,
    ) -> Handle<Mesh> {
        meshes.add(trimesh_to_bevy_mesh(&robot.kinematics.body.joint_meshes[k]))
    }

    // Precompute the mesh for each of the six robot joints
    robot.joint_meshes = Some([
        mesh_for_joint(&mut meshes, &robot, 0),
        mesh_for_joint(&mut meshes, &robot, 1),
        mesh_for_joint(&mut meshes, &robot, 2),
        mesh_for_joint(&mut meshes, &robot, 3),
        mesh_for_joint(&mut meshes, &robot, 4),
        mesh_for_joint(&mut meshes, &robot, 5),
    ]);

    robot.environment = robot
        .kinematics
        .body
        .collision_environment
        .iter()
        .map(|x| meshes.add(trimesh_to_bevy_mesh(&x.mesh)))
        .collect();

    if let Some(tool) = robot.kinematics.body.tool.as_ref() {
        robot.tool = Some(meshes.add(trimesh_to_bevy_mesh(tool)));
        robot.tool_material = Some(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 1.0, 0.8),
            metallic: 0.7,
            perceptual_roughness: 0.05,
            ..default()
        }));
    }

    if let Some(base) = robot.kinematics.body.base.as_ref() {
        robot.base = Some(meshes.add(trimesh_to_bevy_mesh(&base.mesh)));
        robot.base_material = Some(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 1.0, 0.8),
            metallic: 0.7,
            perceptual_roughness: 0.05,
            ..default()
        }));
    }

    robot.material = Some(materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 1.0, 0.0),
        metallic: 0.7,
        perceptual_roughness: 0.1,
        ..default()
    }));

    robot.environment_material = Some(materials.add(StandardMaterial {
        base_color: Color::srgb(0.5, 0.5, 0.5),
        metallic: 1.0,
        perceptual_roughness: 1.0,
        ..default()
    }));

    robot.colliding_material = Some(materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.1, 0.1),
        metallic: 1.0,
        perceptual_roughness: 0.1,
        ..default()
    }));

    // Visualize the robot joints with the initial joint values
    let angles = utils::joints(&robot_controls.initial_joint_angles);
    visualize_robot_joints(
        &mut commands,
        &mut robot,
        &angles,
        robot_controls.safety_distance,
    );
    let cartesian = robot.kinematics.kinematics.forward(&angles);
    robot_controls.set_tcp_from_pose(&cartesian);

    // Add camera and light
    commands.spawn((
        DirectionalLight {
            illuminance: 30000.0,
            ..default()
        },
        Transform::from_xyz(5.0, 8.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 30000.0,
            ..default()
        },
        Transform::from_xyz(-5.0, 0.0, -5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Camera3d::default(),
        Transform {
            translation: Vec3::new(0.0, 5.0, 20.0), // Adjust camera position as needed
            // Apply a 90-degree rotation around the X-axis
            rotation: Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
            ..default()
        },
        CameraController::default(), // Custom component for controlling the camera
    ));
}

/// This function generates a visual representation of each joint of a robot
/// based on the provided joint angles, rendering each joint shape at its
/// calculated position and orientation using Bevy's `PbrBundle`.
///
/// # Parameters:
/// - `commands`: Mutable reference to Bevy's `Commands`, used to issue entity spawn commands.
/// - `robot`: Reference to the robot's kinematic model and shapes (`KinematicsWithShape`).
/// - `angles`: Joint angles used to compute forward kinematics for rendering.
///
fn visualize_robot_joints(
    commands: &mut Commands,
    robot: &mut ResMut<Robot>,
    angles: &Joints,
    safety_distance: f32,
) {
    /// Spawns a `PbrBundle` entity for a joint, with specified mesh, material, translation, and rotation.
    fn spawn_joint(commands: &mut Commands, mesh: &Handle<Mesh>, state: RobotPartRenderState) {
        commands.spawn((
            Mesh3d(mesh.clone()),
            MeshMaterial3d(state.material),
            state.transform,
            RenderedRobotPart {
                collision_index: state.collision_index,
            },
        ));
    }

    for state in robot_part_render_states(robot, angles, safety_distance) {
        if let Some(mesh) = mesh_for_rendered_part(robot, state.collision_index) {
            spawn_joint(commands, &mesh, state);
        }
    }
}

fn update_rendered_robot_joints(
    robot: &mut ResMut<Robot>,
    angles: &Joints,
    safety_distance: f32,
    query: &mut Query<(
        &RenderedRobotPart,
        &mut Transform,
        &mut MeshMaterial3d<StandardMaterial>,
    )>,
) {
    let states = robot_part_render_states(robot, angles, safety_distance);
    for (part, mut transform, mut material) in query.iter_mut() {
        if let Some(state) = states
            .iter()
            .find(|state| state.collision_index == part.collision_index)
        {
            *transform = state.transform;
            *material = MeshMaterial3d(state.material.clone());
        }
    }
}

/// Returns a colliding material if the joint is in `colliding_segments`, otherwise returns the default material.
fn maybe_colliding_material(
    robot: &ResMut<Robot>,
    material: &Option<Handle<StandardMaterial>>,
    colliding_segments: &HashSet<usize>,
    joint: &usize,
) -> Handle<StandardMaterial> {
    let selected_material = if colliding_segments.contains(joint) {
        &robot.colliding_material
    } else {
        material
    };
    selected_material.as_ref().unwrap().clone()
}

fn transform_from_pose(pose: &Pose32) -> Transform {
    Transform {
        translation: pose.translation,
        rotation: pose.rotation,
        ..default()
    }
}

fn robot_part_render_states(
    robot: &mut ResMut<Robot>,
    angles: &Joints,
    safety_distance: f32,
) -> Vec<RobotPartRenderState> {
    robot.safety.to_environment = safety_distance;
    robot.safety.to_robot_default = safety_distance;
    let collisions = robot.kinematics.near(angles, &robot.safety);
    let colliding_segments: HashSet<_> = collisions.iter().flat_map(|(x, y)| [*x, *y]).collect();
    let positioned_robot = robot.kinematics.positioned_robot(angles);

    let mut states = Vec::with_capacity(
        positioned_robot.joints.len()
            + usize::from(positioned_robot.tool.is_some())
            + usize::from(robot.kinematics.body.base.is_some())
            + positioned_robot.environment.len(),
    );

    for (joint_index, positioned_joint) in positioned_robot.joints.iter().enumerate() {
        states.push(RobotPartRenderState {
            collision_index: joint_index,
            transform: transform_from_pose(&positioned_joint.transform),
            material: maybe_colliding_material(
                robot,
                &robot.material,
                &colliding_segments,
                &joint_index,
            ),
        });
    }

    if let Some(tool_joint) = positioned_robot.tool.as_ref() {
        states.push(RobotPartRenderState {
            collision_index: J_TOOL,
            transform: transform_from_pose(&tool_joint.transform),
            material: maybe_colliding_material(
                robot,
                &robot.tool_material,
                &colliding_segments,
                &J_TOOL,
            ),
        });
    }

    if let Some(base_joint) = robot.kinematics.body.base.as_ref() {
        states.push(RobotPartRenderState {
            collision_index: J_BASE,
            transform: transform_from_pose(&base_joint.base_pose),
            material: maybe_colliding_material(
                robot,
                &robot.base_material,
                &colliding_segments,
                &J_BASE,
            ),
        });
    }

    for (environment_index, environment_joint) in positioned_robot.environment.iter().enumerate() {
        let collision_index = ENV_START_IDX + environment_index;
        states.push(RobotPartRenderState {
            collision_index,
            transform: transform_from_pose(&environment_joint.pose),
            material: maybe_colliding_material(
                robot,
                &robot.environment_material,
                &colliding_segments,
                &collision_index,
            ),
        });
    }

    states
}

fn mesh_for_rendered_part(robot: &ResMut<Robot>, collision_index: usize) -> Option<Handle<Mesh>> {
    match collision_index {
        0..=5 => robot
            .joint_meshes
            .as_ref()
            .map(|joint_meshes| joint_meshes[collision_index].clone()),
        J_TOOL => robot.tool.clone(),
        J_BASE => robot.base.clone(),
        index if index >= ENV_START_IDX => robot.environment.get(index - ENV_START_IDX).cloned(),
        _ => None,
    }
}

// Update the robot when joint angles change
fn process_visualization_commands(
    mut controls: ResMut<RobotControls>,
    command_receiver: Option<Res<VisualizationCommands>>,
    mut app_exit_writer: MessageWriter<AppExit>,
) {
    let Some(command_receiver) = command_receiver else {
        return;
    };
    let Ok(receiver) = command_receiver.receiver.lock() else {
        return;
    };

    while let Ok(command) = receiver.try_recv() {
        match command {
            VisualizationCommand::SetJointAngles(joint_angles) => {
                controls.joint_angles = joint_angles;
                controls.joint_angles_changed = true;
            }
            VisualizationCommand::Close => {
                app_exit_writer.write(AppExit::Success);
            }
        }
    }
}

fn update_robot(
    mut controls: ResMut<RobotControls>,
    mut robot: ResMut<Robot>,
    mut query: Query<(
        &RenderedRobotPart,
        &mut Transform,
        &mut MeshMaterial3d<StandardMaterial>,
    )>,
) {
    let joint_angles_changed = controls.joint_angles_changed;
    let tcp_changed = controls.tcp_changed;
    let safety_distance_changed = controls.safety_distance_changed;

    controls.joint_angles_changed = false;
    controls.tcp_changed = false;
    controls.safety_distance_changed = false;

    if joint_angles_changed {
        let angles = utils::joints(&controls.joint_angles);
        update_rendered_robot_joints(&mut robot, &angles, controls.safety_distance, &mut query);
        controls.previous_joint_angles = controls.joint_angles;

        // A joint slider edit is forward kinematics only. Update the displayed TCP,
        // but do not feed it back into inverse kinematics and rewrite other joints.
        let pose = robot.kinematics.kinematics.forward(&angles);
        controls.set_tcp_from_pose(&pose);
        controls.previous_tcp = controls.tcp;
        controls.previous_safety_distance = controls.safety_distance;
    } else if tcp_changed {
        // Revisualize the robot joints with the updated joint angles
        let angles = utils::joints(&controls.joint_angles);
        let pose = controls.pose();

        // We ask kinematics with shape to do the inverse kinematics.
        // This means, colliding solutions will be discarded.
        let start = Instant::now();
        let ik = robot.kinematics.inverse_continuing(&pose, &angles);
        println!("Time for inverse kinematics: {:?}", start.elapsed());
        if !ik.is_empty() {
            let angles = ik[0];
            update_rendered_robot_joints(&mut robot, &angles, controls.safety_distance, &mut query);

            // Update joint angles to match the current TCP position.
            controls.joint_angles = utils::to_degrees(&angles);
        } else {
            println!(
                "  no solution for pose {:.1} {:.1} {:.1} rotation Z/Y/X {:.1} {:.1} {:.1}",
                controls.tcp[0],
                controls.tcp[1],
                controls.tcp[2],
                controls.tcp[3],
                controls.tcp[4],
                controls.tcp[5]
            );
        }
        controls.previous_tcp = controls.tcp;
        controls.previous_joint_angles = controls.joint_angles;
        controls.previous_safety_distance = controls.safety_distance;
    } else if safety_distance_changed {
        let angles = utils::joints(&controls.joint_angles);
        update_rendered_robot_joints(&mut robot, &angles, controls.safety_distance, &mut query);
        controls.previous_safety_distance = controls.safety_distance;
    }
}

// Control panel for adjusting joint angles and tool center point
fn control_panel(mut egui_contexts: EguiContexts, mut controls: ResMut<RobotControls>) -> Result {
    egui::Window::new("Robot Controls").show(egui_contexts.ctx_mut()?, |ui| {
        ui.label("Joint rotations");
        let mut joint_angles_changed = false;
        for (i, angle) in controls.joint_angles.iter_mut().enumerate() {
            joint_angles_changed |= ui
                .add(egui::Slider::new(angle, -225.0..=225.0).text(format!("Joint {}", i + 1)))
                .changed();
        }
        if joint_angles_changed {
            controls.joint_angles_changed = true;
        }

        let tcp_x_range = controls.tcp_box[0].clone();
        let tcp_y_range = controls.tcp_box[1].clone();
        let tcp_z_range = controls.tcp_box[2].clone();

        ui.add_space(10.0);
        ui.label("Tool center point (TCP)");
        let mut tcp_changed = false;
        tcp_changed |= ui
            .add(egui::Slider::new(&mut controls.tcp[0], tcp_x_range).text("X"))
            .changed();
        tcp_changed |= ui
            .add(egui::Slider::new(&mut controls.tcp[1], tcp_y_range).text("Y"))
            .changed();
        tcp_changed |= ui
            .add(egui::Slider::new(&mut controls.tcp[2], tcp_z_range).text("Z"))
            .changed();

        ui.add_space(10.0);
        ui.label("TCP Euler angles (ZYX)");
        tcp_changed |= ui
            .add(egui::Slider::new(&mut controls.tcp[3], -90.0..=90.0).text("Z rotation"))
            .changed();
        tcp_changed |= ui
            .add(egui::Slider::new(&mut controls.tcp[4], -90.0..=90.0).text("Y rotation"))
            .changed();
        tcp_changed |= ui
            .add(egui::Slider::new(&mut controls.tcp[5], -90.0..=90.0).text("X rotation"))
            .changed();
        if tcp_changed {
            controls.tcp_changed = true;
        }

        ui.add_space(10.0);
        ui.label("Safety distance");
        if ui
            .add(egui::Slider::new(&mut controls.safety_distance, 0.0..=0.5).text("Max"))
            .changed()
        {
            controls.safety_distance_changed = true;
        }
    });
    Ok(())
}
