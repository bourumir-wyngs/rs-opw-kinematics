//! Provides visualization window with sliders for angles and pose.
//! 
//! <img src="https://github.com/user-attachments/assets/3a0e6293-519e-455f-bf8b-0ff1090103b1" alt="screenshot" width="300"/>
//! 
//! The [`crate::kinematics_with_shape::KinematicsWithShape`] structure fully integrates kinematics 
//! with 3D meshes representing the robot, making it straightforward to visualize.
//! To display the robot, simply pass this structure to the built-in function 
//! [`visualize_robot`].  
//!
//! ```rust
//! fn main() {
//!     use std::ops::RangeInclusive;
//!     use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
//!     use rs_opw_kinematics::visualization;
//! 
//!     // See `complete_visible_robot example` how to build this structure
//!     let robot: KinematicsWithShape; 
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


use std::collections::HashSet;
use std::ops::{RangeInclusive};
use std::time::Instant;
use bevy::prelude::*;
use parry3d::shape::TriMesh;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use crate::kinematic_traits::{Joints, Kinematics, Pose, ENV_START_IDX, J_BASE, J_TOOL};
use crate::kinematics_with_shape::{KinematicsWithShape};
use crate::utils;
use crate::camera_controller::{camera_controller_system, CameraController};

// Convert a parry3d `TriMesh` into a bevy `Mesh` for rendering
fn trimesh_to_bevy_mesh(trimesh: &TriMesh) -> Mesh {
    let mut mesh = Mesh::new(bevy::render::mesh::PrimitiveTopology::TriangleList);

    // Step 1: Extract vertices and indices from the TriMesh
    let vertices: Vec<_> = trimesh.vertices().iter().map(|v| [v.x, v.y, v.z]).collect();
    let indices: Vec<_> = trimesh.indices().iter().flat_map(|i| i.to_vec()).collect::<Vec<_>>();

    // Step 2: Initialize normal vectors for each vertex
    let mut normals: Vec<[f32; 3]> = vec![[0.0, 0.0, 0.0]; vertices.len()];

    // Step 3: Calculate normals for each face (triangle)
    for triangle in trimesh.indices() {
        let i0 = triangle[0] as usize;
        let i1 = triangle[1] as usize;
        let i2 = triangle[2] as usize;

        // Get the three vertices of the triangle
        let v0 = nalgebra::Vector3::new(vertices[i0][0], vertices[i0][1], vertices[i0][2]);
        let v1 = nalgebra::Vector3::new(vertices[i1][0], vertices[i1][1], vertices[i1][2]);
        let v2 = nalgebra::Vector3::new(vertices[i2][0], vertices[i2][1], vertices[i2][2]);

        // Calculate the two edge vectors
        let edge1 = v1 - v0;
        let edge2 = v2 - v0;

        // Calculate the normal using the cross product of the two edges
        let normal = edge1.cross(&edge2).normalize();

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
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);  // Add the flipped normals

    // Step 6: Set the indices
    mesh.set_indices(Some(bevy::render::mesh::Indices::U32(indices)));

    mesh
}

/// Data to store the current joint angles and TCP as they are shown in control panel
#[derive(Resource)]
struct RobotControls {
    joint_angles: [f32; 6],
    tcp: [f64; 6],
    tcp_box: [RangeInclusive<f64>; 3], // Boundaries for sliders in GUI to move the tool center point arround 
    initial_joint_angles: [f32; 6], // Initial angles at the start of visualization    
    previous_joint_angles: [f32; 6], // Store previous joint angles here
    previous_tcp: [f64; 6],
}

impl RobotControls {
    fn set_tcp_from_pose(&mut self, pose: &Isometry3<f64>) {
        let (roll, pitch, yaw) = pose.rotation.to_rotation_matrix().euler_angles();
        self.tcp = [
            pose.translation.x, pose.translation.y, pose.translation.z,
            roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees(),
        ];
    }

    fn pose(&self) -> Pose {
        fn quat_from_euler(this: &RobotControls) -> UnitQuaternion<f64> {
            let roll = this.tcp[3].to_radians();
            let pitch = this.tcp[4].to_radians();
            let yaw = this.tcp[5].to_radians();

            // Combine rotations in roll-pitch-yaw order
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), roll) *
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), pitch) *
                UnitQuaternion::from_axis_angle(&Vector3::x_axis(), yaw)
        }

        Isometry3::from_parts(
            Translation3::new(self.tcp[0], self.tcp[1], self.tcp[2]),
            quat_from_euler(&self))
    }
}

// Resource to store the current robot instance
#[derive(Resource)]
struct Robot {
    kinematics: KinematicsWithShape,
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
/// The sliders for specifying the tool center point location with take the boundaries
/// from the tcp_box (given in meters). Bevy will be used for visualization.
pub fn visualize_robot(robot: KinematicsWithShape,
                       intial_angles: [f32; 6], tcp_box: [RangeInclusive<f64>; 3]) {
    App::new()
        .add_plugins((DefaultPlugins, EguiPlugin)) // Use add_plugin for Egui
        .insert_resource(RobotControls {
            initial_joint_angles: intial_angles.clone(),
            joint_angles: intial_angles.clone(),
            tcp: [0.0; 6],
            tcp_box: tcp_box,
            previous_joint_angles: intial_angles.clone(),
            previous_tcp: [0.0; 6],
        })
        .insert_resource(Robot {
            kinematics: robot,
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
        .add_systems(Update, (update_robot, camera_controller_system, control_panel)) // Add systems without .system()
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut robot_controls: ResMut<RobotControls>,
    mut robot: ResMut<Robot>,
) {
    fn mesh_for_joint(meshes: &mut ResMut<Assets<Mesh>>, robot: &ResMut<Robot>, k: usize) -> Handle<Mesh> {
        meshes.add(trimesh_to_bevy_mesh(&robot.kinematics.body.joint_meshes[k]))
    }

    // Precompute the mesh for each of the six robot joints
    robot.joint_meshes = Some([
        mesh_for_joint(&mut meshes, &robot, 0),
        mesh_for_joint(&mut meshes, &robot, 1),
        mesh_for_joint(&mut meshes, &robot, 2),
        mesh_for_joint(&mut meshes, &robot, 3),
        mesh_for_joint(&mut meshes, &robot, 4),
        mesh_for_joint(&mut meshes, &robot, 5)
    ]);

    robot.environment = robot.kinematics.body.collision_environment.iter()
        .map(|x| { meshes.add(trimesh_to_bevy_mesh(&x.mesh)) })
        .collect();


    if let Some(tool) = robot.kinematics.body.tool.as_ref() {
        robot.tool = Some(meshes.add(trimesh_to_bevy_mesh(&tool)));
        robot.tool_material = Some(
            materials.add(StandardMaterial {
                base_color: Color::rgb(0.8, 1.0, 0.8),
                metallic: 0.7,
                perceptual_roughness: 0.05,
                ..default()
            })
        );
    }

    if let Some(base) = robot.kinematics.body.base.as_ref() {
        robot.base = Some(meshes.add(trimesh_to_bevy_mesh(&base.mesh)));
        robot.base_material = Some(
            materials.add(StandardMaterial {
                base_color: Color::rgb(0.8, 1.0, 0.8),
                metallic: 0.7,
                perceptual_roughness: 0.05,
                ..default()
            })
        );
    }

    robot.material = Some(
        materials.add(StandardMaterial {
            base_color: Color::rgb(1.0, 1.0, 0.0),
            metallic: 0.7,
            perceptual_roughness: 0.1,
            ..default()
        })
    );

    robot.environment_material = Some(
        materials.add(StandardMaterial {
            base_color: Color::rgb(0.5, 0.5, 0.5),
            metallic: 1.0,
            perceptual_roughness: 1.0,
            ..default()
        })
    );

    robot.colliding_material = Some(
        materials.add(StandardMaterial {
            base_color: Color::rgb(1.0, 0.1, 0.1),
            metallic: 1.0,
            perceptual_roughness: 0.1,
            ..default()
        })
    );

    // Visualize the robot joints with the initial joint values
    let angles = utils::joints(&robot_controls.initial_joint_angles);
    visualize_robot_joints(&mut commands, &robot, &angles);
    let cartesian = robot.kinematics.kinematics.forward(&angles);
    robot_controls.set_tcp_from_pose(&cartesian);

    // Add camera and light
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 30000.0,
            ..default()
        },
        transform: Transform::from_xyz(5.0, 8.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 30000.0,
            ..default()
        },
        transform: Transform::from_xyz(-5.0, 0.0, -5.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    commands.spawn((
        Camera3dBundle {
            transform: Transform {
                translation: Vec3::new(0.0, 5.0, 20.0), // Adjust camera position as needed
                // Apply a 90-degree rotation around the X-axis
                rotation: Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
                ..default()
            },
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
    robot: &ResMut<Robot>,
    angles: &Joints,
) {
    // Helper function to transform coordinates to Bevy's coordinate system
    fn as_bevy(transform: &Isometry3<f32>) -> (Vec3, Quat) {
        let translation = transform.translation.vector;
        let rotation = transform.rotation;
        (
            Vec3::new(translation.x, translation.y, translation.z),
            Quat::from_xyzw(rotation.i, rotation.j, rotation.k, rotation.w),
        )
    }

    /// Spawns a `PbrBundle` entity for a joint, with specified mesh, material, translation, and rotation.
    fn spawn_joint(
        commands: &mut Commands,
        mesh: &Handle<Mesh>,
        material: Handle<StandardMaterial>,
        pose: &Isometry3<f32>,
    ) {
        let (translation, rotation) = as_bevy(&pose);
        commands.spawn(PbrBundle {
            mesh: mesh.clone(),
            material,
            transform: Transform {
                translation,
                rotation,
                ..default()
            },
            ..default()
        });
    }

    // Detect collisions between joints
    let start = Instant::now();
    let collisions = robot.kinematics.collision_details(&angles);
    println!("Time for collision check: {:?}", start.elapsed());

    let colliding_segments: HashSet<_> =
        collisions.iter().flat_map(|(x, y)| [*x, *y]).collect();

    // Visualize each joint in the robot
    let positioned_robot = robot.kinematics.positioned_robot(angles);
    for (j, positioned_joint) in positioned_robot.joints.iter().enumerate() {
        spawn_joint(commands, &robot.joint_meshes.as_ref().unwrap()[j],
                    maybe_colliding_material(&robot, &robot.material, &colliding_segments, &j),
                    &positioned_joint.transform);
    }

    // Visualize the tool if present
    if let (Some(tool), Some(tool_joint)) = (&robot.tool, positioned_robot.tool.as_ref()) {
        spawn_joint(commands, tool,
                    maybe_colliding_material(&robot, &robot.tool_material, &colliding_segments, &J_TOOL),
                    &tool_joint.transform);
    }

    // Visualize the base if present
    if let (Some(base), Some(base_joint)) = (&robot.base, robot.kinematics.body.base.as_ref()) {
        spawn_joint(commands, base,
                    maybe_colliding_material(&robot, &robot.base_material, &colliding_segments, &J_BASE),
                    &base_joint.base_pose);
    }

    // Add environment objects
    for (i, environment_joint) in positioned_robot.environment.iter().enumerate() {
        spawn_joint(commands, &robot.environment[i],
                    maybe_colliding_material(&robot, &robot.environment_material, &colliding_segments, &(ENV_START_IDX + i)),
                    &environment_joint.pose);
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

// Update the robot when joint angles change
fn update_robot(
    mut commands: Commands,
    mut controls: ResMut<RobotControls>,
    mut robot: ResMut<Robot>,
    query: Query<Entity, With<Handle<Mesh>>>,  // Query entities with Mesh components
) {
    if controls.joint_angles != controls.previous_joint_angles {
        // Despawn the existing visualized robot joints
        for entity in query.iter() {
            commands.entity(entity).despawn();
        }

        // Revisualize the robot joints with the updated joint angles
        // Visualize each joint of the robot
        let angles = utils::joints(&controls.joint_angles);
        visualize_robot_joints(&mut commands, &mut robot, &angles);
        controls.previous_joint_angles = controls.joint_angles.clone();

        // Update the TCP position (this is the end of the tool, not the base)
        let pose = robot.kinematics.kinematics.forward(&angles);
        controls.set_tcp_from_pose(&pose);
        controls.previous_tcp = controls.tcp;
    } else if controls.tcp != controls.previous_tcp {
        // Revisualize the robot joints with the updated joint angles
        let angles = utils::joints(&controls.joint_angles);
        let pose = controls.pose();

        // We ask kinematics with shape to do the inverse kinematics.
        // This means, colliding solutions will be discarded.
        let start = Instant::now();
        let ik = robot.kinematics.inverse_continuing(&pose, &angles);
        println!("Time for inverse kinematics: {:?}", start.elapsed());
        if ik.len() > 0 {
            for entity in query.iter() {
                commands.entity(entity).despawn();
            }
            let angles = ik[0];
            visualize_robot_joints(&mut commands, &mut robot, &angles);

            // Update joint angles to match the current position
            controls.joint_angles = utils::to_degrees(&angles);
        } else {
            println!(
                "  no solution for pose {:.1} {:.1} {:.1} rotation {:.1} {:.1} {:.1}",
                controls.tcp[0], controls.tcp[1], controls.tcp[2],
                controls.tcp[3], controls.tcp[4], controls.tcp[5]
            );
        }
        controls.previous_tcp = controls.tcp.clone();
        controls.previous_joint_angles = controls.joint_angles;
    }
}

// Control panel for adjusting joint angles and tool center point
fn control_panel(
    mut egui_contexts: EguiContexts,
    mut controls: ResMut<RobotControls>,
) {
    bevy_egui::egui::Window::new("Robot Controls").show(egui_contexts.ctx_mut(), |ui| {
        ui.label("Joint rotations");
        for (i, angle) in controls.joint_angles.iter_mut().enumerate() {
            ui.add(egui::Slider::new(angle, -225.0..=225.0).text(format!("Joint {}", i + 1)));
        }

        let tcp_x_range = controls.tcp_box[0].clone();
        let tcp_y_range = controls.tcp_box[1].clone();
        let tcp_z_range = controls.tcp_box[2].clone();

        ui.add_space(10.0);
        ui.label("Tool center point (TCP)");
        ui.add(egui::Slider::new(&mut controls.tcp[0], tcp_x_range).text("X"));
        ui.add(egui::Slider::new(&mut controls.tcp[1], tcp_y_range).text("Y"));
        ui.add(egui::Slider::new(&mut controls.tcp[2], tcp_z_range).text("Z"));

        ui.add_space(10.0);
        ui.label("TCP Euler angles");
        ui.add(egui::Slider::new(&mut controls.tcp[3], -90.0..=90.0).text("Roll"));
        ui.add(egui::Slider::new(&mut controls.tcp[4], -90.0..=90.0).text("Pitch"));
        ui.add(egui::Slider::new(&mut controls.tcp[5], -90.0..=90.0).text("Yaw"));
    });
}
