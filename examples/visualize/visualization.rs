use bevy::prelude::*;        // Add the **flipped** normal to each vertex's normal (negating the normal)
use bevy::pbr::wireframe::Wireframe;
use nalgebra::Isometry3;
use parry3d::shape::TriMesh;
use crate::camera_controller::{camera_controller_system, CameraController};
use crate::robot_body_builder::create_sample_robot;

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
            normals[i][0] = normal.x;
            normals[i][1] = normal.y;
            normals[i][2] = normal.z;
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

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;

// Data to store the current joint angles
#[derive(Resource)]
struct RobotControls {
    joint_angles: [f32; 6],
}

pub(crate) fn mein() {
    App::new()
        .add_plugins((DefaultPlugins, EguiPlugin)) // Use add_plugin for Egui
        .insert_resource(RobotControls {
            joint_angles: [0.0; 6],  // Initialize all joints to 0 degrees
        })
        .add_systems(Startup, setup)               // Register setup system in Startup phase
        .add_systems(Update, (update_robot_joints, camera_controller_system, control_panel)) // Add systems without .system()
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    robot_controls: Res<RobotControls>,  // Get joint angles for initial setup
) {
    // Create the sample robot using initial joint values
    let robot = _create_sample_robot(robot_controls.joint_angles);

    // Visualize each joint of the robot
    for (i, joint_body) in robot.body.joint_bodies.iter().enumerate() {
        let transform = robot.body.joint_origins[i];
        let translation = transform.translation.vector;
        let rotation = transform.rotation;

        let translation_vec3 = Vec3::new(translation.x, translation.z, translation.y);
        let swap_quat = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
        let rotation_quat = Quat::from_xyzw(rotation.i, rotation.j, rotation.k, rotation.w);
        let final_rotation = swap_quat * rotation_quat;

        commands.spawn(PbrBundle {
            mesh: meshes.add(trimesh_to_bevy_mesh(&joint_body.transformed_shape)),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(1.0, 1.0, 0.0),  // Yellow color
                metallic: 0.2,
                perceptual_roughness: 0.3,
                ..default()
            }),
            transform: Transform {
                translation: translation_vec3,
                rotation: final_rotation,
                ..default()
            },
            ..default()
        });
    }

    // Add camera and light
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 30000.0,
            ..default()
        },
        transform: Transform::from_xyz(5.0, 8.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 5.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        CameraController::default(), // Custom component for controlling the camera
    ));
}

// Update the robot when joint angles change
fn update_robot_joints(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    robot_controls: Res<RobotControls>,
    query: Query<Entity, With<Handle<Mesh>>>,  // Query entities with Mesh components
) {
    // Despawn all current robot joints
    for entity in query.iter() {
        commands.entity(entity).despawn();
    }

    // Recreate the robot with the updated joint angles
    let robot = _create_sample_robot(robot_controls.joint_angles);

    // (Same joint visualization code as in setup, reusing robot)
    for (i, joint_body) in robot.body.joint_bodies.iter().enumerate() {
        let transform = robot.body.joint_origins[i];
        let translation = transform.translation.vector;
        let rotation = transform.rotation;

        let translation_vec3 = Vec3::new(translation.x, translation.z, translation.y);
        let swap_quat = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
        let rotation_quat = Quat::from_xyzw(rotation.i, rotation.j, rotation.k, rotation.w);
        let final_rotation = swap_quat * rotation_quat;

        commands.spawn(PbrBundle {
            mesh: meshes.add(trimesh_to_bevy_mesh(&joint_body.transformed_shape)),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(1.0, 1.0, 0.0),
                metallic: 0.2,
                perceptual_roughness: 0.3,
                ..default()
            }),
            transform: Transform {
                translation: translation_vec3,
                rotation: final_rotation,
                ..default()
            },
            ..default()
        });
    }
}


// Control panel for adjusting joint angles
fn control_panel(
    mut egui_contexts: EguiContexts,  // Use EguiContexts instead of EguiContext
    mut robot_controls: ResMut<RobotControls>,
) {
    bevy_egui::egui::Window::new("Robot Controls").show(egui_contexts.ctx_mut(), |ui| {
        for (i, angle) in robot_controls.joint_angles.iter_mut().enumerate() {
            ui.add(egui::Slider::new(angle, -360.0..=360.0).text(format!("Joint {}", i + 1)));
        }
    });
}

// Function to create a robot with joint angles (you need to implement this)
fn _create_sample_robot(joint_angles: [f32; 6]) -> KinematicsWithShape {
    create_sample_robot()
}
