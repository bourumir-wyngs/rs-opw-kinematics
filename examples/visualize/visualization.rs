use bevy::prelude::*;        // Add the **flipped** normal to each vertex's normal (negating the normal)
use parry3d::shape::TriMesh;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use bevy_egui::egui::emath::Numeric;

use crate::camera_controller::{camera_controller_system, CameraController};
use crate::robot_body_builder::create_sample_robot;
use rs_opw_kinematics::kinematics_with_shape::{KinematicsWithShape, PositionedJoint};


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

// Data to store the current joint angles
#[derive(Resource)]
struct RobotControls {
    joint_angles: [f32; 6],
}

// Resource to store the current robot instance
#[derive(Resource)]
struct Robot {
    kinematics: KinematicsWithShape,
    joint_meshes: Option<[Handle<Mesh>; 6]>, // Precomputed and converted meshes
    material: Option<Handle<StandardMaterial>>,
    tool_material: Option<Handle<StandardMaterial>>,
    previous_joint_angles: [f32; 6], // Store previous joint angles here
    tool: Option<Handle<Mesh>>,
}

pub(crate) fn main_method() {
    App::new()
        .add_plugins((DefaultPlugins, EguiPlugin)) // Use add_plugin for Egui
        .insert_resource(RobotControls {
            joint_angles: [0.0; 6],  // Initialize all joints to 0 degrees
        })
        .insert_resource(Robot {
            kinematics: create_sample_robot(),
            joint_meshes: None,
            material: None,
            tool_material: None,
            tool: None,
            previous_joint_angles: [0.0; 6],  // Track previous joint angles            
        })
        .add_systems(Startup, setup)               // Register setup system in Startup phase
        .add_systems(Update, (update_robot_joints, camera_controller_system, control_panel)) // Add systems without .system()
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    robot_controls: Res<RobotControls>,
    mut robot: ResMut<Robot>,
) {
    fn prepare(meshes: &mut ResMut<Assets<Mesh>>, robot: &ResMut<Robot>, k: usize) -> Handle<Mesh> {
        meshes.add(trimesh_to_bevy_mesh(&robot.kinematics.body.joint_bodies[k].transformed_shape))
    }

    // Precompute the mesh for each of the six robot joints
    robot.joint_meshes = Some([
        prepare(&mut meshes, &robot, 0), prepare(&mut meshes, &robot, 1),
        prepare(&mut meshes, &robot, 2), prepare(&mut meshes, &robot, 3),
        prepare(&mut meshes, &robot, 4), prepare(&mut meshes, &robot, 5)
    ]);

    if let Some(tool) = robot.kinematics.body.tool.as_ref() {
        robot.tool = Some(meshes.add(trimesh_to_bevy_mesh(&tool.transformed_shape)));
        robot.tool_material = Some(
            materials.add(StandardMaterial {
                base_color: Color::rgb(1.0, 1.0, 1.0),
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

    // Visualize the robot joints with the initial joint values
    visualize_robot_joints(&mut commands, &robot, &robot_controls.joint_angles);

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
/// based on the provided joint angles. The joint shapes are transformed
/// according to the kinematic configuration and rendered using Bevy's `PbrBundle`
/// for materials and lighting.
///
/// # Parameters:
/// - `commands`: Mutable reference to Bevy's `Commands`, used to issue entity spawn commands.
/// - `meshes`: Mutable resource containing Bevy's asset storage for meshes.
/// - `materials`: Mutable resource containing Bevy's asset storage for standard materials.
/// - `robot`: Reference to the robot's kinematic model and shapes (`KinematicsWithShape`).
/// - `joint_angles`: Array of 6 f32 values representing the joint angles of the robot in degrees.
///
/// Each joint's shape is transformed and rendered at the correct position and orientation
/// based on the robot's forward kinematics.
fn visualize_robot_joints(
    commands: &mut Commands,
    robot: &ResMut<Robot>,
    joint_angles: &[f32; 6],
) {
    // Obtain Bevy-compliate coordinates
    fn as_bevy(positioned_joint: &&PositionedJoint) -> (Vec3, Quat) {
        let transform = &positioned_joint.transform;
        let translation = &transform.translation.vector;
        let rotation = &transform.rotation;

        // Migrate to Bevy system of coordinates
        let translation_vec3 = Vec3::new(translation.x, translation.y, translation.z);
        let rotation_quat = Quat::from_xyzw(rotation.i, rotation.j, rotation.k, rotation.w);
        let final_rotation = rotation_quat;
        (translation_vec3, final_rotation)
    }

    // Visualize each joint of the robot
    let angles: [f64; 6] = [
        joint_angles[0].to_f64().to_radians(),
        joint_angles[1].to_f64().to_radians(),
        joint_angles[2].to_f64().to_radians(),
        joint_angles[3].to_f64().to_radians(),
        joint_angles[4].to_f64().to_radians(),
        joint_angles[5].to_f64().to_radians()
    ];

    let positioned_robot = robot.kinematics.positioned_robot(&angles);
    for j in 0..6 {
        let positioned_joint = &positioned_robot.joints[j];
        let (translation_vec3, final_rotation) = as_bevy(&positioned_joint);

        commands.spawn(PbrBundle {
            mesh: robot.joint_meshes.as_ref().unwrap()[j].clone(),
            material: robot.material.as_ref().unwrap().clone(),
            transform: Transform {
                translation: translation_vec3,
                rotation: final_rotation,
                ..default()
            },
            ..default()
        });
    }


    if let (Some(tool), Some(positioned_joint)) =
        (&robot.tool, positioned_robot.tool.as_ref()) {
        let (translation_vec3, final_rotation) = as_bevy(&positioned_joint);

        commands.spawn(PbrBundle {
            mesh: tool.clone(),
            material: robot.tool_material.as_ref().unwrap().clone(),
            transform: Transform {
                translation: translation_vec3,
                rotation: final_rotation,
                ..default()
            },
            ..default()
        });
    }
}

// Update the robot when joint angles change
fn update_robot_joints(
    mut commands: Commands,
    robot_controls: Res<RobotControls>,
    mut robot: ResMut<Robot>,
    query: Query<Entity, With<Handle<Mesh>>>,  // Query entities with Mesh components
) {
    if robot_controls.joint_angles != robot.previous_joint_angles {
        // Despawn the existing visualized robot joints
        for entity in query.iter() {
            commands.entity(entity).despawn();
        }

        // Revisualize the robot joints with the updated joint angles
        visualize_robot_joints(&mut commands, &mut robot, &robot_controls.joint_angles);

        robot.previous_joint_angles = robot_controls.joint_angles.clone();
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
