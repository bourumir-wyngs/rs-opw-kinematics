use bevy::prelude::*;
use bevy::pbr::wireframe::{Wireframe, WireframePlugin};
use bevy::input::mouse::{MouseWheel};
use bevy::window::PrimaryWindow;
use parry3d::shape::TriMesh;

use rs_opw_kinematics::kinematics_with_shape::PositionedRobot;

fn trimesh_to_bevy_mesh(trimesh: &TriMesh) -> Mesh {
    let mut mesh = Mesh::new(bevy::render::mesh::PrimitiveTopology::TriangleList);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, trimesh.vertices().iter().map(|v| [v.x, v.y, v.z]).collect::<Vec<_>>());
    mesh.set_indices(Some(bevy::render::mesh::Indices::U32(trimesh.indices().iter().flat_map(|i| i.to_vec()).collect())));
    mesh
}

fn create_pyramid() -> TriMesh {
    let vertices = vec![
        nalgebra::Point3::new(1.0, 1.0, 1.0),     // V0
        nalgebra::Point3::new(-1.0, -1.0, 1.0),   // V1
        nalgebra::Point3::new(-1.0, 1.0, -1.0),   // V2
        nalgebra::Point3::new(1.0, -1.0, -1.0),   // V3
    ];

    let indices = vec![
        [0, 1, 2],  // Face 1: (V0, V1, V2)
        [0, 3, 1],  // Face 2: (V0, V3, V1)
        [0, 2, 3],  // Face 3: (V0, V2, V3)
        [1, 3, 2],  // Face 4: (V1, V3, V2)
    ];

    TriMesh::new(vertices, indices)
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Example robot with multiple joints (pyramids)
    let pyramid_shape = create_pyramid();
    let joint_count = 5;  // Number of joints
    let spacing = 4.0;    // Spacing between pyramids

    for i in 0..joint_count {
        // Place each pyramid in a single row
        let position = Vec3::new(i as f32 * spacing, 0.0, 0.0);

        commands.spawn(PbrBundle {
            mesh: meshes.add(trimesh_to_bevy_mesh(&pyramid_shape)),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(1.0, 1.0, 0.0),  // Yellow base color
                metallic: 0.2,
                perceptual_roughness: 0.3,
                ..default()
            }),
            transform: Transform::from_translation(position),
            ..default()
        }).insert(Wireframe);  // Wireframe to show the edges
    }

    // Add a directional light
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 30000.0,  // Bright enough to illuminate all pyramids
            ..default()
        },
        transform: Transform::from_xyz(5.0, 8.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // Add a camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 5.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        CameraController::default(), // Custom component for controlling the camera
    ));
}

// Component for camera control
#[derive(Component)]
struct CameraController {
    pub speed: f32,
    pub sensitivity: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub distance: f32,
    pub last_mouse_position: Option<Vec2>,  // Track the last known mouse position
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            speed: 10.0,
            sensitivity: 0.1,
            pitch: 0.0,
            yaw: 0.0,
            distance: 20.0,  // Initial camera distance from the object
            last_mouse_position: None,  // No mouse position yet
        }
    }
}

// System to handle mouse movement, zoom, and rotate the camera
fn camera_controller_system(
    mouse_button_input: Res<Input<MouseButton>>,  // Track mouse button input
    mut wheel_events: EventReader<MouseWheel>,    // Track mouse wheel events for zoom
    mut query: Query<(&mut Transform, &mut CameraController)>,
    windows: Query<&Window, With<PrimaryWindow>>,  // Correct query for primary window
) {
    let window = windows.single();

    for (mut transform, mut controller) in query.iter_mut() {
        // Check if the left mouse button is pressed
        if mouse_button_input.pressed(MouseButton::Left) {
            // Get the current cursor position
            if let Some(cursor_position) = window.cursor_position() {
                // If the last mouse position is not set, initialize it
                if controller.last_mouse_position.is_none() {
                    controller.last_mouse_position = Some(cursor_position);
                    return;
                }

                // Calculate the delta between the last and current position
                if let Some(last_position) = controller.last_mouse_position {
                    let delta = cursor_position - last_position;
                    controller.yaw -= delta.x * controller.sensitivity;
                    controller.pitch -= delta.y * controller.sensitivity;
                }

                // Update the last known mouse position
                controller.last_mouse_position = Some(cursor_position);
            }

            // Clamp the pitch to avoid flipping the camera upside down
            controller.pitch = controller.pitch.clamp(-89.0, 89.0);

            // Update camera's direction based on the new pitch and yaw values
            let yaw_radians = controller.yaw.to_radians();
            let pitch_radians = controller.pitch.to_radians();

            let direction = Vec3::new(
                yaw_radians.cos() * pitch_radians.cos(),
                pitch_radians.sin(),
                yaw_radians.sin() * pitch_radians.cos(),
            );

            transform.translation = direction * controller.distance;
            transform.look_at(Vec3::ZERO, Vec3::Y); // Always look at the origin
        } else {
            // Reset last mouse position when the left mouse button is released
            controller.last_mouse_position = None;
        }

        // Handle mouse wheel zoom
        for event in wheel_events.read() {
            controller.distance -= event.y * controller.speed * 0.1;
            controller.distance = controller.distance.clamp(2.0, 50.0);  // Prevent zooming too far in or out

            // Update camera position for zoom
            let yaw_radians = controller.yaw.to_radians();
            let pitch_radians = controller.pitch.to_radians();

            let direction = Vec3::new(
                yaw_radians.cos() * pitch_radians.cos(),
                pitch_radians.sin(),
                yaw_radians.sin() * pitch_radians.cos(),
            );
            transform.translation = direction * controller.distance;
        }
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WireframePlugin)  // Ensure the wireframe plugin is added
        .add_systems(Startup, setup)  // Register the setup system
        .add_systems(Update, camera_controller_system)  // Reintroduce the camera control system
        .run();
}