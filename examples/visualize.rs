use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;
use bevy::pbr::wireframe::{Wireframe, WireframePlugin};
use bevy::render::mesh::PrimitiveTopology;
use parry3d::shape::TriMesh;
use nalgebra::Point3;

fn trimesh_to_bevy_mesh(trimesh: &TriMesh) -> Mesh {
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, trimesh.vertices().iter().map(|v| [v.x, v.y, v.z]).collect::<Vec<_>>());
    mesh.set_indices(Some(bevy::render::mesh::Indices::U32(trimesh.indices().iter().flat_map(|i| i.to_vec()).collect())));
    mesh
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Vertices of the tetrahedron (pyramid)
    let vertices = vec![
        Point3::new(1.0, 1.0, 1.0),     // V0
        Point3::new(-1.0, -1.0, 1.0),   // V1
        Point3::new(-1.0, 1.0, -1.0),   // V2
        Point3::new(1.0, -1.0, -1.0),   // V3
    ];

    // Triangles (indices) connecting the vertices
    let indices = vec![
        [0, 1, 2],  // Face 1: (V0, V1, V2)
        [0, 3, 1],  // Face 2: (V0, V3, V1)
        [0, 2, 3],  // Face 3: (V0, V2, V3)
        [1, 3, 2],  // Face 4: (V1, V3, V2)
    ];

    // Create transformed_shape using TriMesh::new
    let transformed_shape = TriMesh::new(vertices.clone(), indices.clone());

    // Add transformed_shape with a brighter yellow base color and white wireframe
    commands.spawn(PbrBundle {
        mesh: meshes.add(trimesh_to_bevy_mesh(&transformed_shape)),
        material: materials.add(StandardMaterial {
            base_color: Color::rgb(1.0, 1.0, 0.0),  // Bright yellow base color
            metallic: 0.2,  // A bit more metallic to reflect light better
            perceptual_roughness: 0.3,  // Less rough for better light reflection
            ..default()
        }),
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..default()
    }).insert(Wireframe);  // Enable wireframe for white edges

    // Add a directional light for better shading
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 30000.0,  // Increased brightness for better illumination
            ..default()
        },
        transform: Transform::from_xyz(5.0, 8.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // Add ambient light to brighten the overall scene
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 0.5,  // Brighten the scene with ambient light
    });

    // Add a camera with a custom component to track its movement
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 5.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y), // Camera position and orientation
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
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            speed: 10.0,
            sensitivity: 0.4,
            pitch: 0.0,
            yaw: 0.0,
            distance: 10.0, // Initial camera distance from the object
        }
    }
}

// System to handle mouse movement, zoom, and rotate the camera
fn camera_controller_system(
    mouse_button_input: Res<Input<MouseButton>>, // Track mouse button input
    mut motion_events: EventReader<MouseMotion>,
    mut wheel_events: EventReader<MouseWheel>,
    mut query: Query<(&mut Transform, &mut CameraController)>,
) {
    for (mut transform, mut controller) in query.iter_mut() {
        // Handle mouse motion for rotation only when left mouse button is pressed
        if mouse_button_input.pressed(MouseButton::Left) {
            for event in motion_events.iter() {
                controller.yaw -= event.delta.x * controller.sensitivity;
                controller.pitch -= event.delta.y * controller.sensitivity;
            }

            // Clamp the pitch to avoid the camera flipping upside down
            controller.pitch = controller.pitch.clamp(-89.0, 89.0);

            // Update camera's direction based on the new pitch and yaw values
            let yaw_radians = controller.yaw.to_radians();
            let pitch_radians = controller.pitch.to_radians();

            let direction = Vec3::new(
                yaw_radians.cos() * pitch_radians.cos(),
                pitch_radians.sin(),
                yaw_radians.sin() * pitch_radians.cos(),
            );

            // Update camera position based on direction and distance
            transform.translation = direction * controller.distance;
            transform.look_at(Vec3::ZERO, Vec3::Y); // Always look at the origin
        }

        // Handle mouse wheel zoom (independent of mouse button state)
        for event in wheel_events.iter() {
            controller.distance -= event.y * controller.speed * 0.1;
            controller.distance = controller.distance.clamp(2.0, 50.0); // Prevent zooming too far in or out

            // Update camera position based on direction and distance (for zoom)
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
        .add_systems(Startup, setup)
        .add_systems(Update, camera_controller_system) // Add the camera control system
        .run();
}