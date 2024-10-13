use bevy::prelude::*;
use bevy::input::mouse::MouseWheel;
use bevy::window::PrimaryWindow;
use bevy::prelude::Component;

// Component for camera control
#[derive(Component)]
pub struct CameraController {
    pub speed: f32,
    pub sensitivity: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub distance: f32,
    pub last_mouse_position: Option<Vec2>,
    pub pan_target: Vec3,                 // Point the camera orbits around (panning target)
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            speed: 10.0,
            sensitivity: 0.1,
            pitch: 0.0,
            yaw: 0.0,
            distance: 40.0,  // Start with a distance of 40 units
            last_mouse_position: None,
            pan_target: Vec3::ZERO,  // Start by orbiting around the origin
        }
    }
}

pub fn camera_controller_system(
    mouse_button_input: Res<Input<MouseButton>>,
    mut wheel_events: EventReader<MouseWheel>,
    mut query: Query<(&mut Transform, &mut CameraController)>,
    windows: Query<&Window, With<PrimaryWindow>>,
) {
    let window = windows.single();

    for (mut transform, mut controller) in query.iter_mut() {

        if let Some(cursor_position) = window.cursor_position() {
            // If last mouse position is not set, initialize it
            if controller.last_mouse_position.is_none() {
                controller.last_mouse_position = Some(cursor_position);
                return;  // No action on first click
            }

            if let Some(last_position) = controller.last_mouse_position {
                let delta = cursor_position - last_position;

                // Panning: both mouse buttons pressed
                if mouse_button_input.pressed(MouseButton::Left) && mouse_button_input.pressed(MouseButton::Right) {
                    // Adjust the panning target along world X and Y axes
                    let right = Vec3::X * -delta.x * controller.sensitivity * 0.5;  // Pan left/right in world space
                    let up = Vec3::Y * delta.y * controller.sensitivity * 0.5;     // Pan up/down in world space

                    // Adjust the pan target in world space
                    controller.pan_target += right + up;
                }

                // Rotation: only the left button is pressed
                else if mouse_button_input.pressed(MouseButton::Left) {
                    if delta.length_squared() > 0.001 {
                        controller.yaw -= delta.x * controller.sensitivity;
                        controller.pitch -= delta.y * controller.sensitivity;
                    }

                    // Clamp pitch to avoid flipping the camera
                    controller.pitch = controller.pitch.clamp(-89.0, 89.0);
                }
            }

            // Update last known mouse position for the next frame
            controller.last_mouse_position = Some(cursor_position);
        }

        // Reset last_mouse_position only when no mouse buttons are pressed
        if !mouse_button_input.pressed(MouseButton::Left) && !mouse_button_input.pressed(MouseButton::Right) {
            controller.last_mouse_position = None;
        }

        // Handle mouse wheel for zoom
        for event in wheel_events.read() {
            controller.distance -= event.y * controller.speed * 0.5;
            controller.distance = controller.distance.clamp(2.0, 100.0);  // Restrict zoom range
        }

        // Recalculate the camera's position based on the yaw, pitch, and distance
        let yaw_radians = controller.yaw.to_radians();
        let pitch_radians = controller.pitch.to_radians();

        let x = controller.distance * yaw_radians.cos() * pitch_radians.cos();
        let y = controller.distance * pitch_radians.sin();
        let z = controller.distance * yaw_radians.sin() * pitch_radians.cos();

        // Update the camera's position relative to the pan target
        transform.translation = Vec3::new(x, y, z) + controller.pan_target;

        // Ensure the camera is always looking at the pan target
        transform.look_at(controller.pan_target, Vec3::Y);
    }
}
