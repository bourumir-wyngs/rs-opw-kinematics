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
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            speed: 10.0,
            sensitivity: 0.1,
            pitch: 0.0,
            yaw: 0.0,
            distance: 20.0,
            last_mouse_position: None,
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

                // Only adjust yaw and pitch if both buttons are not pressed
                if mouse_button_input.pressed(MouseButton::Left) && mouse_button_input.pressed(MouseButton::Right) {
                    // Panning logic: move the camera without changing yaw/pitch
                    let right = transform.rotation * Vec3::X * -delta.x * controller.sensitivity * 0.5;
                    let up = transform.rotation * Vec3::Y * delta.y * controller.sensitivity * 0.5;
                    transform.translation += right + up;
                } else if mouse_button_input.pressed(MouseButton::Left) {
                    // Rotation logic: adjust yaw and pitch when only the left button is pressed
                    if delta.length_squared() > 0.001 {
                        controller.yaw -= delta.x * controller.sensitivity;
                        controller.pitch -= delta.y * controller.sensitivity;
                    }

                    // Clamp pitch to avoid camera flipping
                    controller.pitch = controller.pitch.clamp(-89.0, 89.0);

                    // Calculate new direction based on yaw and pitch
                    let yaw_radians = controller.yaw.to_radians();
                    let pitch_radians = controller.pitch.to_radians();

                    let direction = Vec3::new(
                        yaw_radians.cos() * pitch_radians.cos(),
                        pitch_radians.sin(),
                        yaw_radians.sin() * pitch_radians.cos(),
                    );

                    // Set the camera's new translation and direction
                    transform.translation = direction * controller.distance;
                    transform.look_at(Vec3::ZERO, Vec3::Y);  // Always look at the origin
                }
            }

            // Update last known mouse position
            controller.last_mouse_position = Some(cursor_position);
        }

        // Reset the last mouse position if no mouse buttons are pressed
        if !(mouse_button_input.pressed(MouseButton::Left) || mouse_button_input.pressed(MouseButton::Right)) {
            controller.last_mouse_position = None;
        }

        // Handle mouse wheel for zoom
        for event in wheel_events.read() {
            controller.distance -= event.y * controller.speed * 0.1;
            controller.distance = controller.distance.clamp(2.0, 50.0);

            // Recalculate direction for zoom
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
