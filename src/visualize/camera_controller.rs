use bevy::prelude::*;
use bevy::input::mouse::MouseWheel;
use bevy::math::vec3;
use bevy::window::PrimaryWindow;
use bevy::prelude::Component;
use bevy_egui::EguiContexts;

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
            distance: 5.0,  
            last_mouse_position: None,
            pan_target: vec3(0.0,0.0,1.0),  
        }
    }
}

pub fn camera_controller_system(
    mouse_button_input: Res<Input<MouseButton>>,
    mut wheel_events: EventReader<MouseWheel>,
    mut query: Query<(&mut Transform, &mut CameraController)>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut egui_contexts: EguiContexts
) {
    let egui_ctx = egui_contexts.ctx_mut(); // Call ctx_mut to get the mutable context
    if egui_ctx.wants_pointer_input() || egui_ctx.wants_keyboard_input() {
        // If Egui is handling mouse or keyboard input, skip the camera controller
        return;
    }

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
                    let right = Vec3::Z * delta.y * controller.sensitivity * 0.5;  
                    let up = Vec3::Y * -delta.x * controller.sensitivity * 0.5;    
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
            controller.distance = controller.distance.clamp(0.1, 100.0);  // Restrict zoom range
        }

        // Recalculate the camera's position based on the yaw, pitch, and distance
        let yaw_radians = controller.yaw.to_radians();
        let pitch_radians = controller.pitch.to_radians();

        // Swapping Y and Z axes for the position calculation
        let x = controller.distance * yaw_radians.cos() * pitch_radians.cos();
        let z = controller.distance * pitch_radians.sin();  // Swapped Z for vertical movement
        let y = controller.distance * yaw_radians.sin() * pitch_radians.cos();  // Swapped Y for horizontal movement

        // Update the camera's position relative to the pan target
        transform.translation = Vec3::new(x, y, z) + controller.pan_target;

        // Ensure the camera is always looking at the pan target
        // Use Vec3::Z as the "up" vector instead of Vec3::Y
        transform.look_at(controller.pan_target, Vec3::Z);
    }
}

