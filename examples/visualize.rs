use bevy::app::{App, Startup, Update};
use bevy::DefaultPlugins;
use bevy::pbr::wireframe::WireframePlugin;
use crate::camera_controller::camera_controller_system;
use crate::visualization::{setup};

// Declare the modules from the visualize folder
#[path = "visualize/robot_body_builder.rs"]
mod robot_body_builder;
#[path = "visualize/visualization.rs"]
mod visualization;

#[path = "visualize/camera_controller.rs"]
mod camera_controller;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WireframePlugin)  // Ensure the wireframe plugin is added
        .add_systems(Startup, setup)  // Register the setup system
        .add_systems(Update, camera_controller_system)  // Reintroduce the camera control system
        .run();
}