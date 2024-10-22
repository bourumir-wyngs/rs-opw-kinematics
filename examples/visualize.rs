// Declare the modules from the visualize folder
#[path = "visualize/robot_body_builder.rs"]
mod robot_body_builder;
#[path = "visualize/visualization.rs"]
mod visualization;

#[path = "visualize/camera_controller.rs"]
mod camera_controller;
#[path = "visualize/read_trimesh.rs"]
mod read_trimesh;

fn main() {
  visualization::main_method();
}