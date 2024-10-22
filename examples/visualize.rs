/// This example builds and visualizes complete robot, using Bevy.
/// The visualization provides control bars to change the joint
/// angles, the visualization of the robot is updated accordingly.
/// This visualization is not part of the main library but
/// rather example intended to show that everything works as 
/// expected. You can use the modified version to test your own
/// robot.
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