//! Rust implementation of inverse and forward kinematic solutions for six-axis industrial robots 
//! with a parallel base and spherical wrist

pub mod parameters;
pub mod parameters_robots;

#[cfg(feature = "allow_filesystem")]
pub mod parameters_from_file;

pub mod utils;
pub mod kinematic_traits;
pub mod kinematics_impl;

pub mod constraints;

pub mod tool;

pub mod jacobian;

#[cfg(feature = "allow_filesystem")]
pub mod urdf;
#[cfg(feature = "allow_filesystem")]
pub mod parameter_error;

#[cfg(feature = "allow_filesystem")]
mod simplify_joint_name;

#[cfg(test)]
#[cfg(feature = "allow_filesystem")]
mod tests;







