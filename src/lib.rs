//! Rust implementation of inverse and forward kinematic solutions for six-axis industrial robots 
//! with a parallel base and spherical wrist
//!
//! This work builds upon the 2014 paper titled _An Analytical Solution of the Inverse Kinematics Problem of Industrial
//! Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist_, authored by Mathias Brandstötter, Arthur
//! Angerer, and Michael Hofbaur. The paper is available on [ResearchGate](https://www.researchgate.net/profile/Mathias-Brandstoetter/publication/264212870_An_Analytical_Solution_of_the_Inverse_Kinematics_Problem_of_Industrial_Serial_Manipulators_with_an_Ortho-parallel_Basis_and_a_Spherical_Wrist/links/53d2417e0cf2a7fbb2e98b09/An-Analytical-Solution-of-the-Inverse-Kinematics-Problem-of-Industrial-Serial-Manipulators-with-an-Ortho-parallel-Basis-and-a-Spherical-Wrist.pdf).
//! Additionally, it draws inspiration from a similar C++ project, 
//! [Jmeyer1292/opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics), which served as a reference 
//! implementation for generating data for the test suite. This documentation also incorporates the robot diagram 
//! from that project.
//! 
//! # Features
//! 
//! - All returned solutions are valid, normalized, and cross-checked with forward kinematics.
//! - Joint angles can be checked against constraints, ensuring only compliant solutions are returned.
//! - To generate a trajectory of the robot (sequence of poses), it is possible to use "previous joint positions" as
//!   additional input.
//! - If the previous joint positions are provided, the solutions are sorted by proximity to them (closest first).
//!   It is also possible to prioritize proximity to the center of constraints.
//! - For kinematic singularity at J5 = 0° or J5 = ±180° positions, this solver provides reasonable J4 and J6
//!   values close to the previous positions of these joints (and not arbitrary ones that may result in a large jerk 
//!   of the real robot).
//! - Jacobian, torques, and velocities
//! - The robot can be equipped with the tool and placed on the base, planning for the desired location and orientation
//!   of the tool center point (TCP) rather than any part of the robot.
//! - Experimental support for parameter extraction from URDF.
//!  
//! # Parameters
//! 
//! This library uses seven kinematic parameters (_a1, a2, b, c1, c2, c3_, and _c4_). This solver assumes that the arm is
//! at zero when all joints stick straight up in the air, as seen in the image below. It also assumes that all
//! rotations are positive about the base axis of the robot. No other setup is required.
//! 
//! ![OPW Diagram](https://bourumir-wyngs.github.io/rs-opw-kinematics/documentation/opw.gif)
//! 
//! To use the library, fill out an `opw_kinematics::Parameters` data structure.


pub mod parameters;
pub mod parameters_robots;

#[cfg(feature = "allow_filesystem")]
pub mod parameters_from_file;

pub mod utils;
pub mod kinematic_traits;
pub mod kinematics_impl;

pub mod constraints;

pub mod tool;

pub mod frame;

pub mod parallelogram;

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









