mod arm_boundary_tests;
mod arm_continuum_2d_tests;
mod arm_singularity_tests;
mod kinematics_impl_tests;
mod wrist_singularity_tests;

#[cfg(feature = "allow_filesystem")]
mod constraint_test;
#[cfg(feature = "allow_filesystem")]
mod constraint_test_various;
#[cfg(feature = "allow_filesystem")]
mod garde_robotics;
#[cfg(feature = "allow_filesystem")]
mod robot_presets;
#[cfg(feature = "allow_filesystem")]
mod test_from_yaml;
#[cfg(feature = "allow_filesystem")]
mod test_individual_link_positions;
#[cfg(feature = "allow_filesystem")]
mod test_parallelogram;
#[cfg(feature = "allow_filesystem")]
mod test_utils;
#[cfg(feature = "allow_filesystem")]
mod testcases;
#[cfg(feature = "allow_filesystem")]
mod tool_base_test;
#[cfg(feature = "allow_filesystem")]
mod urdf_extractor;
