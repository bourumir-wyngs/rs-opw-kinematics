use crate::parameters::opw_kinematics::Parameters;

mod arm_boundary_tests;
mod arm_singularity_tests;
mod j1j2free_tests;
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

fn scale_geometry(mut parameters: Parameters, scale: f64) -> Parameters {
    parameters.a1 *= scale;
    parameters.a2 *= scale;
    parameters.b *= scale;
    parameters.c1 *= scale;
    parameters.c2 *= scale;
    parameters.c3 *= scale;
    parameters.c4 *= scale;
    parameters
}
