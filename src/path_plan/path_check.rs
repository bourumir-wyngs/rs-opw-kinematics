use rayon::prelude::*;
use crate::kinematic_traits::Joints;


/// Checks a series of joint configurations (`steps`) for collisions between consecutive poses.
/// These checks are done in parallel so should be fast.
///
/// # Parameters
/// - `steps`: A reference to a vector of `Joints`, representing the sequence of joint configurations 
///            to be checked for collisions.
///
/// # Returns
/// - `Option<usize>`: The wrapped index of the first detected collision in the `steps` vector or None
///            if no collisions have been found

pub fn check_path(
    steps: &Vec<Joints>,
) -> Option<usize> {
    // Helper function to check if transition from previous is possible, and
    // the current pose does not collide with previous.
    fn is_colliding(prev: Option<&Joints>, current: &Joints) -> bool {
        if let Some(prev_joints) = prev {
            // Perform collision check with both the previous and current pose
            println!("Checking collision between {:?} and {:?}", prev_joints, current);
        } else {
            // Perform collision check for the first pose
            println!("Checking collision for the first pose: {:?}", current);
        }
        false // Replace with actual collision logic
    }

    // Build the task vector with indices and perform collision checks in parallel.
    let collision_index = (0..steps.len())
        .into_par_iter()
        .find_first(|&i| {
            let prev = if i > 0 { Some(&steps[i - 1]) } else { None };
            let current = &steps[i];
            is_colliding(prev, current)
        });

    collision_index
}
