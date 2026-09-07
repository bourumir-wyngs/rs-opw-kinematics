use crate::kinematic_traits::{Joints, Kinematics};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::utils::dump_joints;
use rand::{Rng, RngExt};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;
use tracing::debug;

pub use crate::rrt_to::dual_rrt_connect;

const RRT_SMOOTH_DUPLICATE_EPSILON: f64 = 1e-9;

#[derive(Debug)]
/// Defines the RRT planner that relocates the robot between the two positions in a
/// collision free way.
pub struct RRTPlanner {
    /// Step size in the joint space (value in Radians). This should be small
    /// enough to prevent robot colliding with something while moving
    /// in possibly less predictable way between the joints.
    pub step_size_joint_space: f64,

    /// The "max try" parameter of RRT algorithm, reasonable values
    /// are in order 1000 ... 4000
    pub max_try: usize,

    /// Number of shortcut candidates to check after RRT succeeds. Set to zero
    /// to return the raw RRT tree path without simplification.
    pub smooth: usize,

    /// Flag to emit extra diagnostics if required.
    pub debug: bool,
}

impl Default for RRTPlanner {
    fn default() -> Self {
        Self {
            step_size_joint_space: 3_f64.to_radians(),
            max_try: 2000,
            smooth: 0,
            debug: false,
        }
    }
}

fn joint_space_distance(left: &[f64], right: &[f64]) -> f64 {
    assert_eq!(left.len(), right.len());
    left.iter()
        .zip(right)
        .map(|(left, right)| {
            let delta = left - right;
            delta * delta
        })
        .sum::<f64>()
        .sqrt()
}

fn same_configuration(left: &[f64], right: &[f64], epsilon: f64) -> bool {
    left.len() == right.len() && joint_space_distance(left, right) <= epsilon
}

fn interpolate_configuration(from: &[f64], to: &[f64], p: f64) -> Vec<f64> {
    assert_eq!(from.len(), to.len());
    from.iter()
        .zip(to)
        .map(|(from, to)| from + (to - from) * p)
        .collect()
}

fn check_cancelled(stop: &AtomicBool) -> Result<(), String> {
    if stop.load(Ordering::Relaxed) {
        Err("Cancelled".to_string())
    } else {
        Ok(())
    }
}

fn clone_path(path: &[Vec<f64>], stop: &AtomicBool) -> Result<Vec<Vec<f64>>, String> {
    check_cancelled(stop)?;
    let mut cloned = Vec::with_capacity(path.len());
    for configuration in path {
        check_cancelled(stop)?;
        cloned.push(configuration.clone());
    }
    check_cancelled(stop)?;
    Ok(cloned)
}

fn edge_is_free<FF>(
    from: &[f64],
    to: &[f64],
    step_size_joint_space: f64,
    collision_free: &mut FF,
    stop: &AtomicBool,
) -> Result<bool, String>
where
    FF: FnMut(&[f64]) -> bool,
{
    check_cancelled(stop)?;
    if !step_size_joint_space.is_finite() || step_size_joint_space <= 0.0 {
        return Ok(false);
    }

    let distance = joint_space_distance(from, to);
    let steps = (distance / step_size_joint_space).ceil() as usize;
    for step in 1..steps {
        check_cancelled(stop)?;
        let p = step as f64 / steps as f64;
        let candidate = interpolate_configuration(from, to, p);
        let valid = collision_free(&candidate);
        check_cancelled(stop)?;
        if !valid {
            return Ok(false);
        }
    }
    check_cancelled(stop)?;
    Ok(true)
}

fn remove_adjacent_duplicates(
    path: &[Vec<f64>],
    stop: &AtomicBool,
) -> Result<Vec<Vec<f64>>, String> {
    check_cancelled(stop)?;
    let mut deduplicated: Vec<Vec<f64>> = Vec::with_capacity(path.len());
    for configuration in path {
        check_cancelled(stop)?;
        if deduplicated.last().is_none_or(|last| {
            !same_configuration(last, configuration, RRT_SMOOTH_DUPLICATE_EPSILON)
        }) {
            deduplicated.push(configuration.clone());
        }
    }
    check_cancelled(stop)?;
    Ok(deduplicated)
}

fn resample_path(
    path: &[Vec<f64>],
    step_size_joint_space: f64,
    stop: &AtomicBool,
) -> Result<Vec<Vec<f64>>, String> {
    check_cancelled(stop)?;
    if path.len() < 2 || !step_size_joint_space.is_finite() || step_size_joint_space <= 0.0 {
        return clone_path(path, stop);
    }

    let mut resampled = Vec::with_capacity(path.len());
    resampled.push(path[0].clone());
    for pair in path.windows(2) {
        check_cancelled(stop)?;
        let from = &pair[0];
        let to = &pair[1];
        let distance = joint_space_distance(from, to);
        let steps = (distance / step_size_joint_space).ceil() as usize;

        for step in 1..steps {
            check_cancelled(stop)?;
            let p = step as f64 / steps as f64;
            resampled.push(interpolate_configuration(from, to, p));
        }

        if resampled
            .last()
            .is_none_or(|last| !same_configuration(last, to, RRT_SMOOTH_DUPLICATE_EPSILON))
        {
            resampled.push(to.clone());
        }
    }
    check_cancelled(stop)?;
    Ok(resampled)
}

fn random_shortcut_indices(path_len: usize, rng: &mut impl Rng) -> Option<(usize, usize)> {
    if path_len < 3 {
        return None;
    }

    let gap = rng.random_range(2..path_len);
    let start_index = rng.random_range(0..(path_len - gap));
    Some((start_index, start_index + gap))
}

/// Simplifies a raw RRT path using bounded shortcut attempts.
///
/// `smooth` is a budget measured in candidate shortcut edges. A zero budget
/// returns the original path unchanged. Positive budgets remove adjacent
/// duplicates, randomly sample shortcut candidates across the current path,
/// replace subpaths with direct joint-space segments that pass collision
/// checks, and resample accepted shortcuts so adjacent returned states stay
/// within `step_size_joint_space`.
///
/// If cancellation is observed during smoothing, returns a copy of the original
/// path. This preserves a complete path rather than returning partial work.
pub fn smooth_rrt_path<FF>(
    path: &[Vec<f64>],
    step_size_joint_space: f64,
    smooth: usize,
    collision_free: FF,
    stop: &AtomicBool,
) -> Vec<Vec<f64>>
where
    FF: FnMut(&[f64]) -> bool,
{
    try_smooth_rrt_path(path, step_size_joint_space, smooth, collision_free, stop)
        .unwrap_or_else(|_| path.to_vec())
}

fn try_smooth_rrt_path<FF>(
    path: &[Vec<f64>],
    step_size_joint_space: f64,
    smooth: usize,
    mut collision_free: FF,
    stop: &AtomicBool,
) -> Result<Vec<Vec<f64>>, String>
where
    FF: FnMut(&[f64]) -> bool,
{
    check_cancelled(stop)?;
    if smooth == 0 {
        return clone_path(path, stop);
    }

    let mut smoothed = remove_adjacent_duplicates(path, stop)?;
    let mut rng = rand::rng();

    for _ in 0..smooth {
        check_cancelled(stop)?;

        let Some((start_index, end_index)) = random_shortcut_indices(smoothed.len(), &mut rng)
        else {
            break;
        };

        if edge_is_free(
            &smoothed[start_index],
            &smoothed[end_index],
            step_size_joint_space,
            &mut collision_free,
            stop,
        )? {
            smoothed.drain((start_index + 1)..end_index);
        }
    }

    resample_path(&smoothed, step_size_joint_space, stop)
}

impl RRTPlanner {
    /// Plans a path from `start` to `goal` joint configuration,
    /// using `KinematicsWithShape` for collision checking.
    /// start and goal are included into the returned path.
    fn plan_path(
        &self,
        kinematics: &KinematicsWithShape,
        start: &Joints,
        goal: &Joints,
        stop: &AtomicBool,
    ) -> Result<Vec<Vec<f64>>, String> {
        check_cancelled(stop)?;
        //return Ok(vec![Vec::from(start.clone()), Vec::from(goal.clone())]);

        let mut collision_free = |joint_angles: &[f64]| -> bool {
            let joints = &<Joints>::try_from(joint_angles).expect("Cannot convert vector to array");
            !kinematics.collides(joints)
        };

        // Constraint compliant random joint configuration generator.
        let random_joint_angles = || -> Vec<f64> {
            // RRT requires vector and we return array so convert
            kinematics
                .constraints()
                .expect("Set joint ranges on kinematics")
                .random_angles()
                .to_vec()
        };

        // Plan the path with RRT

        let path = dual_rrt_connect(
            start,
            goal,
            &mut collision_free,
            random_joint_angles,
            self.step_size_joint_space, // Step size in joint space
            self.max_try,               // Max iterations
            stop,
        )?;

        check_cancelled(stop)?;
        if self.smooth == 0 {
            return Ok(path);
        }

        try_smooth_rrt_path(
            &path,
            self.step_size_joint_space,
            self.smooth,
            collision_free,
            stop,
        )
    }

    fn convert_result(
        &self,
        data: Result<Vec<Vec<f64>>, String>,
        stop: &AtomicBool,
    ) -> Result<Vec<Joints>, String> {
        check_cancelled(stop)?;
        let vectors = data?;
        let mut joints = Vec::with_capacity(vectors.len());
        for vector in vectors {
            check_cancelled(stop)?;
            if vector.len() != 6 {
                return Err("One of the inner vectors does not have 6 elements.".to_string());
            }
            joints.push([
                vector[0], vector[1], vector[2], vector[3], vector[4], vector[5],
            ]);
        }
        check_cancelled(stop)?;
        Ok(joints)
    }

    #[allow(dead_code)]
    fn print_summary(&self, planning_result: &Result<Vec<[f64; 6]>, String>) {
        match planning_result {
            Ok(path) => {
                println!("Steps:");
                for step in path {
                    dump_joints(step);
                }
            }
            Err(error_message) => {
                println!("Error: {}", error_message);
            }
        }
    }

    /// Plans collision - free relocation from 'start' into 'goal', using
    /// provided instance of KinematicsWithShape for both inverse kinematics and
    /// collision avoidance.
    /// Returns `Err("Cancelled")` when cancellation is observed during planning,
    /// smoothing, or conversion of the result.
    pub fn plan_rrt(
        &self,
        start: &Joints,
        goal: &Joints,
        kinematics: &KinematicsWithShape,
        stop: &AtomicBool,
    ) -> Result<Vec<Joints>, String> {
        check_cancelled(stop)?;
        if self.debug {
            debug!(?start, ?goal, "RRT started");
        }
        let started = self.debug.then(Instant::now);
        let path = self.plan_path(kinematics, start, goal, stop);
        let result = self.convert_result(path, stop);

        if self.debug {
            match &result {
                Ok(path) => {
                    debug!(steps = path.len(), "RRT succeeded");
                }
                Err(error_message) => {
                    debug!(error = %error_message, "Direct RRT failed");
                }
            }

            if let Some(started) = started {
                debug!(
                    elapsed = ?started.elapsed(),
                    ?start,
                    ?goal,
                    "RRT finished"
                );
            }
        }
        // self.print_summary(&result);

        check_cancelled(stop)?;
        result
    }
}

#[cfg(test)]
mod tests {
    use super::{
        RRTPlanner, remove_adjacent_duplicates, resample_path, smooth_rrt_path, try_smooth_rrt_path,
    };
    use std::sync::atomic::{AtomicBool, Ordering};

    #[test]
    fn cancelled_smoothing_returns_the_complete_original_path() {
        let path = vec![vec![0.0], vec![0.0], vec![2.0]];

        for smooth in [0, 1] {
            let smoothed = smooth_rrt_path(
                &path,
                1.0,
                smooth,
                |_| panic!("cancelled smoothing must not check collisions"),
                &AtomicBool::new(true),
            );

            assert_eq!(smoothed, path);
        }
    }

    #[test]
    fn smoothing_cancellation_during_collision_check_returns_original_path() {
        let path = vec![vec![0.0, 0.0], vec![2.0, 1.0], vec![4.0, 0.0]];

        for valid in [true, false] {
            let stop = AtomicBool::new(false);
            let mut checks = 0;
            let smoothed = smooth_rrt_path(
                &path,
                1.0,
                1,
                |_| {
                    checks += 1;
                    stop.store(true, Ordering::Relaxed);
                    valid
                },
                &stop,
            );

            assert_eq!(smoothed, path);
            assert_eq!(checks, 1);
        }
    }

    #[test]
    fn planner_smoothing_reports_cancellation_during_collision_check() {
        let path = vec![vec![0.0, 0.0], vec![2.0, 1.0], vec![4.0, 0.0]];

        for valid in [true, false] {
            let stop = AtomicBool::new(false);
            let mut checks = 0;
            let result = try_smooth_rrt_path(
                &path,
                1.0,
                1,
                |_| {
                    checks += 1;
                    stop.store(true, Ordering::Relaxed);
                    valid
                },
                &stop,
            );

            assert_eq!(result, Err("Cancelled".to_string()));
            assert_eq!(checks, 1);
        }
    }

    #[test]
    fn cancelled_path_processing_rejects_partial_output() {
        let stop = AtomicBool::new(true);
        let path = vec![vec![0.0], vec![0.0], vec![2.0]];

        assert_eq!(
            remove_adjacent_duplicates(&path, &stop),
            Err("Cancelled".to_string())
        );
        assert_eq!(
            resample_path(&path, 1.0, &stop),
            Err("Cancelled".to_string())
        );
    }

    #[test]
    fn planner_result_conversion_respects_cancellation() {
        let planner = RRTPlanner::default();
        let result =
            planner.convert_result(Ok(vec![vec![0.0; 6], vec![1.0; 6]]), &AtomicBool::new(true));

        assert_eq!(result, Err("Cancelled".to_string()));
    }

    #[test]
    fn smooth_zero_returns_raw_path() {
        let stop = AtomicBool::new(false);
        let path = vec![vec![0.0, 0.0], vec![1.0, 1.0], vec![2.0, 0.0]];

        let smoothed = smooth_rrt_path(&path, 1.0, 0, |_| true, &stop);

        assert_eq!(smoothed, path);
    }

    #[test]
    fn smooth_shortcuts_collision_free_zigzag() {
        let stop = AtomicBool::new(false);
        let path = vec![vec![0.0, 0.0], vec![1.0, 1.0], vec![2.0, 0.0]];

        let smoothed = smooth_rrt_path(&path, 1.0, 1, |_| true, &stop);

        assert_eq!(
            smoothed,
            vec![vec![0.0, 0.0], vec![1.0, 0.0], vec![2.0, 0.0]]
        );
    }

    #[test]
    fn smooth_positive_budget_removes_adjacent_duplicates() {
        let stop = AtomicBool::new(false);
        let path = vec![vec![0.0, 0.0], vec![0.0, 0.0], vec![1.0, 0.0]];

        let smoothed = smooth_rrt_path(&path, 1.0, 1, |_| true, &stop);

        assert_eq!(smoothed, vec![vec![0.0, 0.0], vec![1.0, 0.0]]);
    }

    #[test]
    fn smooth_keeps_subpath_when_shortcut_collides() {
        let stop = AtomicBool::new(false);
        let path = vec![vec![0.0, 0.0], vec![0.6, 0.4], vec![1.2, 0.0]];

        let smoothed = smooth_rrt_path(
            &path,
            1.0,
            1,
            |configuration| configuration[1].abs() > 0.2,
            &stop,
        );

        assert_eq!(smoothed, path);
    }
}
