//! Cartesian stroke

use crate::annotations::{AnnotatedJoints, AnnotatedPose, PathFlags};
use crate::interpolator::Interpolator;
use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions, J_TOOL};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::rrt::RRTPlanner;
use crate::utils;
use crate::utils::{assert_pose_eq, dump_joints, dump_solutions, transition_costs};
use bevy::render::render_resource::encase::private::RuntimeSizedArray;
use bitflags::Flags;
use nalgebra::{Isometry3, Vector3};
use rayon::prelude::{IntoParallelRefIterator, ParallelIterator};
use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

/// Reasonable default transition costs. Rotation of smaller joints is more tolerable.
/// The sum of all weights is 6.0
pub const DEFAULT_TRANSITION_COSTS: [f64; 6] = [1.2, 1.1, 1.1, 0.9, 0.9, 0.8];

/// Class doing Cartesian planning
pub struct Cartesian<'a> {
    pub robot: &'a KinematicsWithShape,

    /// Check step size in meters. Objects and features of the robotic cell smaller
    /// than this may not be noticed during collision checks.
    pub check_step_m: f64,

    /// Check step size in radians. Objects and features of the robotic cell smaller
    /// than this may not be noticed during collision checks.
    pub check_step_rad: f64,

    /// Maximum allowed transition cost between Joints
    pub max_transition_cost: f64,

    /// Maximal rotation of any joint per step under any conditions. Set this to a large value,
    /// it is only to protect the hardware against something unexpected. If you exceed this limit,
    /// reduce the check_step_m to obtain a more fine-grained trajectory.
    pub max_step_cost: f64,

    /// Transition cost coefficients (smaller joints are allowed to rotate more)
    pub transition_coefficients: Joints,

    /// If movement between adjacent poses results transition costs over this threshold,
    /// the Cartesian segment is divided by half, checking boths side separatedly while
    /// collision checking also the middle segment.
    pub linear_recursion_depth: usize,

    /// RRT planner that plans the onboarding part, and may potentially plan other
    /// parts that involve collision free relocation of the robot, but without
    /// Cartesian (linear) movement.
    pub rrt: RRTPlanner,

    /// If set, linear interpolated poses are included in the output.
    /// Otherwise, they are discarded, many robots can do Cartesian stroke
    /// much better on they own
    pub include_linear_interpolation: bool,

    /// If set, tool is only checked for collision on joint (RRT) relocations
    /// but not on cartesian stroke. This is necessary if the tool comes into
    /// contact or very close to the target surface when the robot is working
    /// on it during Cartesian strokes.
    pub cartesian_excludes_tool: bool,

    /// Time out for path planning
    pub time_out_seconds: u64,

    /// Debug mode for logging
    pub debug: bool,
}

struct Transition {
    note: String,
    from: AnnotatedPose,
    to: AnnotatedPose,
    previous: Joints,
    solutions: Solutions,
}

impl Cartesian<'_> {
    /// Path plan for the given vector of poses. The returned path must be transitionable
    /// and collision free.
    pub fn plan(
        &self,
        from: &Joints,
        land_first: &Option<Pose>,
        steps: &Vec<Pose>,
        park: &Option<Pose>,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        if self.robot.collides(from) {
            return Err("Onboarding point collides".into());
        }

        let (land, land_flags) = if let Some(land_pose) = land_first {
            (land_pose, PathFlags::LAND)
        } else if let Some(step_pose) = steps.first() {
            (step_pose, PathFlags::STROKE)
        } else if let Some(park_pose) = park {
            (park_pose, PathFlags::PARKING)
        } else {
            return Ok(vec![]); // No job to do
        };

        let strategies = self.robot.inverse_continuing(land, from);
        if strategies.is_empty() {
            return Err("Unable to start from onboarding point".into());
        }
        println!("Strategies to try: {}", strategies.len());

        let poses = self.with_intermediate_poses(land_first, &steps, park);

        // Global stop once a solution is found
        let stop = Arc::new(AtomicBool::new(false));
        let stop_borrow = Arc::clone(&stop);

        // Set the expiration time (e.g., 5 seconds)
        let expiration_time = Duration::from_secs(self.time_out_seconds);
        thread::spawn(move || {
            // Wait for the duration to expire
            thread::sleep(expiration_time);
            if !stop_borrow.load(Ordering::Relaxed) {
                println!(
                    "ABORT: No solution found after {} seconds",
                    expiration_time.as_secs()
                );
                stop_borrow.store(true, Ordering::Relaxed);
            }
        });

        if true {
            // Parallel version
            strategies
                .par_iter()
                .find_map_any(|strategy| {
                    match self.probe_strategy(
                        from,
                        &AnnotatedJoints {
                            joints: strategy.clone(),
                            flags: land_flags,
                        },
                        &poses,
                        &stop,
                    ) {
                        Ok(outcome) => {
                            println!("Strategy worked out: {:?}", strategy);
                            stop.store(true, Ordering::Relaxed);
                            Some(Ok(outcome))
                        }
                        Err(msg) => {
                            if self.debug {
                                println!("Strategy failed: {:?}, {}", strategy, msg);
                            }
                            None // Continue searching
                        }
                    }
                })
                .unwrap_or_else(|| {
                    Err(format!(
                        "No strategy worked out of {} tried",
                        strategies.len()
                    ))
                })
        } else {
            // Sequential version
            strategies
                .iter()
                .find_map(|strategy| {
                    // Sequentially check each strategy
                    match self.probe_strategy(
                        from,
                        &AnnotatedJoints {
                            joints: strategy.clone(),
                            flags: land_flags,
                        },
                        &poses,
                        &stop,
                    ) {
                        Ok(outcome) => {
                            println!("Strategy worked out: {:?}", strategy);
                            stop.store(true, Ordering::Relaxed);
                            Some(Ok(outcome)) // Solution found, return it
                        }
                        Err(msg) => {
                            if self.debug {
                                println!("Strategy failed: {:?}, {}", strategy, msg);
                            }
                            None // Continue searching
                        }
                    }
                })
                .unwrap_or_else(|| {
                    // No valid strategy found
                    Err(format!(
                        "No strategy worked out of {} tried",
                        strategies.len()
                    ))
                })
        }
    }

    /// Computes pose, moved by the distance dz in the local orientation of the Isometry3.
    /// This is used for computing starting and landing poses if the Cartesian path needs
    /// to exclude the tool from collision check (as it touches the working surface during
    /// operation)
    ///
    /// This function takes Option and returns None if None is passed. This is handy
    /// when start and end poses are taken from the list that might be empty
    ///
    /// positive value of z means moving in negative direction (lifting up)
    pub fn elevated_z(isometry: Option<&Isometry3<f64>>, dz: f64) -> Option<Isometry3<f64>> {
        if let Some(isometry) = isometry {
            // Extract the rotation component as a UnitQuaternion
            let rotation = isometry.rotation;

            // Determine the local Z-axis direction (quaternion's orientation)
            let local_z_axis = rotation.transform_vector(&Vector3::z());

            // Compute the new translation by adding dz along the local Z-axis
            let translation = isometry.translation.vector - dz * local_z_axis;

            // Return a new Isometry3 with the updated translation and the same rotation
            Some(Isometry3::from_parts(translation.into(), rotation))
        } else {
            None
        }
    }

    fn probe_strategy(
        &self,
        start: &Joints,
        work_path_start: &AnnotatedJoints,
        poses: &Vec<AnnotatedPose>,
        stop: &AtomicBool,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        // Use Rayon to run both functions in parallel
        let (onboarding, stroke) = rayon::join(
            || {
                self.rrt
                    .plan_rrt(start, &work_path_start.joints, self.robot, stop)
            },
            || self.compute_strategy(work_path_start, &poses, stop),
        );

        if stop.load(Ordering::Relaxed) {
            return Err("Stopped".into());
        }

        // Use `?` to propagate errors upwards
        let onboarding = onboarding?;
        let stroke = stroke?;

        // Combine results into `trace`
        let mut trace = Vec::with_capacity(onboarding.len() + stroke.len() + 10);

        // Add onboarding, omitting the last entry
        trace.extend(
            onboarding
                .iter()
                .take(onboarding.len() - 1)
                .map(|joints| AnnotatedJoints {
                    joints: *joints,
                    flags: PathFlags::ONBOARDING,
                }),
        );

        // Add stroke
        trace.extend(stroke);

        // Remove close poses
        let trace = self.constant_speed(trace);

        // Parallel check with early stopping
        let excluded_joints = if self.cartesian_excludes_tool {
            // Exclude tool for cartesian
            HashSet::from([J_TOOL])
        } else {
            HashSet::with_capacity(0)
        };
        
        let collides = trace.par_iter().any(|joints| {
            // Check the stop flag at the start of the closure
            if stop.load(Ordering::Relaxed) {
                return true; // If we cancel, treat this as colliding
            }
            self.robot.collides_except(&joints.joints, &excluded_joints)
        });

        if collides {
            return Err("Collision or cancel detected".into());
        }

        if self.check_costs(&trace) {
            Ok(trace)
        } else {
            Err("Transition costs too high".into())
        }
    }

    /// Probe the given strategy
    fn compute_strategy(
        &self,
        work_path_start: &AnnotatedJoints,
        poses: &Vec<AnnotatedPose>,
        stop: &AtomicBool,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        println!("Cartesian planning started, computing strategy {work_path_start:?}");

        let started = Instant::now();
        let mut trace = Vec::with_capacity(100 + poses.len() + 10);
        // Push the strategy point, from here the move must be already CARTESIAN
        trace.push(work_path_start.clone());

        let mut step = 1;

        let mut pairs_iterator = poses.windows(2);

        while let Some([from, to]) = pairs_iterator.next() {
            let prev = trace
                .last()
                .expect("Should have start and strategy points")
                .clone();

            if self.debug && !prev.flags.contains(PathFlags::GAP_CLOSING) {
                // If the previous pose contains slight alteration (see below), this will not match
                assert_pose_eq(&from.pose, &self.robot.forward(&prev.joints), 1E-5, 1E-5);
            }

            let transition = self.step_adaptive_linear_transition(&prev.joints, from, to, 0);
            match transition {
                Ok(extension) => {
                    for (p, step) in extension.iter().enumerate() {
                        let flags = if p < extension.len() - 1 {
                            (to.flags | PathFlags::LIN_INTERP)
                                & !(PathFlags::STROKE | PathFlags::PARK)
                        } else {
                            to.flags
                        };
                        trace.push(AnnotatedJoints {
                            joints: step.clone(),
                            flags: flags,
                        });
                    }
                }

                Err(failed_transition) => {
                    let mut success = false;
                    // Try with altered pose
                    println!(
                        "Closing step {:?} with RRT as {:?} ",
                        step, failed_transition.note
                    );
                    let solutions = self.robot.inverse_continuing(&to.pose, &prev.joints);
                    for next in solutions {
                        let path = self.rrt.plan_rrt(&prev.joints, &next, self.robot, stop);
                        if let Ok(path) = path {
                            println!("  ... closed with RRT {} steps", path.len());
                            for step in path {
                                trace.push(AnnotatedJoints {
                                    joints: step,
                                    flags: (to.flags | PathFlags::GAP_CLOSING)
                                        & !(PathFlags::LIN_INTERP | PathFlags::STROKE),
                                });
                            }
                            success = true;
                            break;
                        }
                    }

                    if !success {
                        self.log_failed_transition(&failed_transition, step);
                        return Err(format!("Failed to transition at step {}", step));
                    }
                }
            }
            if to.flags.contains(PathFlags::STROKE) {
                step += 1;
            }
        }

        if self.debug {
            println!(
                "Cartesian planning till collision check took {:?}",
                started.elapsed()
            );
        }
        if stop.load(Ordering::Relaxed) {
            return Err("Stopped".into());
        }

        Ok(trace)
    }

    /// Transition cartesian way from 'from' into 'to' while assuming 'from'
    /// Returns all path, not including "starting", that should not be empty
    /// as it succeeded. Returns description of the transition on failure.
    fn step_adaptive_linear_transition(
        &self,
        starting: &Joints,
        from: &AnnotatedPose, // FK of "starting"
        to: &AnnotatedPose,
        depth: usize,
    ) -> Result<Vec<Joints>, Transition> {
        pub const DIV_RATIO: f64 = 0.5;

        // Not checked for collisions yet
        let solutions = self
            .robot
            .kinematics
            .inverse_continuing(&to.pose, &starting);

        // Solutions are already sorted best first
        for next in &solutions {
            // Internal "miniposes" generated through recursion are not checked for collision.
            // They only check agains continuity of the robot movement (no unexpected jerks)
            let cost = transition_costs(starting, next, &self.transition_coefficients);
            if cost <= self.max_transition_cost {
                return Ok(vec![next.clone()]); // Track minimal cost observed
            }
        }

        // Transitioning not successful.
        // Recursive call reduces step, the goal is to check if there is a continuous
        // linear path on any step.
        if depth < self.linear_recursion_depth {
            // Try to bridge till them middle first, and then from the middle
            // This will result in a shorter distance between from and to.
            let midpose = from.interpolate(to, DIV_RATIO);
            let first_track =
                self.step_adaptive_linear_transition(starting, from, &midpose, depth + 1)?;
            let mid_step = first_track.last().unwrap().clone();

            // If both bridgings were successful, return the final position that resulted from
            // bridging from middle joints to the final pose on this step
            let second_track =
                self.step_adaptive_linear_transition(&mid_step, &midpose, to, depth + 1)?;

            Ok(first_track
                .into_iter()
                .chain(second_track.into_iter())
                .collect())
        } else {
            Err(Transition {
                note: format!(
                    "Adaptive linear transition {:?} to {:?} out of recursion depth {}",
                    from, to, depth
                ),
                from: from.clone(),
                to: to.clone(),
                previous: starting.clone(),
                solutions: solutions,
            })
        }
    }

    fn log_failed_transition(&self, transition: &Transition, step: i32) {
        if !self.debug {
            return;
        }
        println!("Step {} from [1..n] failed: {}", step, transition.note);
        println!(
            "No transition with cost below {}:",
            self.max_transition_cost.to_degrees()
        );
        println!(
            "   from: {:?} collides {}",
            transition.from,
            self.robot.collides(&transition.previous)
        );
        dump_joints(&transition.previous);
        println!("   to: {:?}", transition.to);
        println!("   Possible transitions:");
        for s in &transition.solutions {
            dump_joints(s);
            println!(
                "   transition {} collides: {}",
                utils::transition_costs(&transition.previous, s, &DEFAULT_TRANSITION_COSTS)
                    .to_degrees(),
                self.robot.collides(s)
            );
        }
    }

    fn with_intermediate_poses(
        &self,
        land: &Option<Pose>,
        steps: &Vec<Pose>,
        park: &Option<Pose>,
    ) -> Vec<AnnotatedPose> {
        let mut poses = Vec::with_capacity(steps.len() + 2);

        // Add the landing pose if one provided
        if let Some(land) = land {
            poses.push(AnnotatedPose {
                pose: *land,
                flags: PathFlags::LAND,
            });
        }

        // Add the steps and intermediate poses between them
        for step in steps {
            poses.push(AnnotatedPose {
                pose: step.clone(),
                flags: PathFlags::STROKE,
            });
        }

        // Add the parking pose if one provided
        if let Some(park) = park {
            poses.push(AnnotatedPose {
                pose: park.clone(),
                flags: PathFlags::PARK,
            });
        }

        poses
    }

    /// Add intermediate poses. start and end poses are not added.
    fn add_intermediate_poses(
        &self,
        start: &Pose,
        end: &Pose,
        poses: &mut Vec<AnnotatedPose>,
        flags: PathFlags,
    ) {
        // Calculate the translation difference and distance
        let translation_diff = end.translation.vector - start.translation.vector;
        let translation_distance = translation_diff.norm();

        // Calculate the rotation difference and angle
        let rotation_diff = end.rotation * start.rotation.inverse();
        let rotation_angle = rotation_diff.angle();

        // Calculate the number of steps required for translation and rotation
        let translation_steps = (translation_distance / self.check_step_m).ceil() as usize;
        let rotation_steps = (rotation_angle / self.check_step_rad).ceil() as usize;

        // Choose the greater step count to achieve finer granularity between poses
        let steps = translation_steps.max(rotation_steps).max(1);

        // Calculate incremental translation and rotation per chosen step count
        let translation_step = translation_diff / steps as f64;

        // Generate each intermediate pose. Start and end poses are excluded.
        for i in 1..steps {
            let fraction = i as f64 / steps as f64;

            // Interpolate translation and rotation
            let intermediate_translation = start.translation.vector + translation_step * i as f64;
            let intermediate_rotation = start.rotation.slerp(&end.rotation, fraction);

            // Construct the intermediate pose
            let intermediate_pose =
                Pose::from_parts(intermediate_translation.into(), intermediate_rotation);

            poses.push(AnnotatedPose {
                pose: intermediate_pose,
                flags: flags | PathFlags::LIN_INTERP,
            });
        }
    }

    fn check_costs(&self, steps: &Vec<AnnotatedJoints>) -> bool {
        use crate::utils::transition_costs;

        // Pairwise iterate over adjacent steps and check transition costs
        for pair in steps.windows(2) {
            if let [from, to] = pair {
                let cost =
                    transition_costs(&from.joints, &to.joints, &self.transition_coefficients);
                if cost > self.max_step_cost {
                    println!(
                        "Transition cost exceeded: {:.1} > {} between: {:?} and {:?},",
                        cost.to_degrees(),
                        self.max_step_cost.to_degrees(),
                        from,
                        to
                    );
                    //return false;
                }
            }
        }

        // If all costs are within the limit, return the total cost
        true
    }

    fn constant_speed(&self, steps: Vec<AnnotatedJoints>) -> Vec<AnnotatedJoints> {
        self.generate_constant_speed_path(&steps, 0.1, 0.1)
    }

    fn tcp_speed(&self, steps: &Vec<AnnotatedJoints>) {
        if steps.len() < 2 {
            println!("Not enough steps to calculate TCP speed.");
            return;
        }

        const TIME_STEP: f64 = 1.0; // Assuming a nominal time step of 1 second

        println!("TCP speeds ({} steps):", steps.len());
        for window in steps.windows(2) {
            if let [prev, next] = window {
                // Compute the TCP poses for consecutive joints
                let prev_pose = self.robot.forward(&prev.joints);
                let next_pose = self.robot.forward(&next.joints);

                // Calculate the translational distance between TCP poses
                let distance = (next_pose.translation.vector - prev_pose.translation.vector).norm();

                // Calculate speed (distance divided by time step)
                let speed = distance / TIME_STEP;

                // Print the speed and details of the poses
                println!(
                    "TCP Speed: {:.4} (from {:.5?} to {:.5?})",
                    speed, prev_pose.translation.vector, next_pose.translation.vector
                );
            }
        }
    }

    pub fn generate_constant_speed_path(
        &self,
        steps: &Vec<AnnotatedJoints>, // Original trajectory waypoints
        v_tcp: f64,                   // Desired TCP speed
        dt: f64,                      // Time step
    ) -> Vec<AnnotatedJoints> {
        let strong_interpolator = Interpolator::new(steps.clone());

        let mut arc_lengths = Vec::new();
        let mut cumulative_distance = 0.0;

        // Step 1: Sample the interpolator to compute TCP arc lengths
        let mut sampled_poses = Vec::new();
        let num_samples = 12 * steps.len(); // Initial fine sampling resolution
        for i in 0..=num_samples {
            let t = i as f64 / num_samples as f64; // Timeline parameter in [0, 1]
            let sample = strong_interpolator.interpolate(t);
            let tcp_pose = self.robot.forward(&sample.joints);

            // Store sampled TCP pose for distance computation
            sampled_poses.push(tcp_pose);

            // Compute distance from the previous sample (skip first sample)
            if i > 0 {
                let dist =
                    (tcp_pose.translation.vector - sampled_poses[i - 1].translation.vector).norm();
                cumulative_distance += dist;
            }

            arc_lengths.push(cumulative_distance); // Cumulative arc length
        }

        let total_arc_length = cumulative_distance;

        // Step 2: Reparameterize timeline
        let mut timeline_map = Vec::new();
        for i in 0..arc_lengths.len() {
            let s = arc_lengths[i] / total_arc_length; // Normalize arc length
            timeline_map.push((s, i as f64 / num_samples as f64)); // (normalized length, t)
        }

        // Function to interpolate `s -> t` mapping
        let interpolate_t = |s: f64| -> f64 {
            for i in 1..timeline_map.len() {
                let (s_prev, t_prev) = timeline_map[i - 1];
                let (s_next, t_next) = timeline_map[i];

                if s >= s_prev && s <= s_next {
                    // Linear interpolation for simplicity
                    let alpha = (s - s_prev) / (s_next - s_prev);
                    return t_prev + alpha * (t_next - t_prev);
                }
            }
            1.0 // Default to endpoint
        };

        // Step 3: Resample trajectory based on constant TCP speed
        let mut path = Vec::with_capacity(2 * steps.len());
        let step_distance = v_tcp * dt;
        let num_steps = (total_arc_length / step_distance).ceil() as usize;

        let mut s = 0.0;
        let standard_step = 1.0 / num_steps as f64;
        path.push(steps[0].clone());
        while s <= 1.0 {
            let prev = path.last().unwrap();
            let mut step = standard_step;
            loop {
                let t_r = interpolate_t(s + step);

                // Query the strong interpolator for the new parameter
                let next = strong_interpolator.interpolate(t_r);
                if transition_costs(&prev.joints, &next.joints, &self.transition_coefficients)
                    < self.max_step_cost
                {
                    path.push(next);
                    s = s + step;
                    break;
                } else {
                    // Try with smaller step
                    step = 0.9 * step;
                    assert!(step > 0.00001 * standard_step);
                }
            }
        }
        let last_step = steps.last().unwrap();
        let last_path = path.last().unwrap();
        if transition_costs(
            &last_path.joints,
            &last_path.joints,
            &self.transition_coefficients,
        ) > 0.001_f64.to_degrees()
        {
            // Add the last pose if for some reason we are not already at.
            path.push(steps.last().unwrap().clone());
        }

        path
    }

    /// Linear interpolation between two joint configurations.
    fn interpolate_joints(&self, from: &[f64; 6], to: &[f64; 6], t: f64) -> [f64; 6] {
        let mut interpolated = [0.0; 6];
        for i in 0..6 {
            interpolated[i] = from[i] + t * (to[i] - from[i]);
        }
        interpolated
    }
}
