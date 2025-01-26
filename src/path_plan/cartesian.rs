//! Cartesian stroke

use crate::altered_pose::alter_poses;
use crate::annotations::{AnnotatedJoints, AnnotatedPose, PathFlags};
use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions, J_TOOL};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::rrt::RRTPlanner;
use crate::utils;
use crate::utils::{assert_pose_eq, dump_joints, dump_solutions, transition_costs};
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

    /// If the robot cannot reach exact orientation due to physical limits, allow that much deviation.
    /// The position that is usually much more sensitive to deviations is not altered.
    pub max_orientation_deviation: f64,

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
            (step_pose, PathFlags::TRACE)
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
        let excluded_joints = if self.cartesian_excludes_tool {
            // Exclude tool for cartesian
            HashSet::from([J_TOOL])
        } else {
            HashSet::with_capacity(0)
        };

        let mut trace = Vec::with_capacity(100 + poses.len() + 10);
        // Push the strategy point, from here the move must be already CARTESIAN
        trace.push(work_path_start.clone());

        // "Complete" trace with all intermediate poses. Onboarding is not included
        // as rrt planner checks that path itself.
        let mut check_trace = Vec::with_capacity(poses.len());
        let mut step = 1;

        let mut pairs_iterator = poses.windows(2);

        while let Some([from, to]) = pairs_iterator.next() {
            let prev = trace
                .last()
                .expect("Should have start and strategy points")
                .clone();

            if self.debug && !prev.flags.contains(PathFlags::ALTERED) {
                // If the previous pose contains slight alteration (see below), this will not match
                assert_pose_eq(&from.pose, &self.robot.forward(&prev.joints), 1E-5, 1E-5);
            }

            match self.step_adaptive_linear_transition(&prev.joints, from, to, 0) {
                Ok(next) => {
                    // This trace contains all intermediate poses (collision check)
                    check_trace.push(next);

                    if self.include_linear_interpolation
                        || !to.flags.contains(PathFlags::LIN_INTERP)
                    {
                        trace.push(AnnotatedJoints {
                            joints: next,
                            flags: to.flags,
                        });
                    }
                }

                Err(failed_transition) => {
                    let mut success = false;
                    // Try with altered pose
                    for altered_to in alter_poses(to, self.max_orientation_deviation) {
                        match self.step_adaptive_linear_transition(
                            &prev.joints,
                            from,
                            &altered_to,
                            0,
                        ) {
                            Ok(next) => {
                                println!(
                                    "    Transition with altered pose {:?} successful",
                                    altered_to
                                );
                                check_trace.push(next);
                                trace.push(AnnotatedJoints {
                                    joints: next,
                                    flags: altered_to.flags,
                                });
                                success = true;
                                break;
                            }
                            Err(_) => { // Try next
                            }
                        }
                    }

                    if !success {
                        self.log_failed_transition(&failed_transition, step);
                        return Err(format!(
                            "Failed to transition at step {} with all alterations tried",
                            step
                        ));
                    }
                }
            }
            if to.flags.contains(PathFlags::TRACE) {
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

        // Parallel check with early stopping
        let collides = check_trace.par_iter().any(|joints| {
            // Check the stop flag at the start of the closure
            if stop.load(Ordering::Relaxed) {
                return true; // If we cancel, treat this as colliding
            }
            self.robot.collides_except(joints, &excluded_joints)
        });

        if collides {
            return Err("Collision or cancel detected".into());
        }
        if self.debug {
            println!("Cartesian planning took {:?}", started.elapsed());
        }
        Ok(trace)
    }

    // Transition cartesian way from 'from' into 'to' while assuming 'from'
    // Returns the resolved end pose.
    fn step_adaptive_linear_transition(
        &self,
        starting: &Joints,
        from: &AnnotatedPose, // FK of "starting"
        to: &AnnotatedPose,
        depth: usize,
    ) -> Result<Joints, Transition> {

        pub const DIV_RATIO: f64 = 0.5;

        // Not checked for collisions yet
        let solutions = self
            .robot
            .kinematics
            .inverse_continuing(&to.pose, &starting);
        let mut cheapest_cost = std::f64::MAX;

        // Solutions are already sorted best first
        for next in &solutions {
            // Internal "miniposes" generated through recursion are not checked for collision.
            // They only check agains continuity of the robot movement (no unexpected jerks)
            let cost = transition_costs(starting, next, &self.transition_coefficients);
            if cost <= self.max_transition_cost {
                return Ok(next.clone()); // Track minimal cost observed
            }
            cheapest_cost = cheapest_cost.min(cost);
        }

        // Transitioning not successful.
        // Recursive call reduces step, the goal is to check if there is a continuous
        // linear path on any step.
        if depth < self.linear_recursion_depth {
            // Try to bridge till them middle first, and then from the middle
            // This will result in a shorter distance between from and to.
            let midpose = from.interpolate(to, DIV_RATIO);
            let middle_joints =
                self.step_adaptive_linear_transition(starting, from, &midpose, depth + 1)?;

            // If both bridgings were successful, return the final position that resulted from
            // bridging from middle joints to the final pose on this step
            let end_step =
                self.step_adaptive_linear_transition(&middle_joints, &midpose, to, depth + 1)?;

            let cost_start_mid =
                transition_costs(starting, &middle_joints, &self.transition_coefficients);
            let cost_mid_end =
                transition_costs(&middle_joints, &end_step, &self.transition_coefficients);
            if cost_start_mid <= self.max_transition_cost
                && cost_mid_end <= self.max_transition_cost
            {
                // Both steps are very small
                return Ok(end_step);
            }

            // Cost of any step separately should not be more than cost of the whole transition
            if cost_start_mid <= cheapest_cost && cost_mid_end <= cheapest_cost {
                // Otherwise only recognize if the cost does not rise in the intermediate steps.
                return Ok(end_step);
            }
            return Err(Transition {
                note: format!(
                    "Rising costs as region divided, costs {} and {}, cc {}",
                    cost_start_mid, cost_mid_end, cheapest_cost
                ),
                from: from.clone(),
                to: to.clone(),
                previous: starting.clone(),
                solutions: solutions,
            });
        }

        Err(Transition {
            note: format!(
                "Adaptive linear transition {:?} to {:?} out of recursion depth",
                from, to
            ),
            from: from.clone(),
            to: to.clone(),
            previous: starting.clone(),
            solutions: solutions,
        })
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
        let mut poses = Vec::with_capacity(10 * steps.len() + 2);

        // Add the landing pose if one provided
        if let Some(land) = land {
            poses.push(AnnotatedPose {
                pose: *land,
                flags: PathFlags::LAND,
            });
        }

        // Add the steps and intermediate poses between them
        for step in steps {
            // There may be previous step or landing pose if provided
            if let Some(last) = poses.last() {
                let last = last.clone();
                self.add_intermediate_poses(&last.pose, &step, &mut poses, last.flags);
            }
            poses.push(AnnotatedPose {
                pose: step.clone(),
                flags: PathFlags::TRACE,
            });
        }

        // Add the parking pose if one provided
        if let (Some(park), Some(last)) = (park, poses.last()) {
            self.add_intermediate_poses(
                &last.pose.clone(),
                &park.clone(),
                &mut poses,
                PathFlags::PARK,
            );
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
                if cost > 90_f64.to_radians() {
                    println!(
                        "Transition cost exceeded: {} > {} between {:?} and {:?}",
                        cost, self.max_transition_cost, from.joints, to.joints
                    );
                    return false;
                }
            }
        }

        // If all costs are within the limit, return the total cost
        true
    }
}
