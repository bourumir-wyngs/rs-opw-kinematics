//! Cartesian stroke

use crate::annotations::{AnnotatedJoints, AnnotatedPose, PathFlags};
use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions, J_TOOL};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::rrt::RRTPlanner;
use crate::utils;
use crate::utils::{assert_pose_eq, dump_joints, dump_solutions};
use nalgebra::{Isometry3, Vector3};
use rayon::prelude::{IntoParallelRefIterator, ParallelIterator};
use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

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

    /// Debug mode for logging
    pub debug: bool,
}

struct Transition {
    from: AnnotatedPose,
    to: AnnotatedPose,
    previous: Joints,
    solutions: Solutions,
}

impl Cartesian<'_> {
    pub fn transitionable(&self, from: &Joints, to: &Joints) -> bool {
        utils::transition_costs(from, to, &self.transition_coefficients) <= self.max_transition_cost
    }

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
        let stop = AtomicBool::new(false);

        for strategy in strategies.iter() {
            println!("Strategy: {:?}", strategy);
            println!("From");
            dump_joints(strategy);
            let result = self.probe_strategy(
                from,
                &AnnotatedJoints {
                    joints: strategy.clone(),
                    flags: land_flags,
                },
                &poses,
                &stop,
            );

            match result {
                Ok(outcome) => {
                    println!("Strategy worked out: {:?}", strategy);
                    stop.store(true, Ordering::Relaxed);
                    return Ok(outcome); // Return the successful outcome
                }
                Err(msg) => {
                    if self.debug {
                        println!("Strategy failed: {:?}, {}", strategy, msg);
                    }
                }
            }
        }

        // If no strategies worked
        Err(format!(
            "No strategy worked out of {} tried",
            strategies.len()
        ))
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

        Ok(trace)
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
            println!("Tool excluded from collision check");
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
            let prev = trace.last().expect("Should have start and strategy points");
            assert_pose_eq(&from.pose, &self.robot.forward(&prev.joints), 1E-5, 1E-5);

            println!("**** Transition {:?} --> {:?} ", from, to);
            println!("     Previous: {:?}", prev);
            println!("     Solutions:");
            let solutions = self
                .robot
                .kinematics
                .inverse_continuing(&to.pose, &prev.joints);
            if solutions.is_empty() {
                println!("No solutions");
            }
            for sol_idx in 0..solutions.len() {
                let mut row_str = String::new();
                for joint_idx in 0..6 {
                    let computed = solutions[sol_idx][joint_idx];
                    row_str.push_str(&format!("{:5.2} ", computed.to_degrees()));
                }
                println!(
                    "[{}] {}",
                    row_str.trim_end(),
                    self.robot.collides(&solutions[sol_idx])
                );
            }

            match self.step_adaptive_linear_transition(&prev.joints, from, to, 0) {
                Ok(next) => {
                    println!("     Adaptive linear transition:");
                    dump_joints(&next);
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
                    self.log_failed_transition(&failed_transition, step);
                    return Err(format!(
                        "Linear stroke does not transit at step {} and cannot be fixed",
                        step
                    )
                    .into());
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
        if self.debug {
            assert_pose_eq(
                &self.robot.kinematics.forward(starting),
                &from.pose,
                1E-5,
                1E-5,
            );
        }

        pub const DIV_RATIO: f64 = 0.5;
        let mut success = false;

        // Not checked for collisions yet
        let solutions = self.robot.kinematics.inverse_continuing(&to.pose, &starting);

        // Solutions are already sorted best first
        for (iteration, next) in solutions.iter().enumerate() {
            println!("  #Probing step {iteration} recursive depth {depth}:");
            dump_joints(&starting);
            dump_joints(next);
            // Internal "miniposes" generated through recursion are not checked for collision.
            // Outer calling code is resposible for check if 'current' is collision-free
            let costs = utils::transition_costs(&starting, next, &self.transition_coefficients);
            if self.transitionable(&starting, next) {
                println!(
                    "  #Transitionable {depth}: costs {} max {}",
                    costs, self.max_transition_cost
                );
                return Ok(next.clone());
            } else {
                println!(
                    "  #NOT Transitionable {depth}: costs {} max {}",
                    costs, self.max_transition_cost
                );
            }
            break;
        }

        // Try recursive call. If succeeds, assume as done anyway.        
        if !success && depth < self.linear_recursion_depth {

            // Try to bridge till them middle first, and then from the middle
            // This will result in shorter distance between from and to.
            
            let midpose = from.interpolate(to, DIV_RATIO);            
            let middle_joints =
                self.step_adaptive_linear_transition(starting, from, &midpose, depth + 1)?;
            
            // If both bridgings were successful, return final position that resulted from
            // bridging from middle to 'ta'
            return  Ok(self.step_adaptive_linear_transition(&middle_joints, &midpose, to, depth + 1)?);
        }
        
        Err(Transition {
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
        println!("Step {} from [1..n] failed", step);
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
}
