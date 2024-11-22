//! Cartesian stroke

use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::rrt::RRTPlanner;
use crate::utils;
use crate::utils::{assert_pose_eq, dump_joints, dump_solutions};
use bevy::utils::HashMap;
use bitflags::bitflags;
use nalgebra::Translation3;
use std::fmt;
use std::slice::Windows;

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

    pub rrt: RRTPlanner,

    /// If set, linear interpolated poses are included in the output.
    /// Otherwise, they are discarded, many robots can do Cartesian stroke
    /// much better on they own
    pub include_linear_interpolation: bool,

    /// Debug mode for logging
    pub debug: bool,

    /// Checked collisions
    pub checked: HashMap<[i16; 6], bool>,
}

bitflags! {
    #[derive(Clone, Copy)]
    struct PoseFlags: u32 {
        const TRACE = 0b00000001;
        const LINEAR_INTERPOLATED = 0b00000010;
        const LAND = 0b00000100;
        const PARK = 0b00001000;

        const ORIGINAL = Self::TRACE.bits() | Self::LAND.bits() | Self::PARK.bits();
    }
}

#[derive(Clone, Copy)]
struct AnnotatedPose {
    pub pose: Pose,
    pub flags: PoseFlags,
}

impl AnnotatedPose {
    pub(crate) fn interpolate(&self, other: &AnnotatedPose, p: f64) -> AnnotatedPose {
        assert!((0.0..=1.0).contains(&p));

        // Interpolate translation (linearly)
        let self_translation = &self.pose.translation.vector;
        let other_translation = &other.pose.translation.vector;

        let translation = self_translation.lerp(&other_translation, p);
        let rotation = self.pose.rotation.slerp(&other.pose.rotation, p);

        AnnotatedPose {
            pose: Pose::from_parts(Translation3::from(translation), rotation),
            flags: PoseFlags::LINEAR_INTERPOLATED,
        }
    }
}

impl fmt::Debug for AnnotatedPose {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        fn flag_representation(flags: &PoseFlags) -> String {
            const FLAG_MAP: &[(PoseFlags, &str)] = &[
                (PoseFlags::LINEAR_INTERPOLATED, "LINEAR_INTERPOLATED"),
                (PoseFlags::LAND, "LAND"),
                (PoseFlags::PARK, "PARK"),
                (PoseFlags::TRACE, "TRACE"),
            ];

            FLAG_MAP
                .iter()
                .filter(|(flag, _)| flags.contains(*flag))
                .map(|(_, name)| *name)
                .collect::<Vec<_>>()
                .join(" | ")
        }

        let translation = self.pose.translation.vector;
        let rotation = self.pose.rotation;

        write!(
            formatter,
            "{}: [{:.3}, {:.3}, {:.3}], quat {{ w: {:.3}, i: {:.3}, j: {:.3}, k: {:.3} }}",
            flag_representation(&self.flags),
            translation.x,
            translation.y,
            translation.z,
            rotation.w,
            rotation.i,
            rotation.j,
            rotation.k
        )
    }
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
        &mut self,
        from: &Joints,
        land: &Pose,
        steps: Vec<Pose>,
        park: &Pose,
    ) -> Result<Vec<Joints>, String> {
        // Collision free list
        self.maybe_collides(from, "onbording point")?;
        let strategies = self.robot.inverse_continuing(land, from);
        if strategies.is_empty() {
            return Err("Unable to start from onboarding point".into());
        }
        let poses = self.with_intermediate_poses(land, &steps, park);
        for strategy in &strategies {
            match self.probe_strategy(from, strategy, &poses) {
                Ok(outcome) => return Ok(outcome),
                Err(msg) => {
                    if self.debug {
                        println!("Strategy failed: {}", msg);
                    }
                }
            }
        }
        Err(format!(
            "No strategy worked out of {} tried",
            strategies.len()
        ))
    }

    fn maybe_collides(&mut self, joints: &Joints, note: &str) -> Result<bool, String> {
        if self.collides(joints) {
            let msg = format!("{:?} collides", note);
            if self.debug {
                println!("{}", msg);
                dump_joints(joints);
            }
            return Err(msg);
        }
        Ok(true)
    }

    /// Probe the given strategy
    fn probe_strategy(
        &mut self,
        start: &Joints,
        strategy: &Joints,
        poses: &Vec<AnnotatedPose>,
    ) -> Result<Vec<Joints>, String> {
        let mut trace = self.rrt.plan_rrt(start, strategy, self.robot)?;
        trace.reserve(poses.len());
        let mut istep = 1;

        let mut pairs_iterator = poses.windows(2);

        while let Some([from, to]) = pairs_iterator.next() {
            let prev = trace.last().expect("Should have start and strategy points");
            assert_pose_eq(&from.pose, &self.robot.forward(prev), 1E-5, 1E-5);
            match self.step_adaptive_linear_transition(prev, from, to, 0) {
                Ok(next) => {
                    if self.collides(&next) {
                        let failed_transition = Transition {
                            from: *from,
                            to: *to,
                            previous: *prev,
                            solutions: vec![next],
                        };
                        match self.reconfigure_robot(
                            &mut trace,
                            &failed_transition,
                            istep,
                            &mut pairs_iterator,
                        ) {
                            Ok(next_reconfigured) => {
                                trace.push(next_reconfigured);
                            }
                            Err(message) => {
                                return Err(format!(
                                    "Linear stroke collides at step {} and cannot be fixed: {}",
                                    istep, message
                                )
                                .into());
                            }
                        }
                    } else {
                        if self.include_linear_interpolation
                            || !to.flags.contains(PoseFlags::LINEAR_INTERPOLATED)
                        {
                            trace.push(next);
                        }
                    }
                }
                Err(failed_transition) => {
                    match self.reconfigure_robot(
                        &mut trace,
                        &failed_transition,
                        istep,
                        &mut pairs_iterator,
                    ) {
                        Ok(next_reconfigured) => {
                            trace.push(next_reconfigured);
                        }
                        Err(message) => {
                            return Err(format!(
                                "Linear stroke does not transit at step {} and cannot be fixed: {}",
                                istep, message
                            )
                            .into());
                        }
                    }
                }
            }
            if to.flags.contains(PoseFlags::TRACE) {
                istep += 1;
            }
        }

        // Check last step that does not get into 'prev' as the loop exits.
        let last_step = trace.last().expect("Empty trace after planning");
        self.maybe_collides(last_step, &String::from("last point"))?;

        Ok(trace)
    }

    /// Move positions that cannot be covered by Cartesian stroke in the
    /// current configuration. The position still can be covered if we
    /// allow arbitrarily large jumps, explicitly reconfiguring robot
    /// with explicit path planning.
    ///
    /// First failed transition is covered by failed_transition, but
    /// more failing poses may follow. The iterator is pointing right
    /// after the pair that produced the failed transition
    /// This method returns the joints of the position after fixing ('next').
    /// It adds all fixup (repositioning) code to the trace.
    fn reconfigure_robot(
        &self,
        trace: &mut Vec<Joints>,
        failed_transition: &Transition,
        istep: i32,
        window: &mut Windows<AnnotatedPose>,
    ) -> Result<Joints, String> {
        self.log_failed_transition(&failed_transition, istep);

        // LOOP: Here must be start if the loop
        let mut target_pose = failed_transition.to.clone();
        loop {
            if let Some(previous) = trace.last() {
                // We cannot smoothly continue this stroke. Can we in general visit this position?
                let solutions = self.robot.inverse_continuing(&target_pose.pose, previous);
                if solutions.is_empty() {
                    return Err("No non-colliding solutions found".into());
                } else {
                    println!("Alternative solutions found");
                    dump_solutions(&solutions);
                    for solution in &solutions {
                        println!("Planning reconfiguration");
                        dump_joints(previous);
                        dump_joints(solution);

                        match self.rrt.plan_rrt(previous, solution, self.robot) {
                            Ok(path) => {
                                println!("RRT successful:");
                                for step in &path {
                                    dump_joints(&step);
                                }

                                // Add reconfiguration part to trace. Do not add the first
                                // element as it is the same as from, already present there
                                trace.extend_from_slice(&path[1..]);
                                return Ok(*path.last().unwrap());
                            }
                            Err(error_message) => {
                                println!("RRT not successful, skipping more poses: {}", error_message);
                                // If the window is not empty, get the next target_pose
                                let mut trying_next: Option<&AnnotatedPose> = None;
                                while let Some(next_pose) = window.next() {
                                    if !next_pose[1].flags.contains(PoseFlags::LINEAR_INTERPOLATED) {
                                        // Skip all interpolated poses that come in very minor steps
                                        trying_next = Some(&next_pose[1]);
                                        break;
                                    }
                                }
                                if let Some(trying_next) = trying_next {
                                    println!("Follow-up pose found: {:?}", trying_next);
                                    target_pose = *trying_next;
                                    continue; // Repeat the loop with the updated target_pose
                                } else {
                                    // If no more poses in the window, exit the loop
                                    println!("Follow-up pose not found");
                                    return Err("No more poses to try from the window".into());
                                }
                            }
                        }
                    }
                }
            }
        }
        Err("Robot reconfiguration not possible".into())
    }

    // Transition cartesian way from 'from' into 'to' while assuming 'from'
    // is represented by 'starting'. All new joint positions are
    // checked for collision but 'starting' is not checked (parent call must do this)
    // Returns the resolved end pose.
    fn step_adaptive_linear_transition(
        &mut self,
        starting: &Joints,
        from: &AnnotatedPose,
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

        let midpoint = from.interpolate(to, DIV_RATIO);
        let poses = vec![from, &midpoint, to];

        let mut current = *starting;
        let mut prev_pose = from;
        for pose in poses {
            let mut success = false;

            // Not checked for collisions yet
            let solutions = self
                .robot
                .kinematics
                .inverse_continuing(&pose.pose, &current);

            // Solutions are already sorted best first
            for next in &solutions {
                // Internal "miniposes" generated through recursion are not checked for collision.
                // Outer calling code is reposible for check if 'current' is collision free
                if self.transitionable(&current, next) {
                    success = true;
                    current = *next;
                    break; // break out of solutions loop
                }
            }
            if !success {
                // Try recursive call. If succeeds, assume as done anyway.
                if depth >= self.linear_recursion_depth {
                    return Err(Transition {
                        from: *prev_pose,
                        to: *pose,
                        previous: current,
                        solutions: solutions,
                    });
                }

                // Try to bridge till them middle first, and then from the middle
                // This will result in shorter distance between from and to.
                let till_middle =
                    self.step_adaptive_linear_transition(starting, from, &midpoint, depth + 1)?;
                current =
                    self.step_adaptive_linear_transition(&till_middle, &midpoint, to, depth + 1)?;
            }
            prev_pose = pose;
        }
        Ok(current)
    }

    fn collides(&mut self, joints: &Joints) -> bool {
        // This gives rounding accuracty 0.01 degrees
        let rounded = joints.map(|x| (x.to_degrees() * 50.0).round() as i16);
        if let Some(collision) = self.checked.get(&rounded) {
            return *collision;
        }
        let collision = self.robot.collides(joints);
        self.checked.insert(rounded, collision);
        collision
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
        land: &Pose,
        steps: &Vec<Pose>,
        park: &Pose,
    ) -> Vec<AnnotatedPose> {
        let mut poses = Vec::with_capacity(10 * steps.len() + 2);

        // Add the landing pose
        poses.push(AnnotatedPose {
            pose: *land,
            flags: PoseFlags::LAND,
        });

        if !steps.is_empty() {
            // Add intermediate poses between land and the first step
            self.add_intermediate_poses(land, &steps[0], &mut poses);

            // Add the steps and intermediate poses between them
            for i in 0..steps.len() - 1 {
                poses.push(AnnotatedPose {
                    pose: steps[i].clone(),
                    flags: PoseFlags::TRACE,
                });

                self.add_intermediate_poses(&steps[i], &steps[i + 1], &mut poses);
            }

            // Add the last step
            let last = *steps.last().unwrap();
            poses.push(AnnotatedPose {
                pose: last.clone(),
                flags: PoseFlags::TRACE,
            });

            // Add intermediate poses between the last step and park
            self.add_intermediate_poses(&last, park, &mut poses);
        } else {
            // If no steps, add intermediate poses between land and park directly
            self.add_intermediate_poses(land, park, &mut poses);
        }

        // Add the parking pose
        poses.push(AnnotatedPose {
            pose: park.clone(),
            flags: PoseFlags::PARK,
        });

        poses
    }

    /// Add intermediate poses. start and end poses are not added.
    fn add_intermediate_poses(&self, start: &Pose, end: &Pose, poses: &mut Vec<AnnotatedPose>) {
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
                flags: PoseFlags::LINEAR_INTERPOLATED,
            });
        }
    }
}
