//! Cartesian stroke

use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::rrt::RRTPlanner;
use crate::utils::{dump_joints, transition_costs};
use bitflags::bitflags;
use rayon::prelude::{IntoParallelRefIterator, ParallelIterator};
use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Instant;

/// Reasonable default transition costs. Rotation of smaller joints is more tolerable.
/// The sum of all weights is 6.0
pub const DEFAULT_TRANSITION_COSTS: [f64; 6] = [1.2, 1.1, 1.1, 0.9, 0.9, 0.8];

/// Default number of Cartesian graph prefix states to try for RRT reconfiguration.
pub const DEFAULT_RECONFIGURATION_PREFIX_CANDIDATES: usize = 3;

/// Default number of Cartesian-feasible suffixes to try for onboarding RRT.
pub const DEFAULT_ONBOARDING_SUFFIX_CANDIDATES: usize = 3;

/// Default maximum number of states retained per Cartesian graph layer.
pub const DEFAULT_CARTESIAN_LAYER_STATES: usize = 32;

const JOINT_DEDUP_EPSILON_RAD: f64 = 1e-6;

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

    /// Maximum adaptive split depth for Cartesian transitions after the initial
    /// pose layers generated from [`Self::check_step_m`] and [`Self::check_step_rad`].
    pub linear_recursion_depth: usize,

    /// RRT planner that plans the onboarding part, and may potentially plan other
    /// parts that involve collision free relocation of the robot, but without
    /// Cartesian (linear) movement.
    pub rrt: RRTPlanner,

    /// If set, failed Cartesian stroke segments may be reconfigured through
    /// joint-space RRT movement and marked with [`PathFlags::RECONFIGURING`].
    /// If false, such segments fail the current strategy instead. Default is true
    /// in higher-level constructors.
    pub allow_reconfigure: bool,

    /// Maximum number of failed Cartesian graph prefix states to try as RRT
    /// reconfiguration starts, ordered by increasing prefix cost.
    /// A value of 0 uses [`DEFAULT_RECONFIGURATION_PREFIX_CANDIDATES`].
    pub max_reconfiguration_prefix_candidates: usize,

    /// Maximum number of Cartesian-feasible suffix candidates to try for
    /// onboarding RRT, ordered by suffix rank. A value of 0 uses
    /// [`DEFAULT_ONBOARDING_SUFFIX_CANDIDATES`].
    pub max_onboarding_suffix_candidates: usize,

    /// Maximum number of dynamic-programming states retained in each Cartesian
    /// graph layer. A value of 0 uses [`DEFAULT_CARTESIAN_LAYER_STATES`].
    pub max_cartesian_layer_states: usize,

    /// If set, linear interpolated poses are included in the output.
    /// Otherwise, they are discarded, many robots can do Cartesian stroke
    /// much better on their own. They are still checked internally; disable this
    /// only when the downstream robot executes retained poses as Cartesian moves.
    pub include_linear_interpolation: bool,

    /// Debug mode for logging
    pub debug: bool,
}

bitflags! {
    /// Flags that can be set on AnnotatedJoints in the output
    #[derive(Clone, Copy)]
    pub struct PathFlags: u32 {
        const NONE = 0b0000_0000;

        /// Position is part of the movement from the initil ("home") pose to the landing
        /// pose. It may include very arbitrary joint movements, so Cartesian stroke
        /// may not work on this part of the trajectory.
        const ONBOARDING =          1 << 1;

        /// Position directly matches one of the stroke poses given in the input.
        const TRACE =               1 << 2;

        /// Position is a linear interpolation between two poses of the trace. These poses
        /// are not needed for the robots that have built-in support for Cartesian stroke,
        /// but may be important for more developed models that only rotate between
        /// the given joint positions without guarantees that TCP movement is linear.
        const LIN_INTERP =          1 << 3;

        /// Position corresponds the starting pose ("land") that is normally little above
        /// the start of the required trace
        const LAND =                1 << 4;

        /// The tool is movind down from the landing position to the first stroke position.
        /// Activate the tool mechanism if any (start the drill, open sprayer, etc).
        const LANDING =             1 << 5;

        /// Position corresponds the ennding pose ("park") that is normally little above
        /// the end of the required trace. This is the last pose to include into the
        /// output.
        const PARK =                1 << 6;

        /// The tool is moving up from the last trace point to the parking position
        /// (shut down the effector mechanism if any)
        const PARKING =             1 << 7;

        /// Used with raster projector, indicates the movement considered "forwards"
        const FORWARDS =             1 << 8;

        /// Used with raster projector, indicates the movement considered "backwards"
        const BACKWARDS =            1 << 9;

        /// Reconfiguring joint movement used when Cartesian stroke cannot be followed directly.
        const RECONFIGURING =        1 << 10;

        /// Combined flag representing the "original" position, so the one that was
        /// given in the input.
        const ORIGINAL = Self::TRACE.bits() | Self::LAND.bits() | Self::PARK.bits();

        /// Special flag used in debugging to mark out anything of interest. Largest can be stored
        /// in u32
        const DEBUG = 1 << 31;
    }
}

#[derive(Clone, Copy)]
pub(crate) struct AnnotatedPose {
    pub(crate) pose: Pose,
    pub(crate) flags: PathFlags,
    pub(crate) split_depth: usize,
}

/// Movement type used to reach an [`AnnotatedJoints`] position from the previous output position.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MoveKind {
    Joint,
    Cartesian,
}

/// Annotated joints specifying the position flags and movement type into this position.
#[derive(Clone, Copy)]
pub struct AnnotatedJoints {
    pub joints: Joints,
    pub flags: PathFlags,
    pub move_into: MoveKind,
}

fn should_emit_output_state(include_linear_interpolation: bool, flags: PathFlags) -> bool {
    include_linear_interpolation || !flags.contains(PathFlags::LIN_INTERP)
}

fn reconfiguring_output_flags(flags: PathFlags) -> PathFlags {
    (flags & !PathFlags::LIN_INTERP) | PathFlags::RECONFIGURING
}

fn interpolation_flags_for_edge(from: PathFlags, to: PathFlags) -> PathFlags {
    let mut flags = PathFlags::LIN_INTERP;

    if from.intersects(PathFlags::LAND | PathFlags::LANDING)
        && to.intersects(PathFlags::TRACE | PathFlags::LANDING)
    {
        flags |= PathFlags::LANDING;
    }

    if from.intersects(PathFlags::TRACE | PathFlags::PARKING)
        && to.intersects(PathFlags::PARK | PathFlags::PARKING)
    {
        flags |= PathFlags::PARKING;
    }

    flags |= (from | to) & (PathFlags::FORWARDS | PathFlags::BACKWARDS);

    flags
}

impl AnnotatedPose {
    fn interpolate(&self, other: &AnnotatedPose, p: f64) -> AnnotatedPose {
        assert!((0.0..=1.0).contains(&p));

        let translation = self.pose.translation.lerp(other.pose.translation, p);
        let rotation = self.pose.rotation.slerp(other.pose.rotation, p);

        AnnotatedPose {
            pose: Pose::from_parts(translation, rotation),
            flags: interpolation_flags_for_edge(self.flags, other.flags),
            split_depth: self.split_depth.max(other.split_depth) + 1,
        }
    }
}

fn flag_representation(flags: &PathFlags) -> String {
    const FLAG_MAP: &[(PathFlags, &str)] = &[
        (PathFlags::LIN_INTERP, "LIN_INTERP"),
        (PathFlags::LAND, "LAND"),
        (PathFlags::LANDING, "LANDING"),
        (PathFlags::PARK, "PARK"),
        (PathFlags::PARKING, "PARKING"),
        (PathFlags::TRACE, "TRACE"),
        (PathFlags::RECONFIGURING, "RECONFIGURING"),
        (PathFlags::ONBOARDING, "ONBOARDING"),
        (PathFlags::FORWARDS, "FORWARDS"),
        (PathFlags::BACKWARDS, "BACKWARDS"),
        (PathFlags::DEBUG, "DEBUG"),
    ];

    let mut names = FLAG_MAP
        .iter()
        .filter(|(flag, _)| flags.contains(*flag))
        .map(|(_, name)| *name)
        .collect::<Vec<_>>();

    if flags.intersects(PathFlags::ORIGINAL) {
        names.push("ORIGINAL");
    }

    names.join(" | ")
}

impl fmt::Debug for AnnotatedPose {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let translation = self.pose.translation;
        let rotation = self.pose.rotation;
        write!(
            formatter,
            "{}: [{:.3}, {:.3}, {:.3}], quat {{ x: {:.3}, y: {:.3}, z: {:.3}, w: {:.3} }}",
            flag_representation(&self.flags),
            translation.x,
            translation.y,
            translation.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w
        )
    }
}

impl fmt::Debug for AnnotatedJoints {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{:?} into {}: {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2} ",
            self.move_into,
            flag_representation(&self.flags),
            self.joints[0].to_degrees(),
            self.joints[1].to_degrees(),
            self.joints[2].to_degrees(),
            self.joints[3].to_degrees(),
            self.joints[4].to_degrees(),
            self.joints[5].to_degrees(),
        )
    }
}

struct Transition {
    from: AnnotatedPose,
    to: AnnotatedPose,
    previous: Joints,
    solutions: Solutions,
}

#[derive(Clone, Copy)]
struct LayerState {
    joints: Joints,
    total_cost: f64,
    predecessor: Option<usize>,
}

struct CartesianGraphFailureCandidate {
    planned_prefix: Vec<Joints>,
    transition: Transition,
    prefix_cost: f64,
}

struct CartesianGraphFailure {
    candidates: Vec<CartesianGraphFailureCandidate>,
}

#[derive(Clone)]
struct PlanningOutcome {
    path: Vec<AnnotatedJoints>,
    rank: PlanRank,
}

impl PlanningOutcome {
    fn new(path: Vec<AnnotatedJoints>, transition_coefficients: &Joints) -> Self {
        let rank = PlanRank::from_path(&path, transition_coefficients);
        Self { path, rank }
    }

    fn is_good_enough(&self) -> bool {
        self.rank.is_good_enough()
    }
}

#[derive(Clone)]
struct SuffixPlanningOutcome {
    work_path_start: Joints,
    suffix: Vec<AnnotatedJoints>,
    rank: PlanRank,
}

impl SuffixPlanningOutcome {
    fn new(
        work_path_start: Joints,
        suffix: Vec<AnnotatedJoints>,
        transition_coefficients: &Joints,
    ) -> Self {
        let rank = PlanRank::from_path(&suffix, transition_coefficients);
        Self {
            work_path_start,
            suffix,
            rank,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct PlanRank {
    stroke_reconfigurations: usize,
    stroke_reconfiguration_steps: usize,
    total_transition_cost: f64,
    output_steps: usize,
}

impl PlanRank {
    fn from_path(path: &[AnnotatedJoints], transition_coefficients: &Joints) -> Self {
        let mut stroke_reconfigurations = 0;
        let mut stroke_reconfiguration_steps = 0;
        let mut was_stroke_reconfiguring = false;

        for step in path {
            let is_stroke_reconfiguring = is_stroke_interrupting_reconfiguration(step.flags);
            if is_stroke_reconfiguring {
                stroke_reconfiguration_steps += 1;
                if !was_stroke_reconfiguring {
                    stroke_reconfigurations += 1;
                }
            }
            was_stroke_reconfiguring = is_stroke_reconfiguring;
        }

        let total_transition_cost = path
            .windows(2)
            .map(|window| {
                transition_costs(
                    &window[0].joints,
                    &window[1].joints,
                    transition_coefficients,
                )
            })
            .sum();

        Self {
            stroke_reconfigurations,
            stroke_reconfiguration_steps,
            total_transition_cost,
            output_steps: path.len(),
        }
    }

    fn is_good_enough(&self) -> bool {
        self.stroke_reconfigurations == 0
    }

    fn is_better_than(&self, other: &Self) -> bool {
        self.stroke_reconfigurations
            .cmp(&other.stroke_reconfigurations)
            .then_with(|| {
                self.stroke_reconfiguration_steps
                    .cmp(&other.stroke_reconfiguration_steps)
            })
            .then_with(|| {
                self.total_transition_cost
                    .total_cmp(&other.total_transition_cost)
            })
            .then_with(|| self.output_steps.cmp(&other.output_steps))
            .is_lt()
    }
}

fn is_stroke_interrupting_reconfiguration(flags: PathFlags) -> bool {
    flags.contains(PathFlags::RECONFIGURING)
        && !flags.intersects(
            PathFlags::ONBOARDING | PathFlags::LANDING | PathFlags::PARKING | PathFlags::PARK,
        )
}

fn same_joints(left: &Joints, right: &Joints) -> bool {
    left.iter()
        .zip(right.iter())
        .all(|(left, right)| (left - right).abs() <= JOINT_DEDUP_EPSILON_RAD)
}

fn best_state_index(states: &[LayerState]) -> usize {
    states
        .iter()
        .enumerate()
        .min_by(|(_, left), (_, right)| left.total_cost.total_cmp(&right.total_cost))
        .map(|(index, _)| index)
        .expect("Layer should not be empty")
}

fn best_state_indices_by_cost(states: &[LayerState], limit: usize) -> Vec<usize> {
    let mut indices = (0..states.len()).collect::<Vec<_>>();
    indices.sort_by(|&left, &right| states[left].total_cost.total_cmp(&states[right].total_cost));
    let truncated_len = limit.min(indices.len());
    indices.truncate(truncated_len);
    indices
}

fn limit_layer_states_by_cost(states: &mut Vec<LayerState>, limit: usize) {
    states.sort_by(|left, right| left.total_cost.total_cmp(&right.total_cost));
    states.truncate(limit);
}

fn canceled_cartesian_graph_failure(
    starting: &Joints,
    from: &AnnotatedPose,
    target: &AnnotatedPose,
) -> CartesianGraphFailure {
    CartesianGraphFailure {
        candidates: vec![CartesianGraphFailureCandidate {
            planned_prefix: Vec::new(),
            transition: Transition {
                from: *from,
                to: *target,
                previous: *starting,
                solutions: Vec::new(),
            },
            prefix_cost: 0.0,
        }],
    }
}

fn sort_suffix_candidates_by_rank(candidates: &mut [SuffixPlanningOutcome]) {
    candidates.sort_by(|left, right| {
        if left.rank.is_better_than(&right.rank) {
            std::cmp::Ordering::Less
        } else if right.rank.is_better_than(&left.rank) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });
}

fn add_or_update_state(
    layer: &mut Vec<LayerState>,
    joints: Joints,
    total_cost: f64,
    predecessor: usize,
) {
    if let Some(existing) = layer
        .iter_mut()
        .find(|state| same_joints(&state.joints, &joints))
    {
        if total_cost < existing.total_cost {
            existing.total_cost = total_cost;
            existing.predecessor = Some(predecessor);
        }
    } else {
        layer.push(LayerState {
            joints,
            total_cost,
            predecessor: Some(predecessor),
        });
    }
}

fn reconstruct_path(
    layers: &[Vec<LayerState>],
    mut layer_index: usize,
    mut state_index: usize,
) -> Vec<Joints> {
    let mut path = Vec::with_capacity(layer_index);
    while layer_index > 0 {
        let state = layers[layer_index][state_index];
        path.push(state.joints);
        state_index = state
            .predecessor
            .expect("Non-start layer should have predecessor");
        layer_index -= 1;
    }
    path.reverse();
    path
}

impl Cartesian<'_> {
    fn should_emit_output_state(&self, flags: PathFlags) -> bool {
        should_emit_output_state(self.include_linear_interpolation, flags)
    }

    fn reconfiguration_prefix_candidate_limit(&self) -> usize {
        if self.max_reconfiguration_prefix_candidates == 0 {
            DEFAULT_RECONFIGURATION_PREFIX_CANDIDATES
        } else {
            self.max_reconfiguration_prefix_candidates
        }
    }

    fn onboarding_suffix_candidate_limit(&self) -> usize {
        if self.max_onboarding_suffix_candidates == 0 {
            DEFAULT_ONBOARDING_SUFFIX_CANDIDATES
        } else {
            self.max_onboarding_suffix_candidates
        }
    }

    fn cartesian_layer_state_limit(&self) -> usize {
        if self.max_cartesian_layer_states == 0 {
            DEFAULT_CARTESIAN_LAYER_STATES
        } else {
            self.max_cartesian_layer_states
        }
    }

    /// Path plan for the given vector of poses. The returned path must be transitionable
    /// and collision free.
    pub fn plan(
        &self,
        from: &Joints,
        land: &Pose,
        steps: Vec<Pose>,
        park: &Pose,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        if self.robot.collides(from) {
            return Err("Onboarding point collides".into());
        }
        let strategies = self.robot.inverse_continuing(land, from);
        if strategies.is_empty() {
            return Err("Unable to start from onboarding point".into());
        }
        let poses = self.with_intermediate_poses(land, &steps, park);
        println!("Probing {} strategies", strategies.len());

        let stop = Arc::new(AtomicBool::new(false));

        let mut suffix_candidates = strategies
            .par_iter()
            .filter_map(|strategy| {
                if stop.load(Ordering::Relaxed) {
                    return None;
                }

                match self.probe_cartesian_suffix(strategy, &poses, &stop) {
                    Ok(suffix) => {
                        let outcome = SuffixPlanningOutcome::new(
                            *strategy,
                            suffix,
                            &self.transition_coefficients,
                        );
                        println!(
                            "Cartesian suffix worked out: {:?}, rank {:?}",
                            strategy, outcome.rank
                        );
                        Some(outcome)
                    }
                    Err(msg) => {
                        if self.debug {
                            println!("Cartesian suffix failed: {:?}, {}", strategy, msg);
                        }
                        None
                    }
                }
            })
            .collect::<Vec<_>>();

        if suffix_candidates.is_empty() {
            return Err(format!(
                "No Cartesian suffix worked out of {} strategies",
                strategies.len()
            ));
        }

        sort_suffix_candidates_by_rank(&mut suffix_candidates);

        let onboarding_limit = self
            .onboarding_suffix_candidate_limit()
            .min(suffix_candidates.len());
        if self.debug {
            println!(
                "Trying onboarding RRT for {} of {} Cartesian-feasible suffixes",
                onboarding_limit,
                suffix_candidates.len()
            );
        }

        let mut best_fallback = None::<PlanningOutcome>;
        for suffix_candidate in suffix_candidates.into_iter().take(onboarding_limit) {
            let work_path_start = suffix_candidate.work_path_start;
            match self.attach_onboarding(from, suffix_candidate, &stop) {
                Ok(path) => {
                    let outcome = PlanningOutcome::new(path, &self.transition_coefficients);
                    println!(
                        "Strategy worked out after onboarding: {:?}, rank {:?}",
                        work_path_start, outcome.rank
                    );

                    if outcome.is_good_enough() {
                        return Ok(outcome.path);
                    }

                    if best_fallback
                        .as_ref()
                        .map_or(true, |current| outcome.rank.is_better_than(&current.rank))
                    {
                        best_fallback = Some(outcome);
                    }
                }
                Err(msg) => {
                    if self.debug {
                        println!("Onboarding failed: {}", msg);
                    }
                }
            }
        }

        if let Some(outcome) = best_fallback {
            Ok(outcome.path)
        } else {
            Err(format!(
                "No onboarding RRT worked out for {} Cartesian-feasible suffixes tried",
                onboarding_limit
            ))
        }
    }

    fn attach_onboarding(
        &self,
        onboarding_start: &Joints,
        suffix_candidate: SuffixPlanningOutcome,
        stop: &AtomicBool,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        let mut trace = Vec::with_capacity(100 + suffix_candidate.suffix.len());
        self.append_onboarding(
            onboarding_start,
            &suffix_candidate.work_path_start,
            stop,
            &mut trace,
        )?;
        trace.extend(suffix_candidate.suffix);
        Ok(trace)
    }

    /// Probe the Cartesian suffix for the given landing strategy before onboarding RRT.
    fn probe_cartesian_suffix(
        &self,
        work_path_start: &Joints,
        poses: &[AnnotatedPose],
        stop: &AtomicBool,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        println!("Cartesian suffix planning started, computing strategy {work_path_start:?}");

        let started = Instant::now();
        let mut poses = poses.to_vec();
        let mut trace = Vec::with_capacity(100 + poses.len() + 10);
        let mut previous_joints = *work_path_start;

        let mut step = 1;
        let mut pose_index = 1;

        while pose_index < poses.len() {
            match self.plan_cartesian_graph(
                &previous_joints,
                &poses[pose_index - 1],
                &poses[pose_index..],
                stop,
            ) {
                Ok(extension) => {
                    self.append_cartesian_extension(
                        &extension,
                        &poses[pose_index..pose_index + extension.len()],
                        &mut trace,
                        &mut previous_joints,
                        &mut step,
                    );
                    break;
                }
                Err(failure) => {
                    if stop.load(Ordering::Relaxed) {
                        return Err("Stopped".into());
                    }

                    let first_candidate = failure
                        .candidates
                        .first()
                        .expect("Cartesian graph failure should include candidates");

                    let prefix_len = first_candidate.planned_prefix.len();
                    let failed_pose_index = pose_index + prefix_len;
                    if self.refine_transition(&mut poses, failed_pose_index) {
                        self.append_cartesian_extension(
                            &first_candidate.planned_prefix,
                            &poses[pose_index..failed_pose_index],
                            &mut trace,
                            &mut previous_joints,
                            &mut step,
                        );
                        pose_index = failed_pose_index;
                        continue;
                    }

                    let failed_pose = &poses[failed_pose_index];

                    if !self.allow_reconfigure {
                        if self.debug {
                            println!("Reconfiguration disabled at step {:?}", step);
                        }
                        self.log_failed_transition(&failure, step);
                        return Err(format!("Failed to transition at step {}", step));
                    }

                    if !self.append_reconfiguration_candidates(
                        &failure.candidates,
                        &poses[pose_index..failed_pose_index],
                        failed_pose,
                        stop,
                        &mut trace,
                        &mut previous_joints,
                        &mut step,
                    ) {
                        self.log_failed_transition(&failure, step);
                        return Err(format!("Failed to transition at step {}", step));
                    }

                    previous_joints = trace
                        .last()
                        .expect("Reconfiguration should append a state")
                        .joints;
                    if failed_pose.flags.contains(PathFlags::TRACE) {
                        step += 1;
                    }
                    pose_index = failed_pose_index + 1;
                }
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

    fn refine_transition(&self, poses: &mut Vec<AnnotatedPose>, to_index: usize) -> bool {
        if to_index == 0 {
            return false;
        }

        let from = poses[to_index - 1];
        let to = poses[to_index];
        if from.split_depth.max(to.split_depth) >= self.linear_recursion_depth {
            return false;
        }

        poses.insert(to_index, from.interpolate(&to, 0.5));
        true
    }

    fn append_cartesian_extension(
        &self,
        extension: &[Joints],
        poses: &[AnnotatedPose],
        trace: &mut Vec<AnnotatedJoints>,
        previous_joints: &mut Joints,
        step: &mut i32,
    ) {
        debug_assert_eq!(
            extension.len(),
            poses.len(),
            "Cartesian extension and pose slice lengths must match"
        );

        for (joints, pose) in extension.iter().zip(poses) {
            if self.should_emit_output_state(pose.flags) {
                trace.push(AnnotatedJoints {
                    joints: *joints,
                    flags: pose.flags,
                    move_into: MoveKind::Cartesian,
                });
            }
            *previous_joints = *joints;
            if pose.flags.contains(PathFlags::TRACE) {
                *step += 1;
            }
        }
    }

    fn append_reconfiguration_candidates(
        &self,
        candidates: &[CartesianGraphFailureCandidate],
        prefix_poses: &[AnnotatedPose],
        failed_pose: &AnnotatedPose,
        stop: &AtomicBool,
        trace: &mut Vec<AnnotatedJoints>,
        previous_joints: &mut Joints,
        step: &mut i32,
    ) -> bool {
        let base_trace_len = trace.len();
        let base_previous_joints = *previous_joints;
        let base_step = *step;

        for (candidate_index, candidate) in candidates.iter().enumerate() {
            trace.truncate(base_trace_len);
            *previous_joints = base_previous_joints;
            *step = base_step;

            if self.debug {
                println!(
                    "Trying reconfiguration prefix candidate {} with cost {:.6}",
                    candidate_index + 1,
                    candidate.prefix_cost
                );
            }

            self.append_cartesian_extension(
                &candidate.planned_prefix,
                prefix_poses,
                trace,
                previous_joints,
                step,
            );

            debug_assert!(
                same_joints(previous_joints, &candidate.transition.previous),
                "Reconfiguration candidate prefix should end at its transition start"
            );

            if self.append_reconfiguration(
                previous_joints,
                failed_pose,
                &candidate.transition,
                stop,
                trace,
            ) {
                return true;
            }
        }

        trace.truncate(base_trace_len);
        *previous_joints = base_previous_joints;
        *step = base_step;
        false
    }

    fn append_reconfiguration(
        &self,
        previous_joints: &Joints,
        pose: &AnnotatedPose,
        transition: &Transition,
        stop: &AtomicBool,
        trace: &mut Vec<AnnotatedJoints>,
    ) -> bool {
        println!("Closing step with RRT");
        for next in &transition.solutions {
            let path = self.rrt.plan_rrt(previous_joints, next, self.robot, stop);
            if let Ok(path) = path {
                println!("  ... closed with RRT {} steps", path.len());
                for joints in path {
                    let flags = reconfiguring_output_flags(pose.flags);
                    if self.should_emit_output_state(flags) {
                        trace.push(AnnotatedJoints {
                            joints,
                            flags,
                            move_into: MoveKind::Joint,
                        });
                    }
                }
                return true;
            }
        }
        false
    }

    fn append_onboarding(
        &self,
        start: &Joints,
        land: &Joints,
        stop: &AtomicBool,
        trace: &mut Vec<AnnotatedJoints>,
    ) -> Result<(), String> {
        if same_joints(start, land) {
            trace.push(AnnotatedJoints {
                joints: *start,
                flags: PathFlags::LAND,
                move_into: MoveKind::Joint,
            });
            return Ok(());
        }

        let onboarding_path = self.rrt.plan_rrt(start, land, self.robot, stop)?;
        let last_index = onboarding_path.len().saturating_sub(1);
        for (index, joints) in onboarding_path.into_iter().enumerate() {
            let flags = if index == last_index {
                PathFlags::ONBOARDING | PathFlags::LAND
            } else {
                PathFlags::ONBOARDING
            };
            trace.push(AnnotatedJoints {
                joints,
                flags,
                move_into: MoveKind::Joint,
            });
        }
        Ok(())
    }

    /// Plan a Cartesian path through all provided pose layers using dynamic programming.
    /// The returned path excludes `starting` and contains one joint state per target pose.
    fn plan_cartesian_graph(
        &self,
        starting: &Joints,
        from: &AnnotatedPose,
        targets: &[AnnotatedPose],
        stop: &AtomicBool,
    ) -> Result<Vec<Joints>, CartesianGraphFailure> {
        if targets.is_empty() {
            return Ok(Vec::new());
        }

        let mut layers = vec![vec![LayerState {
            joints: *starting,
            total_cost: 0.0,
            predecessor: None,
        }]];

        for (target_index, target) in targets.iter().enumerate() {
            if stop.load(Ordering::Relaxed) {
                return Err(canceled_cartesian_graph_failure(starting, from, target));
            }

            let previous_layer = layers.last().expect("Start layer should be available");
            let mut next_layer = Vec::new();

            for (previous_index, previous) in previous_layer.iter().enumerate() {
                if stop.load(Ordering::Relaxed) {
                    return Err(canceled_cartesian_graph_failure(starting, from, target));
                }

                let solutions = self
                    .robot
                    .inverse_continuing(&target.pose, &previous.joints);
                for candidate in &solutions {
                    let edge_cost = transition_costs(
                        &previous.joints,
                        candidate,
                        &self.transition_coefficients,
                    );
                    if edge_cost <= self.max_transition_cost {
                        add_or_update_state(
                            &mut next_layer,
                            *candidate,
                            previous.total_cost + edge_cost,
                            previous_index,
                        );
                    }
                }
            }

            if next_layer.is_empty() {
                let previous_layer_index = layers.len() - 1;
                let from_pose = if target_index == 0 {
                    *from
                } else {
                    targets[target_index - 1]
                };
                let previous_layer = &layers[previous_layer_index];
                let candidate_limit = self.reconfiguration_prefix_candidate_limit();
                let candidates = best_state_indices_by_cost(previous_layer, candidate_limit)
                    .into_iter()
                    .map(|previous_state_index| {
                        let previous_state = previous_layer[previous_state_index];
                        let planned_prefix =
                            reconstruct_path(&layers, previous_layer_index, previous_state_index);
                        let previous = previous_state.joints;
                        let solutions = self.robot.inverse_continuing(&target.pose, &previous);

                        CartesianGraphFailureCandidate {
                            planned_prefix,
                            transition: Transition {
                                from: from_pose,
                                to: *target,
                                previous,
                                solutions,
                            },
                            prefix_cost: previous_state.total_cost,
                        }
                    })
                    .collect();

                return Err(CartesianGraphFailure { candidates });
            }

            limit_layer_states_by_cost(&mut next_layer, self.cartesian_layer_state_limit());
            layers.push(next_layer);
        }

        let final_layer_index = layers.len() - 1;
        let final_state_index = best_state_index(&layers[final_layer_index]);
        Ok(reconstruct_path(
            &layers,
            final_layer_index,
            final_state_index,
        ))
    }

    fn log_failed_transition(&self, failure: &CartesianGraphFailure, step: i32) {
        if !self.debug {
            return;
        }
        println!("Step {} from [1..n] failed", step);
        println!(
            "No transition with weighted cost below {:.6}:",
            self.max_transition_cost
        );

        for (candidate_index, candidate) in failure.candidates.iter().enumerate() {
            let transition = &candidate.transition;
            println!(
                "   Candidate {} prefix cost {:.6}",
                candidate_index + 1,
                candidate.prefix_cost
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
                    "   transition weighted cost {:.6} collides: {}",
                    transition_costs(&transition.previous, s, &self.transition_coefficients),
                    self.robot.collides(s)
                );
            }
        }
    }

    fn with_intermediate_poses(
        &self,
        land: &Pose,
        steps: &[Pose],
        park: &Pose,
    ) -> Vec<AnnotatedPose> {
        let mut poses = Vec::with_capacity(10 * steps.len() + 2);

        // Add the landing pose
        poses.push(AnnotatedPose {
            pose: *land,
            flags: PathFlags::LAND,
            split_depth: 0,
        });

        if !steps.is_empty() {
            // Add intermediate poses between land and the first step
            self.add_intermediate_poses(land, &steps[0], PathFlags::LANDING, &mut poses);

            // Add the steps and intermediate poses between them
            for i in 0..steps.len() - 1 {
                poses.push(AnnotatedPose {
                    pose: steps[i],
                    flags: PathFlags::TRACE,
                    split_depth: 0,
                });

                self.add_intermediate_poses(&steps[i], &steps[i + 1], PathFlags::NONE, &mut poses);
            }

            // Add the last step
            let last = *steps.last().unwrap();
            poses.push(AnnotatedPose {
                pose: last,
                flags: PathFlags::TRACE,
                split_depth: 0,
            });

            // Add intermediate poses between the last step and park
            self.add_intermediate_poses(&last, park, PathFlags::PARKING, &mut poses);
        } else {
            // If no steps, add intermediate poses between land and park directly
            self.add_intermediate_poses(land, park, PathFlags::PARKING, &mut poses);
        }

        // Add the parking pose
        poses.push(AnnotatedPose {
            pose: *park,
            flags: PathFlags::PARK,
            split_depth: 0,
        });

        poses
    }

    /// Add intermediate poses. start and end poses are not added.
    fn add_intermediate_poses(
        &self,
        start: &Pose,
        end: &Pose,
        flags: PathFlags,
        poses: &mut Vec<AnnotatedPose>,
    ) {
        // Calculate the translation difference and distance
        let translation_diff = end.translation - start.translation;
        let translation_distance = translation_diff.length();

        // Calculate the rotation difference and angle
        let rotation_diff = end.rotation * start.rotation.inverse();
        let rotation_angle = rotation_diff.to_scaled_axis().length();

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
            let intermediate_translation = start.translation + translation_step * i as f64;
            let intermediate_rotation = start.rotation.slerp(end.rotation, fraction);

            // Construct the intermediate pose
            let intermediate_pose =
                Pose::from_parts(intermediate_translation, intermediate_rotation);

            poses.push(AnnotatedPose {
                pose: intermediate_pose,
                flags: PathFlags::LIN_INTERP | flags,
                split_depth: 0,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        add_or_update_state, best_state_indices_by_cost, canceled_cartesian_graph_failure,
        flag_representation, interpolation_flags_for_edge, limit_layer_states_by_cost,
        reconfiguring_output_flags, should_emit_output_state, sort_suffix_candidates_by_rank,
        AnnotatedJoints, AnnotatedPose, LayerState, MoveKind, PathFlags, PlanRank,
        SuffixPlanningOutcome, DEFAULT_TRANSITION_COSTS,
    };
    use crate::kinematic_traits::Pose;

    #[test]
    fn output_filter_honors_linear_interpolation_flag() {
        assert!(!should_emit_output_state(false, PathFlags::LIN_INTERP));
        assert!(!should_emit_output_state(
            false,
            PathFlags::LIN_INTERP | PathFlags::TRACE
        ));
        assert!(should_emit_output_state(false, PathFlags::TRACE));
        assert!(should_emit_output_state(true, PathFlags::LIN_INTERP));
    }

    #[test]
    fn flag_representation_includes_direction_and_diagnostic_flags() {
        let representation = flag_representation(
            &(PathFlags::TRACE | PathFlags::FORWARDS | PathFlags::BACKWARDS | PathFlags::DEBUG),
        );

        assert!(representation.contains("TRACE"));
        assert!(representation.contains("FORWARDS"));
        assert!(representation.contains("BACKWARDS"));
        assert!(representation.contains("DEBUG"));
        assert!(representation.contains("ORIGINAL"));
    }

    #[test]
    fn movement_type_is_independent_from_node_flags() {
        let landing_from_joint = AnnotatedJoints {
            joints: [0.0; 6],
            flags: PathFlags::ONBOARDING | PathFlags::LAND,
            move_into: MoveKind::Joint,
        };
        let trace_from_cartesian = AnnotatedJoints {
            joints: [0.0; 6],
            flags: PathFlags::TRACE,
            move_into: MoveKind::Cartesian,
        };

        assert!(landing_from_joint.flags.contains(PathFlags::LAND));
        assert_eq!(landing_from_joint.move_into, MoveKind::Joint);
        assert!(trace_from_cartesian.flags.contains(PathFlags::TRACE));
        assert_eq!(trace_from_cartesian.move_into, MoveKind::Cartesian);
    }

    #[test]
    fn reconfiguring_output_flags_mark_rrt_fallback_steps() {
        let interpolated_fallback = reconfiguring_output_flags(PathFlags::LIN_INTERP);
        assert!(interpolated_fallback.contains(PathFlags::RECONFIGURING));
        assert!(!interpolated_fallback.contains(PathFlags::LIN_INTERP));
        assert!(should_emit_output_state(false, interpolated_fallback));

        let trace_fallback = reconfiguring_output_flags(PathFlags::TRACE);
        assert!(trace_fallback.contains(PathFlags::TRACE));
        assert!(trace_fallback.contains(PathFlags::RECONFIGURING));
    }

    #[test]
    fn interpolation_flags_preserve_edge_semantics() {
        let landing = interpolation_flags_for_edge(
            PathFlags::LAND | PathFlags::FORWARDS,
            PathFlags::TRACE | PathFlags::BACKWARDS,
        );

        assert!(landing.contains(PathFlags::LIN_INTERP));
        assert!(landing.contains(PathFlags::LANDING));
        assert!(landing.contains(PathFlags::FORWARDS));
        assert!(landing.contains(PathFlags::BACKWARDS));
        assert!(!landing.contains(PathFlags::LAND));

        let parking = interpolation_flags_for_edge(
            PathFlags::TRACE | PathFlags::FORWARDS,
            PathFlags::PARK | PathFlags::BACKWARDS,
        );

        assert!(parking.contains(PathFlags::LIN_INTERP));
        assert!(parking.contains(PathFlags::PARKING));
        assert!(parking.contains(PathFlags::FORWARDS));
        assert!(parking.contains(PathFlags::BACKWARDS));
        assert!(!parking.contains(PathFlags::TRACE));
        assert!(!parking.contains(PathFlags::PARK));

        let landing_reconfiguration = reconfiguring_output_flags(landing);
        assert!(landing_reconfiguration.contains(PathFlags::LANDING));
        assert!(landing_reconfiguration.contains(PathFlags::RECONFIGURING));
        assert!(!landing_reconfiguration.contains(PathFlags::LIN_INTERP));

        let parking_reconfiguration = reconfiguring_output_flags(parking);
        assert!(parking_reconfiguration.contains(PathFlags::PARKING));
        assert!(parking_reconfiguration.contains(PathFlags::RECONFIGURING));
        assert!(!parking_reconfiguration.contains(PathFlags::LIN_INTERP));
    }

    #[test]
    fn plan_rank_treats_stroke_reconfiguration_as_not_good_enough() {
        let uninterrupted = vec![
            joint_step(0.0, PathFlags::ONBOARDING, MoveKind::Joint),
            joint_step(1.0, PathFlags::TRACE, MoveKind::Cartesian),
            joint_step(
                2.0,
                PathFlags::PARK | PathFlags::RECONFIGURING,
                MoveKind::Joint,
            ),
        ];
        let interrupted = vec![
            joint_step(0.0, PathFlags::ONBOARDING, MoveKind::Joint),
            joint_step(
                1.0,
                PathFlags::TRACE | PathFlags::RECONFIGURING,
                MoveKind::Joint,
            ),
            joint_step(2.0, PathFlags::TRACE, MoveKind::Cartesian),
        ];

        let uninterrupted_rank = PlanRank::from_path(&uninterrupted, &DEFAULT_TRANSITION_COSTS);
        let interrupted_rank = PlanRank::from_path(&interrupted, &DEFAULT_TRANSITION_COSTS);

        assert!(uninterrupted_rank.is_good_enough());
        assert!(!interrupted_rank.is_good_enough());
        assert!(uninterrupted_rank.is_better_than(&interrupted_rank));
    }

    #[test]
    fn plan_rank_ignores_landing_and_parking_reconfiguration() {
        let path = vec![
            joint_step(0.0, PathFlags::ONBOARDING, MoveKind::Joint),
            joint_step(
                1.0,
                PathFlags::LANDING | PathFlags::RECONFIGURING,
                MoveKind::Joint,
            ),
            joint_step(
                2.0,
                PathFlags::PARKING | PathFlags::RECONFIGURING,
                MoveKind::Joint,
            ),
            joint_step(
                3.0,
                PathFlags::PARK | PathFlags::RECONFIGURING,
                MoveKind::Joint,
            ),
        ];

        let rank = PlanRank::from_path(&path, &DEFAULT_TRANSITION_COSTS);

        assert!(rank.is_good_enough());
        assert_eq!(rank.stroke_reconfigurations, 0);
        assert_eq!(rank.stroke_reconfiguration_steps, 0);
    }

    #[test]
    fn dynamic_programming_layer_keeps_lowest_cost_state() {
        let mut layer = Vec::new();
        add_or_update_state(&mut layer, [1.0; 6], 10.0, 0);
        add_or_update_state(&mut layer, [1.0; 6], 5.0, 2);

        assert_eq!(layer.len(), 1);
        assert_eq!(layer[0].total_cost, 5.0);
        assert_eq!(layer[0].predecessor, Some(2));
    }

    #[test]
    fn dynamic_programming_layer_deduplicates_nearby_joint_states() {
        let mut layer = Vec::new();
        add_or_update_state(&mut layer, [1.0; 6], 10.0, 0);
        add_or_update_state(&mut layer, [1.0 + 0.5e-6; 6], 5.0, 2);

        assert_eq!(layer.len(), 1);
        assert_eq!(layer[0].total_cost, 5.0);
        assert_eq!(layer[0].predecessor, Some(2));
    }

    #[test]
    fn best_state_indices_by_cost_returns_limited_sorted_prefixes() {
        let states = vec![
            layer_state(0.0, 30.0),
            layer_state(1.0, 10.0),
            layer_state(2.0, 20.0),
        ];

        assert_eq!(best_state_indices_by_cost(&states, 2), vec![1, 2]);
        assert_eq!(best_state_indices_by_cost(&states, 10), vec![1, 2, 0]);
        assert!(best_state_indices_by_cost(&states, 0).is_empty());
    }

    #[test]
    fn layer_states_are_limited_by_cost() {
        let mut states = vec![
            layer_state(0.0, 30.0),
            layer_state(1.0, 10.0),
            layer_state(2.0, 20.0),
        ];

        limit_layer_states_by_cost(&mut states, 2);

        assert_eq!(states.len(), 2);
        assert_eq!(states[0].joints, [1.0; 6]);
        assert_eq!(states[1].joints, [2.0; 6]);
    }

    #[test]
    fn canceled_graph_failure_uses_empty_prefix_and_start_state() {
        let starting = [1.0; 6];
        let from = annotated_pose(PathFlags::LAND);
        let target = annotated_pose(PathFlags::TRACE);

        let failure = canceled_cartesian_graph_failure(&starting, &from, &target);

        assert_eq!(failure.candidates.len(), 1);
        let candidate = &failure.candidates[0];
        assert!(candidate.planned_prefix.is_empty());
        assert_eq!(candidate.prefix_cost, 0.0);
        assert_eq!(candidate.transition.previous, starting);
        assert!(candidate.transition.from.flags.contains(PathFlags::LAND));
        assert!(candidate.transition.to.flags.contains(PathFlags::TRACE));
        assert!(candidate.transition.solutions.is_empty());
    }

    #[test]
    fn suffix_candidates_are_sorted_by_rank() {
        let mut candidates = vec![
            suffix_candidate(
                2.0,
                vec![joint_step(
                    0.0,
                    PathFlags::TRACE | PathFlags::RECONFIGURING,
                    MoveKind::Joint,
                )],
            ),
            suffix_candidate(
                1.0,
                vec![joint_step(0.0, PathFlags::TRACE, MoveKind::Cartesian)],
            ),
        ];

        sort_suffix_candidates_by_rank(&mut candidates);

        assert_eq!(candidates[0].work_path_start, [1.0; 6]);
        assert_eq!(candidates[1].work_path_start, [2.0; 6]);
    }

    fn layer_state(value: f64, total_cost: f64) -> LayerState {
        LayerState {
            joints: [value; 6],
            total_cost,
            predecessor: None,
        }
    }

    fn suffix_candidate(value: f64, suffix: Vec<AnnotatedJoints>) -> SuffixPlanningOutcome {
        SuffixPlanningOutcome::new([value; 6], suffix, &DEFAULT_TRANSITION_COSTS)
    }

    fn annotated_pose(flags: PathFlags) -> AnnotatedPose {
        AnnotatedPose {
            pose: Pose::identity(),
            flags,
            split_depth: 0,
        }
    }

    fn joint_step(value: f64, flags: PathFlags, move_into: MoveKind) -> AnnotatedJoints {
        AnnotatedJoints {
            joints: [value; 6],
            flags,
            move_into,
        }
    }
}
