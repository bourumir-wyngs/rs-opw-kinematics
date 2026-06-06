//! Cartesian stroke path planning.
//!
//! This module plans a collision-free path that enters a Cartesian stroke,
//! follows the requested TCP poses, optionally bridges infeasible stroke
//! segments with joint-space RRT reconfiguration, and exits at the park pose.

use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::rrt::RRTPlanner;
use crate::utils::{dump_joints, transition_costs};
use bitflags::bitflags;
use rayon::prelude::{IndexedParallelIterator, IntoParallelRefIterator, ParallelIterator};
use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
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

/// Default number of Cartesian suffix solutions to await before cancelling extra probes.
pub const DEFAULT_MAX_SOLUTIONS_AWAIT: usize = 3;

/// Joint-space tolerance used to merge numerically equivalent IK states in DP layers.
const JOINT_DEDUP_EPSILON_RAD: f64 = 1e-6;

/// Configurable Cartesian stroke planner for a robot with collision geometry.
pub struct Cartesian<'a> {
    /// Robot model used for inverse kinematics and collision checks.
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
    /// graph layer during the fast pass. A value of 0 uses
    /// [`DEFAULT_CARTESIAN_LAYER_STATES`]. If the fast pass cannot produce an
    /// acceptable complete path, planning retries without this beam limit before
    /// returning failure or a fallback path.
    /// A value of [`usize::MAX`] disables this beam limit.
    pub max_cartesian_layer_states: usize,

    /// Maximum number of Cartesian suffix solutions to collect in the fast pass before
    /// starting later strategy batches. If capped candidates cannot produce an acceptable
    /// complete path, planning retries without this suffix cap.
    /// A value of 0 uses [`DEFAULT_MAX_SOLUTIONS_AWAIT`].
    pub max_solutions_await: usize,

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

/// Input pose plus semantic flags used while refining and planning the stroke.
#[derive(Clone, Copy)]
pub(crate) struct AnnotatedPose {
    /// TCP pose to reach.
    pub(crate) pose: Pose,

    /// Semantic role of the pose in the path.
    pub(crate) flags: PathFlags,

    /// Adaptive refinement depth used to avoid endlessly splitting a failed edge.
    pub(crate) split_depth: usize,
}

/// Movement type used to reach an [`AnnotatedJoints`] position from the previous output position.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MoveKind {
    /// Joint-space motion is used to enter this state.
    Joint,

    /// Cartesian motion is expected to enter this state.
    Cartesian,
}

/// Annotated joints specifying the position flags and movement type into this position.
#[derive(Clone, Copy)]
pub struct AnnotatedJoints {
    /// Joint angles for this output waypoint.
    pub joints: Joints,

    /// Semantic role of this waypoint in the output path.
    pub flags: PathFlags,

    /// Motion mode expected between the previous output waypoint and this waypoint.
    pub move_into: MoveKind,
}

/// Returns whether an internal state should be included in the public output path.
fn should_emit_output_state(include_linear_interpolation: bool, flags: PathFlags) -> bool {
    include_linear_interpolation || !flags.contains(PathFlags::LIN_INTERP)
}

/// Converts the flags of a failed Cartesian pose into the flags of a joint-space fallback step.
fn reconfiguring_output_flags(flags: PathFlags) -> PathFlags {
    (flags & !PathFlags::LIN_INTERP) | PathFlags::RECONFIGURING
}

/// Builds semantic flags for an interpolated pose inserted between two existing poses.
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
    /// Interpolates pose geometry and carries edge-level semantic flags to the midpoint.
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

/// Produces a compact textual representation of flags for diagnostics.
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

    if flags.bits() == PathFlags::ORIGINAL.bits() {
        names.push("ORIGINAL");
    }

    names.join(" | ")
}

impl fmt::Debug for AnnotatedPose {
    /// Formats an annotated pose with semantic flags and TCP pose components.
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
    /// Formats an output waypoint with its motion mode, semantic flags, and joint angles.
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

/// Failed Cartesian edge together with candidate IK solutions for the target pose.
struct Transition {
    /// Pose that the failed Cartesian edge starts from.
    from: AnnotatedPose,

    /// Pose that could not be reached within the allowed transition cost.
    to: AnnotatedPose,

    /// Joint state at the start of the failed edge.
    previous: Joints,

    /// IK solutions for `to` from `previous`, including candidates that may need RRT.
    solutions: Solutions,
}

/// Dynamic-programming state for one Cartesian target layer.
#[derive(Clone, Copy)]
struct LayerState {
    /// Joint state selected for this layer.
    joints: Joints,

    /// Accumulated weighted transition cost from the graph start to this state.
    total_cost: f64,

    /// Index of the preceding state in the previous layer.
    predecessor: Option<usize>,
}

/// One prefix state from which a failed Cartesian edge may be reconfigured by RRT.
struct CartesianGraphFailureCandidate {
    /// Cartesian joint prefix that reaches `transition.previous` from the graph start.
    planned_prefix: Vec<Joints>,

    /// Failed edge starting from the prefix end state.
    transition: Transition,

    /// Accumulated cost of `planned_prefix`.
    prefix_cost: f64,
}

/// Cartesian graph failure containing the best prefix candidates to try next.
struct CartesianGraphFailure {
    /// Candidate failed edges sorted by increasing prefix cost.
    candidates: Vec<CartesianGraphFailureCandidate>,
}

/// Mutable cursors used while appending a reconfiguration bridge.
struct ReconfigurationAppendState<'a> {
    stop: &'a AtomicBool,
    trace: &'a mut Vec<AnnotatedJoints>,
    previous_joints: &'a mut Joints,
    step: &'a mut i32,
}

/// Complete planning attempt plus the rank used to compare fallbacks.
#[derive(Clone)]
struct PlanningOutcome {
    /// Output path for this attempt.
    path: Vec<AnnotatedJoints>,

    /// Ranking summary used to prefer fewer stroke interruptions and lower cost.
    rank: PlanRank,
}

impl PlanningOutcome {
    /// Builds a planning outcome and computes its rank from the generated path.
    fn new(path: Vec<AnnotatedJoints>, transition_coefficients: &Joints) -> Self {
        let rank = PlanRank::from_path(&path, transition_coefficients);
        Self { path, rank }
    }

    /// Returns true when the path has no reconfiguration that interrupts the stroke.
    fn is_good_enough(&self) -> bool {
        self.rank.is_good_enough()
    }
}

/// Cartesian suffix that has been proven feasible before onboarding RRT is attempted.
#[derive(Clone)]
struct SuffixPlanningOutcome {
    /// Landing joint state from which the Cartesian suffix starts.
    work_path_start: Joints,

    /// Cartesian suffix output waypoints, excluding onboarding.
    suffix: Vec<AnnotatedJoints>,

    /// Rank of the suffix path used to choose which onboarding attempts are worth trying.
    rank: PlanRank,
}

impl SuffixPlanningOutcome {
    /// Builds a suffix outcome and ranks it from the selected landing state.
    fn new(
        work_path_start: Joints,
        suffix: Vec<AnnotatedJoints>,
        transition_coefficients: &Joints,
    ) -> Self {
        let synthetic_start = AnnotatedJoints {
            joints: work_path_start,
            flags: PathFlags::LAND,
            move_into: MoveKind::Joint,
        };
        let mut rank_path = Vec::with_capacity(suffix.len() + 1);
        rank_path.push(synthetic_start);
        rank_path.extend_from_slice(&suffix);
        let rank = PlanRank::from_path(&rank_path, transition_coefficients);
        Self {
            work_path_start,
            suffix,
            rank,
        }
    }
}

/// Comparable quality summary for one planned output path.
#[derive(Clone, Copy, Debug, PartialEq)]
struct PlanRank {
    /// Number of distinct reconfiguration runs that interrupt trace motion.
    stroke_reconfigurations: usize,

    /// Number of output waypoints that are part of stroke-interrupting reconfiguration.
    stroke_reconfiguration_steps: usize,

    /// Sum of weighted transition costs between consecutive output waypoints.
    total_transition_cost: f64,

    /// Number of output waypoints in the planned path.
    output_steps: usize,
}

impl PlanRank {
    /// Computes ranking metrics from a complete or suffix output path.
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

    /// Returns true when the path does not interrupt trace motion with reconfiguration.
    fn is_good_enough(&self) -> bool {
        self.stroke_reconfigurations == 0
    }

    /// Orders ranks by stroke interruptions first, then reconfiguration length, cost, and size.
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

/// Returns true when reconfiguration happens inside the actual stroke instead of entry/exit moves.
fn is_stroke_interrupting_reconfiguration(flags: PathFlags) -> bool {
    flags.contains(PathFlags::RECONFIGURING)
        && !flags.intersects(
            PathFlags::ONBOARDING | PathFlags::LANDING | PathFlags::PARKING | PathFlags::PARK,
        )
}

/// Compares two joint states using the looser tolerance used for IK deduplication.
fn same_joints(left: &Joints, right: &Joints) -> bool {
    left.iter()
        .zip(right.iter())
        .all(|(left, right)| (left - right).abs() <= JOINT_DEDUP_EPSILON_RAD)
}

/// Returns the lowest-cost state index in a non-empty DP layer.
fn best_state_index(states: &[LayerState]) -> usize {
    states
        .iter()
        .enumerate()
        .min_by(|(_, left), (_, right)| left.total_cost.total_cmp(&right.total_cost))
        .map(|(index, _)| index)
        .expect("Layer should not be empty")
}

/// Returns up to `limit` state indices sorted by increasing total cost.
fn best_state_indices_by_cost(states: &[LayerState], limit: usize) -> Vec<usize> {
    let mut indices = (0..states.len()).collect::<Vec<_>>();
    indices.sort_by(|&left, &right| states[left].total_cost.total_cmp(&states[right].total_cost));
    let truncated_len = limit.min(indices.len());
    indices.truncate(truncated_len);
    indices
}

/// Keeps only the cheapest states in a DP layer.
fn limit_layer_states_by_cost(states: &mut Vec<LayerState>, limit: usize) {
    states.sort_by(|left, right| left.total_cost.total_cmp(&right.total_cost));
    states.truncate(limit);
}

/// Creates a graph failure used when cancellation interrupts Cartesian DP planning.
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

/// Sorts suffix candidates by the same quality rank used for complete planning outcomes.
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

/// Appends one parallel batch in strategy-index order so capping is independent of scheduling.
fn append_suffix_candidates_by_strategy_order(
    suffix_candidates: &mut Vec<SuffixPlanningOutcome>,
    mut batch_candidates: Vec<(usize, SuffixPlanningOutcome)>,
    solution_limit: Option<usize>,
) -> bool {
    batch_candidates.sort_by_key(|(strategy_index, _)| *strategy_index);

    for (_, candidate) in batch_candidates {
        if solution_limit.is_some_and(|limit| suffix_candidates.len() >= limit) {
            return true;
        }
        suffix_candidates.push(candidate);
    }

    solution_limit.is_some_and(|limit| suffix_candidates.len() >= limit)
}

/// Inserts a DP layer state, or updates an equivalent state if the new path is cheaper.
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

/// Reconstructs a joint path by following predecessor indices through DP layers.
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
    /// Applies this planner's output filtering setting to one annotated state.
    fn should_emit_output_state(&self, flags: PathFlags) -> bool {
        should_emit_output_state(self.include_linear_interpolation, flags)
    }

    /// Returns the configured number of prefix states to try for reconfiguration.
    fn reconfiguration_prefix_candidate_limit(&self) -> usize {
        if self.max_reconfiguration_prefix_candidates == 0 {
            DEFAULT_RECONFIGURATION_PREFIX_CANDIDATES
        } else {
            self.max_reconfiguration_prefix_candidates
        }
    }

    /// Returns the configured number of Cartesian-feasible suffixes to try for onboarding.
    fn onboarding_suffix_candidate_limit(&self) -> usize {
        if self.max_onboarding_suffix_candidates == 0 {
            DEFAULT_ONBOARDING_SUFFIX_CANDIDATES
        } else {
            self.max_onboarding_suffix_candidates
        }
    }

    /// Returns the configured maximum number of DP states retained per Cartesian layer.
    fn cartesian_layer_state_limit(&self) -> usize {
        if self.max_cartesian_layer_states == 0 {
            DEFAULT_CARTESIAN_LAYER_STATES
        } else {
            self.max_cartesian_layer_states
        }
    }

    /// Returns the configured number of suffix solutions to await before cancelling probes.
    fn max_solutions_await_limit(&self) -> usize {
        if self.max_solutions_await == 0 {
            DEFAULT_MAX_SOLUTIONS_AWAIT
        } else {
            self.max_solutions_await
        }
    }

    /// Plans a complete path from onboarding start through land, trace steps, and park.
    ///
    /// The planner first proves Cartesian suffix feasibility for landing IK candidates, then
    /// spends RRT time on ranked suffixes to connect the provided start state. Configured layer,
    /// suffix, and onboarding caps limit the fast pass only; if that pass cannot produce an
    /// acceptable complete path, the planner retries with less restrictive limits before returning
    /// failure or the best fallback path.
    pub fn plan(
        &self,
        from: &Joints,
        land: &Pose,
        steps: Vec<Pose>,
        park: &Pose,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        let (strategies, poses) = self.prepare_plan_inputs(from, land, steps, park)?;

        let layer_state_limit = self.cartesian_layer_state_limit();
        let fast_outcome =
            self.plan_with_cartesian_layer_limit(from, &strategies, &poses, layer_state_limit);

        if layer_state_limit == usize::MAX {
            return fast_outcome.map(|outcome| outcome.path);
        }

        match fast_outcome {
            Ok(outcome) if outcome.is_good_enough() => Ok(outcome.path),
            Ok(fast_fallback) => {
                if self.debug {
                    println!(
                        "Fast Cartesian planning produced a fallback path; retrying without layer beam limit"
                    );
                }
                match self.plan_with_cartesian_layer_limit(from, &strategies, &poses, usize::MAX) {
                    Ok(exhaustive_outcome) => {
                        if exhaustive_outcome.rank.is_better_than(&fast_fallback.rank) {
                            Ok(exhaustive_outcome.path)
                        } else {
                            Ok(fast_fallback.path)
                        }
                    }
                    Err(_) => Ok(fast_fallback.path),
                }
            }
            Err(first_err) => {
                if self.debug {
                    println!("Fast Cartesian planning failed; retrying without layer beam limit");
                }
                self.plan_with_cartesian_layer_limit(from, &strategies, &poses, usize::MAX)
                    .map(|outcome| outcome.path)
                    .map_err(|second_err| {
                        format!(
                            "fast Cartesian planning failed: {first_err}; exhaustive Cartesian retry failed: {second_err}"
                        )
                    })
            }
        }
    }

    /// Plans a path using the configured Cartesian layer beam as a hard limit.
    ///
    /// Unlike [`Self::plan`], this does not retry the Cartesian suffix graph with unbounded
    /// layers after beam pruning. Use this only when lower latency is more important than avoiding
    /// beam-pruning false negatives.
    pub fn plan_fast_approximate(
        &self,
        from: &Joints,
        land: &Pose,
        steps: Vec<Pose>,
        park: &Pose,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        let (strategies, poses) = self.prepare_plan_inputs(from, land, steps, park)?;
        self.plan_with_cartesian_layer_limit(
            from,
            &strategies,
            &poses,
            self.cartesian_layer_state_limit(),
        )
        .map(|outcome| outcome.path)
    }

    /// Validates common inputs and prepares landing strategies plus sampled Cartesian poses.
    fn prepare_plan_inputs(
        &self,
        from: &Joints,
        land: &Pose,
        steps: Vec<Pose>,
        park: &Pose,
    ) -> Result<(Vec<Joints>, Vec<AnnotatedPose>), String> {
        if self.robot.collides(from) {
            return Err("Onboarding point collides".into());
        }
        let strategies = self.robot.inverse_continuing(land, from);
        if strategies.is_empty() {
            return Err("Unable to start from onboarding point".into());
        }
        let poses = self.with_intermediate_poses(land, &steps, park);
        if self.debug {
            eprintln!("Probing {} Cartesian landing strategies", strategies.len());
        }
        Ok((strategies, poses))
    }

    /// Runs the suffix-first planner with a specific Cartesian graph layer-state limit.
    fn plan_with_cartesian_layer_limit(
        &self,
        from: &Joints,
        strategies: &[Joints],
        poses: &[AnnotatedPose],
        layer_state_limit: usize,
    ) -> Result<PlanningOutcome, String> {
        let max_solutions_await = self.max_solutions_await_limit();
        let mut suffix_candidates = self.collect_suffix_candidates(
            strategies,
            poses,
            Some(max_solutions_await),
            layer_state_limit,
        );

        let suffix_probe_was_limited = max_solutions_await < strategies.len();
        let mut best_fallback = None::<PlanningOutcome>;
        let mut onboarding_attempts = 0;

        if !suffix_candidates.is_empty() {
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

            if let Some(path) = self.try_onboarding_candidates(
                from,
                &suffix_candidates[..onboarding_limit],
                &mut best_fallback,
                &mut onboarding_attempts,
            ) {
                return Ok(path);
            }

            if onboarding_limit < suffix_candidates.len() {
                if self.debug {
                    println!(
                        "Fast onboarding candidates failed; trying {} remaining collected suffixes",
                        suffix_candidates.len() - onboarding_limit
                    );
                }
                if let Some(path) = self.try_onboarding_candidates(
                    from,
                    &suffix_candidates[onboarding_limit..],
                    &mut best_fallback,
                    &mut onboarding_attempts,
                ) {
                    return Ok(path);
                }
            }
        }

        if suffix_probe_was_limited {
            if self.debug {
                println!(
                    "Capped suffix candidates did not produce a complete path; retrying all {} strategies",
                    strategies.len()
                );
            }

            let mut suffix_candidates =
                self.collect_suffix_candidates(strategies, poses, None, layer_state_limit);
            if !suffix_candidates.is_empty() {
                sort_suffix_candidates_by_rank(&mut suffix_candidates);
                if self.debug {
                    println!(
                        "Trying onboarding RRT for all {} Cartesian-feasible suffixes",
                        suffix_candidates.len()
                    );
                }
                if let Some(path) = self.try_onboarding_candidates(
                    from,
                    &suffix_candidates,
                    &mut best_fallback,
                    &mut onboarding_attempts,
                ) {
                    return Ok(path);
                }
            }
        }

        if let Some(outcome) = best_fallback {
            Ok(outcome)
        } else if onboarding_attempts == 0 {
            Err(format!(
                "No Cartesian suffix worked out of {} strategies",
                strategies.len()
            ))
        } else {
            Err(format!(
                "No onboarding RRT worked out for {} Cartesian-feasible suffix attempts",
                onboarding_attempts
            ))
        }
    }

    /// Collects Cartesian-feasible suffix candidates, optionally stopping after `solution_limit`.
    fn collect_suffix_candidates(
        &self,
        strategies: &[Joints],
        poses: &[AnnotatedPose],
        solution_limit: Option<usize>,
        layer_state_limit: usize,
    ) -> Vec<SuffixPlanningOutcome> {
        let suffix_stop = Arc::new(AtomicBool::new(false));
        let solution_limit = solution_limit.map(|limit| limit.max(1));
        let batch_size = solution_limit.unwrap_or_else(|| strategies.len().max(1));
        let mut suffix_candidates = Vec::new();

        for (batch_index, strategy_batch) in strategies.chunks(batch_size).enumerate() {
            let strategy_offset = batch_index * batch_size;
            let batch_candidates = strategy_batch
                .par_iter()
                .enumerate()
                .filter_map(|(strategy_index_in_batch, strategy)| {
                    if suffix_stop.load(Ordering::Relaxed) {
                        return None;
                    }

                    match self.probe_cartesian_suffix(
                        strategy,
                        poses,
                        &suffix_stop,
                        layer_state_limit,
                    ) {
                        Ok(suffix) => {
                            let outcome = SuffixPlanningOutcome::new(
                                *strategy,
                                suffix,
                                &self.transition_coefficients,
                            );
                            if self.debug {
                                eprintln!(
                                    "Cartesian suffix worked out: {:?}, rank {:?}",
                                    strategy, outcome.rank
                                );
                            }
                            Some((strategy_offset + strategy_index_in_batch, outcome))
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

            if append_suffix_candidates_by_strategy_order(
                &mut suffix_candidates,
                batch_candidates,
                solution_limit,
            ) {
                break;
            }
        }

        suffix_candidates
    }

    /// Tries already ranked suffix candidates and returns the first complete path good enough to use.
    fn try_onboarding_candidates(
        &self,
        from: &Joints,
        suffix_candidates: &[SuffixPlanningOutcome],
        best_fallback: &mut Option<PlanningOutcome>,
        onboarding_attempts: &mut usize,
    ) -> Option<PlanningOutcome> {
        let onboarding_stop = AtomicBool::new(false);

        for suffix_candidate in suffix_candidates.iter().cloned() {
            *onboarding_attempts += 1;
            let work_path_start = suffix_candidate.work_path_start;
            match self.attach_onboarding(from, suffix_candidate, &onboarding_stop) {
                Ok(path) => {
                    let outcome = PlanningOutcome::new(path, &self.transition_coefficients);
                    if self.debug {
                        eprintln!(
                            "Strategy worked out after onboarding: {:?}, rank {:?}",
                            work_path_start, outcome.rank
                        );
                    }

                    if outcome.is_good_enough() {
                        return Some(outcome);
                    }

                    if best_fallback
                        .as_ref()
                        .is_none_or(|current| outcome.rank.is_better_than(&current.rank))
                    {
                        *best_fallback = Some(outcome);
                    }
                }
                Err(msg) => {
                    if self.debug {
                        println!("Onboarding failed: {}", msg);
                    }
                }
            }
        }

        None
    }

    /// Prepends onboarding RRT movement to an already feasible Cartesian suffix.
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

    /// Plans the Cartesian suffix for one landing strategy before onboarding RRT is attempted.
    ///
    /// This method may adaptively refine failed edges, and may use RRT reconfiguration inside
    /// the suffix when allowed.
    fn probe_cartesian_suffix(
        &self,
        work_path_start: &Joints,
        poses: &[AnnotatedPose],
        stop: &AtomicBool,
        layer_state_limit: usize,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        if self.debug {
            eprintln!("Cartesian suffix planning started, computing strategy {work_path_start:?}");
        }

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
                layer_state_limit,
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

                    let reconfigured = {
                        let mut append_state = ReconfigurationAppendState {
                            stop,
                            trace: &mut trace,
                            previous_joints: &mut previous_joints,
                            step: &mut step,
                        };
                        self.append_reconfiguration_candidates(
                            &failure.candidates,
                            &poses[pose_index..failed_pose_index],
                            failed_pose,
                            &mut append_state,
                        )
                    };

                    if !reconfigured {
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

    /// Inserts a midpoint before `to_index` when a failed transition can still be refined.
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

    /// Appends a Cartesian graph extension to the public trace and updates planning cursors.
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

    /// Tries failed-prefix candidates in order until one can bridge the failed pose by RRT.
    fn append_reconfiguration_candidates(
        &self,
        candidates: &[CartesianGraphFailureCandidate],
        prefix_poses: &[AnnotatedPose],
        failed_pose: &AnnotatedPose,
        state: &mut ReconfigurationAppendState<'_>,
    ) -> bool {
        let base_trace_len = state.trace.len();
        let base_previous_joints = *state.previous_joints;
        let base_step = *state.step;

        for (candidate_index, candidate) in candidates.iter().enumerate() {
            state.trace.truncate(base_trace_len);
            *state.previous_joints = base_previous_joints;
            *state.step = base_step;

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
                state.trace,
                state.previous_joints,
                state.step,
            );

            debug_assert!(
                same_joints(state.previous_joints, &candidate.transition.previous),
                "Reconfiguration candidate prefix should end at its transition start"
            );

            if self.append_reconfiguration(
                state.previous_joints,
                failed_pose,
                &candidate.transition,
                state.stop,
                state.trace,
            ) {
                return true;
            }
        }

        state.trace.truncate(base_trace_len);
        *state.previous_joints = base_previous_joints;
        *state.step = base_step;
        false
    }

    /// Appends a joint-space RRT bridge from the current state to one target IK solution.
    fn append_reconfiguration(
        &self,
        previous_joints: &Joints,
        pose: &AnnotatedPose,
        transition: &Transition,
        stop: &AtomicBool,
        trace: &mut Vec<AnnotatedJoints>,
    ) -> bool {
        if self.debug {
            eprintln!("Closing step with RRT");
        }
        for next in &transition.solutions {
            let path = self.rrt.plan_rrt(previous_joints, next, self.robot, stop);
            if let Ok(path) = path {
                if self.debug {
                    eprintln!("  ... closed with RRT {} steps", path.len());
                }
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

    /// Appends the initial joint-space path from the requested start to the selected land state.
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

    /// Plans a Cartesian path through all provided pose layers using dynamic programming.
    ///
    /// Each layer contains IK states for one target pose. Equivalent states are deduplicated, the
    /// cheapest states are retained as a beam, and failure reports contain the best previous-layer
    /// prefixes for possible RRT reconfiguration.
    /// The returned path excludes `starting` and contains one joint state per target pose.
    fn plan_cartesian_graph(
        &self,
        starting: &Joints,
        from: &AnnotatedPose,
        targets: &[AnnotatedPose],
        stop: &AtomicBool,
        layer_state_limit: usize,
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

            if layer_state_limit != usize::MAX {
                limit_layer_states_by_cost(&mut next_layer, layer_state_limit);
            }
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

    /// Prints detailed diagnostics for failed Cartesian transitions when debug logging is enabled.
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

    /// Builds the annotated pose sequence including land, trace, park, and fixed-step midpoints.
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

    /// Adds fixed-step intermediate poses between two endpoints without adding the endpoints.
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
        AnnotatedJoints, AnnotatedPose, Cartesian, DEFAULT_MAX_SOLUTIONS_AWAIT,
        DEFAULT_ONBOARDING_SUFFIX_CANDIDATES, DEFAULT_RECONFIGURATION_PREFIX_CANDIDATES,
        DEFAULT_TRANSITION_COSTS, LayerState, MoveKind, PathFlags, PlanRank, SuffixPlanningOutcome,
        add_or_update_state, append_suffix_candidates_by_strategy_order,
        best_state_indices_by_cost, canceled_cartesian_graph_failure, flag_representation,
        interpolation_flags_for_edge, is_stroke_interrupting_reconfiguration,
        limit_layer_states_by_cost, reconfiguring_output_flags, should_emit_output_state,
        sort_suffix_candidates_by_rank,
    };
    use crate::collisions::{CheckMode, RobotBody, SafetyDistances};
    use crate::constraints::Constraints;
    use crate::kinematic_traits::{Joints, Kinematics, Pose, Singularity, Solutions};
    use crate::kinematics_with_shape::KinematicsWithShape;
    use crate::rrt::RRTPlanner;
    use glam::DVec3;
    use parry3d::math::Vector;
    use parry3d::shape::TriMesh;
    use std::sync::Arc;
    use std::sync::atomic::AtomicBool;

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
    fn flag_representation_avoids_original_alias_for_single_original_bits() {
        let representation = flag_representation(
            &(PathFlags::TRACE | PathFlags::FORWARDS | PathFlags::BACKWARDS | PathFlags::DEBUG),
        );

        assert!(representation.contains("TRACE"));
        assert!(representation.contains("FORWARDS"));
        assert!(representation.contains("BACKWARDS"));
        assert!(representation.contains("DEBUG"));
        assert!(!representation.contains("ORIGINAL"));

        let exact_original = flag_representation(&PathFlags::ORIGINAL);
        assert!(exact_original.contains("LAND"));
        assert!(exact_original.contains("PARK"));
        assert!(exact_original.contains("TRACE"));
        assert!(exact_original.contains("ORIGINAL"));
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
    fn reconfiguration_accounting_is_role_specific() {
        assert!(is_stroke_interrupting_reconfiguration(
            PathFlags::TRACE | PathFlags::RECONFIGURING
        ));
        assert!(!is_stroke_interrupting_reconfiguration(
            PathFlags::LANDING | PathFlags::RECONFIGURING
        ));
        assert!(!is_stroke_interrupting_reconfiguration(
            PathFlags::PARKING | PathFlags::RECONFIGURING
        ));
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

    #[test]
    fn suffix_rank_includes_landing_to_first_suffix_edge() {
        let suffix = vec![
            joint_step(10.0, PathFlags::TRACE, MoveKind::Cartesian),
            joint_step(11.0, PathFlags::TRACE, MoveKind::Cartesian),
        ];
        let already_at_first_suffix =
            SuffixPlanningOutcome::new(joints(10.0), suffix.clone(), &DEFAULT_TRANSITION_COSTS);
        let far_from_first_suffix =
            SuffixPlanningOutcome::new(joints(0.0), suffix, &DEFAULT_TRANSITION_COSTS);

        assert!(
            already_at_first_suffix
                .rank
                .is_better_than(&far_from_first_suffix.rank)
        );
        assert!(
            already_at_first_suffix.rank.total_transition_cost
                < far_from_first_suffix.rank.total_transition_cost
        );
    }

    #[test]
    fn limited_suffix_candidates_are_appended_by_strategy_order_not_completion_order() {
        let mut candidates = Vec::new();
        let completion_order = vec![
            (
                3,
                suffix_candidate(
                    4.0,
                    vec![joint_step(4.0, PathFlags::PARK, MoveKind::Cartesian)],
                ),
            ),
            (
                1,
                suffix_candidate(
                    2.0,
                    vec![joint_step(2.0, PathFlags::PARK, MoveKind::Cartesian)],
                ),
            ),
            (
                0,
                suffix_candidate(
                    1.0,
                    vec![joint_step(1.0, PathFlags::PARK, MoveKind::Cartesian)],
                ),
            ),
            (
                2,
                suffix_candidate(
                    3.0,
                    vec![joint_step(3.0, PathFlags::PARK, MoveKind::Cartesian)],
                ),
            ),
        ];

        let limit_reached =
            append_suffix_candidates_by_strategy_order(&mut candidates, completion_order, Some(3));

        let selected_starts = candidates
            .iter()
            .map(|candidate| candidate.work_path_start[0])
            .collect::<Vec<_>>();
        assert!(limit_reached);
        assert_eq!(selected_starts, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn plan_cartesian_graph_honors_stop_flag() {
        let robot = test_robot();
        let planner = test_planner(&robot, 3);
        let starting = joints(0.0);
        let from = annotated_pose_at(0.0, PathFlags::LAND);
        let targets = vec![annotated_pose_at(1.0, PathFlags::TRACE)];
        let stop = AtomicBool::new(true);

        let failure = planner
            .plan_cartesian_graph(
                &starting,
                &from,
                &targets,
                &stop,
                planner.cartesian_layer_state_limit(),
            )
            .expect_err("stopped graph planning should return a failure");

        assert_eq!(failure.candidates.len(), 1);
        let candidate = &failure.candidates[0];
        assert!(candidate.planned_prefix.is_empty());
        assert_eq!(candidate.transition.previous, starting);
        assert!(candidate.transition.solutions.is_empty());
        assert!(candidate.transition.from.flags.contains(PathFlags::LAND));
        assert!(candidate.transition.to.flags.contains(PathFlags::TRACE));
    }

    #[test]
    fn plan_cartesian_graph_reports_only_retained_states_after_beam_pruning() {
        let robot = test_robot();
        let planner = test_planner(&robot, 2);
        let targets = vec![
            annotated_pose_at(1.0, PathFlags::TRACE),
            annotated_pose_at(2.0, PathFlags::TRACE),
        ];
        let stop = AtomicBool::new(false);

        let failure = planner
            .plan_cartesian_graph(
                &joints(0.0),
                &annotated_pose(PathFlags::LAND),
                &targets,
                &stop,
                planner.cartesian_layer_state_limit(),
            )
            .expect_err("tight beam should prune the only branch that can reach the second target");

        let previous_values = failure
            .candidates
            .iter()
            .map(|candidate| candidate.transition.previous[0])
            .collect::<Vec<_>>();

        assert_eq!(previous_values, vec![1.0, 2.0]);
        assert_eq!(failure.candidates.len(), 2);
        assert!(
            failure
                .candidates
                .iter()
                .all(|candidate| candidate.planned_prefix.len() == 1)
        );
        assert!(
            failure
                .candidates
                .iter()
                .all(|candidate| candidate.transition.solutions.is_empty())
        );
    }

    #[test]
    fn plan_cartesian_graph_keeps_required_branch_when_beam_is_wide_enough() {
        let robot = test_robot();
        let planner = test_planner(&robot, 3);
        let targets = vec![
            annotated_pose_at(1.0, PathFlags::TRACE),
            annotated_pose_at(2.0, PathFlags::TRACE),
        ];
        let stop = AtomicBool::new(false);

        let path = match planner.plan_cartesian_graph(
            &joints(0.0),
            &annotated_pose(PathFlags::LAND),
            &targets,
            &stop,
            planner.cartesian_layer_state_limit(),
        ) {
            Ok(path) => path,
            Err(_) => panic!("wider beam should keep the required branch"),
        };

        assert_eq!(path, vec![joints(3.0), joints(4.0)]);
    }

    #[test]
    fn plan_retries_unbounded_layer_after_beam_pruning() {
        let robot = test_robot();
        let mut planner = test_planner(&robot, 2);
        planner.check_step_m = 10.0;
        planner.check_step_rad = 10.0;

        let path = planner
            .plan(
                &joints(0.0),
                &pose_at(0.0),
                vec![pose_at(1.0)],
                &pose_at(2.0),
            )
            .expect("unbounded retry should recover the branch pruned by the fast beam");

        assert_eq!(path.len(), 3);
        assert_eq!(path[0].joints, joints(0.0));
        assert_eq!(path[1].joints, joints(3.0));
        assert_eq!(path[2].joints, joints(4.0));
        assert!(path[0].flags.contains(PathFlags::LAND));
        assert!(path[1].flags.contains(PathFlags::TRACE));
        assert!(path[2].flags.contains(PathFlags::PARK));
    }

    #[test]
    fn plan_without_linear_interpolation_outputs_land_trace_park_with_move_kinds() {
        let robot = linear_robot();
        let mut planner = test_planner(&robot, usize::MAX);
        planner.check_step_m = 0.25;
        planner.check_step_rad = 10.0;
        planner.include_linear_interpolation = false;

        let path = planner
            .plan(
                &joints(0.0),
                &pose_at(0.0),
                vec![pose_at(1.0)],
                &pose_at(2.0),
            )
            .expect("omitting interpolation should still return semantic waypoints");

        assert_eq!(path.len(), 3);
        assert_eq!(path[0].joints, joints(0.0));
        assert_eq!(path[1].joints, joints(1.0));
        assert_eq!(path[2].joints, joints(2.0));
        assert!(path[0].flags.contains(PathFlags::LAND));
        assert!(path[1].flags.contains(PathFlags::TRACE));
        assert!(path[2].flags.contains(PathFlags::PARK));
        assert_eq!(path[0].move_into, MoveKind::Joint);
        assert_eq!(path[1].move_into, MoveKind::Cartesian);
        assert_eq!(path[2].move_into, MoveKind::Cartesian);
        assert!(
            path.iter()
                .all(|step| !step.flags.contains(PathFlags::LIN_INTERP))
        );
        assert!(
            path.windows(2)
                .all(|window| window[0].joints != window[1].joints)
        );
    }

    #[test]
    fn plan_fast_approximate_keeps_beam_limit_as_hard_boundary() {
        let robot = test_robot();
        let mut planner = test_planner(&robot, 2);
        planner.check_step_m = 10.0;
        planner.check_step_rad = 10.0;

        let err = planner
            .plan_fast_approximate(
                &joints(0.0),
                &pose_at(0.0),
                vec![pose_at(1.0)],
                &pose_at(2.0),
            )
            .expect_err("approximate mode should keep the tight beam pruning failure");

        assert!(err.contains("No Cartesian suffix worked out"));
    }

    #[test]
    fn plan_retries_uncapped_suffixes_after_capped_onboarding_failures() {
        let robot = capped_retry_robot();
        let mut planner = test_planner(&robot, 3);
        planner.check_step_m = 10.0;
        planner.check_step_rad = 10.0;
        planner.max_solutions_await = 3;
        planner.max_onboarding_suffix_candidates = 3;
        planner.rrt.max_try = 0;

        let path = planner
            .plan(&joints(0.0), &pose_at(0.0), Vec::new(), &pose_at(1.0))
            .expect("uncapped retry should try the fourth reachable suffix");

        assert_eq!(path.len(), 2);
        assert_eq!(path[0].joints, joints(0.0));
        assert!(path[0].flags.contains(PathFlags::LAND));
        assert!(path[1].flags.contains(PathFlags::PARK));
    }

    /// Deterministic fake kinematics used to exercise Cartesian graph branching behavior.
    struct GraphTestKinematics {
        constraints: Option<Constraints>,
    }

    impl GraphTestKinematics {
        /// Creates fake kinematics without joint constraints because RRT is not used by these tests.
        fn new() -> Self {
            Self { constraints: None }
        }

        /// Returns pose-dependent IK branches that make one expensive first-layer branch necessary.
        fn solutions_for_pose(&self, pose: &Pose, previous: &Joints) -> Solutions {
            match rounded_x(pose) {
                0 => vec![joints(0.0)],
                1 => vec![joints(1.0), joints(2.0), joints(3.0)],
                2 if same_joint_value(previous, 3.0) => vec![joints(4.0)],
                2 => Vec::new(),
                _ => Vec::new(),
            }
        }
    }

    impl Kinematics for GraphTestKinematics {
        /// Returns deterministic IK solutions for the target pose with a zero previous state.
        fn inverse(&self, pose: &Pose) -> Solutions {
            self.solutions_for_pose(pose, &joints(0.0))
        }

        /// Returns deterministic IK solutions that may depend on the previous graph state.
        fn inverse_continuing(&self, pose: &Pose, previous: &Joints) -> Solutions {
            self.solutions_for_pose(pose, previous)
        }

        /// Returns identity because forward kinematics is irrelevant for graph tests.
        fn forward(&self, _qs: &Joints) -> Pose {
            Pose::identity()
        }

        /// Reuses the 6-DOF fake IK because the tests do not distinguish 5-DOF behavior.
        fn inverse_5dof(&self, pose: &Pose, _j6: f64) -> Solutions {
            self.inverse(pose)
        }

        /// Reuses continuing fake IK because the tests do not distinguish 5-DOF behavior.
        fn inverse_continuing_5dof(&self, pose: &Pose, prev: &Joints) -> Solutions {
            self.inverse_continuing(pose, prev)
        }

        /// Provides no constraints because RRT sampling is outside these graph tests.
        fn constraints(&self) -> &Option<Constraints> {
            &self.constraints
        }

        /// Reports no singularity because singularity handling is outside these graph tests.
        fn kinematic_singularity(&self, _qs: &Joints) -> Option<Singularity> {
            None
        }

        /// Returns identity joint poses because collision checks are disabled in the test robot.
        fn forward_with_joint_poses(&self, _joints: &Joints) -> [Pose; 6] {
            [Pose::identity(); 6]
        }
    }

    /// Fake kinematics that maps pose x directly to all joint values for output-shape tests.
    struct LinearKinematics {
        constraints: Option<Constraints>,
    }

    impl LinearKinematics {
        fn new() -> Self {
            Self { constraints: None }
        }
    }

    impl Kinematics for LinearKinematics {
        fn inverse(&self, pose: &Pose) -> Solutions {
            vec![joints(pose.translation.x)]
        }

        fn inverse_continuing(&self, pose: &Pose, _previous: &Joints) -> Solutions {
            self.inverse(pose)
        }

        fn forward(&self, qs: &Joints) -> Pose {
            pose_at(qs[0])
        }

        fn inverse_5dof(&self, pose: &Pose, _j6: f64) -> Solutions {
            self.inverse(pose)
        }

        fn inverse_continuing_5dof(&self, pose: &Pose, prev: &Joints) -> Solutions {
            self.inverse_continuing(pose, prev)
        }

        fn constraints(&self) -> &Option<Constraints> {
            &self.constraints
        }

        fn kinematic_singularity(&self, _qs: &Joints) -> Option<Singularity> {
            None
        }

        fn forward_with_joint_poses(&self, joints: &Joints) -> [Pose; 6] {
            [self.forward(joints); 6]
        }
    }

    /// Fake kinematics where only the fourth landing strategy can onboard deterministically.
    struct CappedRetryKinematics {
        constraints: Option<Constraints>,
    }

    impl CappedRetryKinematics {
        fn new() -> Self {
            Self { constraints: None }
        }
    }

    impl Kinematics for CappedRetryKinematics {
        fn inverse(&self, pose: &Pose) -> Solutions {
            self.inverse_continuing(pose, &joints(0.0))
        }

        fn inverse_continuing(&self, pose: &Pose, previous: &Joints) -> Solutions {
            match rounded_x(pose) {
                0 => vec![joints(1.0), joints(2.0), joints(3.0), joints(0.0)],
                1 => vec![*previous],
                _ => Vec::new(),
            }
        }

        fn forward(&self, _qs: &Joints) -> Pose {
            Pose::identity()
        }

        fn inverse_5dof(&self, pose: &Pose, _j6: f64) -> Solutions {
            self.inverse(pose)
        }

        fn inverse_continuing_5dof(&self, pose: &Pose, prev: &Joints) -> Solutions {
            self.inverse_continuing(pose, prev)
        }

        fn constraints(&self) -> &Option<Constraints> {
            &self.constraints
        }

        fn kinematic_singularity(&self, _qs: &Joints) -> Option<Singularity> {
            None
        }

        fn forward_with_joint_poses(&self, _joints: &Joints) -> [Pose; 6] {
            [Pose::identity(); 6]
        }
    }

    /// Converts the x translation into a small integer selector for fake IK behavior.
    fn rounded_x(pose: &Pose) -> i32 {
        pose.translation.x.round() as i32
    }

    /// Checks the first joint value exactly enough for deterministic test branches.
    fn same_joint_value(joints: &Joints, value: f64) -> bool {
        (joints[0] - value).abs() <= f64::EPSILON
    }

    /// Builds a collision-free robot wrapper around the deterministic fake kinematics.
    fn test_robot() -> KinematicsWithShape {
        KinematicsWithShape {
            kinematics: Arc::new(GraphTestKinematics::new()),
            body: RobotBody {
                joint_meshes: std::array::from_fn(|_| test_trimesh()),
                tool: None,
                base: None,
                collision_environment: Vec::new(),
                safety: SafetyDistances::standard(CheckMode::NoCheck),
            },
        }
    }

    /// Builds a collision-free robot wrapper around the linear fake kinematics.
    fn linear_robot() -> KinematicsWithShape {
        KinematicsWithShape {
            kinematics: Arc::new(LinearKinematics::new()),
            body: RobotBody {
                joint_meshes: std::array::from_fn(|_| test_trimesh()),
                tool: None,
                base: None,
                collision_environment: Vec::new(),
                safety: SafetyDistances::standard(CheckMode::NoCheck),
            },
        }
    }

    /// Builds a Cartesian planner with a configurable graph beam width for tests.
    fn test_planner(
        robot: &KinematicsWithShape,
        max_cartesian_layer_states: usize,
    ) -> Cartesian<'_> {
        Cartesian {
            robot,
            check_step_m: 0.01,
            check_step_rad: 0.01,
            max_transition_cost: 100.0,
            transition_coefficients: [1.0; 6],
            linear_recursion_depth: 0,
            rrt: RRTPlanner {
                step_size_joint_space: 0.1,
                max_try: 1,
                smooth: 0,
                debug: false,
            },
            allow_reconfigure: false,
            max_reconfiguration_prefix_candidates: DEFAULT_RECONFIGURATION_PREFIX_CANDIDATES,
            max_onboarding_suffix_candidates: DEFAULT_ONBOARDING_SUFFIX_CANDIDATES,
            max_cartesian_layer_states,
            max_solutions_await: DEFAULT_MAX_SOLUTIONS_AWAIT,
            include_linear_interpolation: true,
            debug: false,
        }
    }

    /// Builds a collision-free robot wrapper around the capped-retry fake kinematics.
    fn capped_retry_robot() -> KinematicsWithShape {
        KinematicsWithShape {
            kinematics: Arc::new(CappedRetryKinematics::new()),
            body: RobotBody {
                joint_meshes: std::array::from_fn(|_| test_trimesh()),
                tool: None,
                base: None,
                collision_environment: Vec::new(),
                safety: SafetyDistances::standard(CheckMode::NoCheck),
            },
        }
    }

    /// Creates a minimal valid mesh for the collision body fields that are disabled in tests.
    fn test_trimesh() -> TriMesh {
        TriMesh::new(
            vec![
                Vector::new(0.0, 0.0, 0.0),
                Vector::new(1.0, 0.0, 0.0),
                Vector::new(0.0, 1.0, 0.0),
                Vector::new(0.0, 0.0, 1.0),
            ],
            vec![[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
        )
        .expect("test trimesh should be valid")
    }

    /// Creates a layer state with no predecessor for helper-level DP tests.
    fn layer_state(value: f64, total_cost: f64) -> LayerState {
        LayerState {
            joints: [value; 6],
            total_cost,
            predecessor: None,
        }
    }

    /// Creates a ranked suffix candidate for ordering tests.
    fn suffix_candidate(value: f64, suffix: Vec<AnnotatedJoints>) -> SuffixPlanningOutcome {
        SuffixPlanningOutcome::new([value; 6], suffix, &DEFAULT_TRANSITION_COSTS)
    }

    /// Creates an annotated pose at a chosen x coordinate for graph-planning tests.
    fn annotated_pose_at(x: f64, flags: PathFlags) -> AnnotatedPose {
        AnnotatedPose {
            pose: Pose::from_translation(DVec3::new(x, 0.0, 0.0)),
            flags,
            split_depth: 0,
        }
    }

    /// Creates an annotated identity pose for helper-level tests.
    fn annotated_pose(flags: PathFlags) -> AnnotatedPose {
        annotated_pose_at(0.0, flags)
    }

    /// Creates a pose at a chosen x coordinate for full-planner tests.
    fn pose_at(x: f64) -> Pose {
        Pose::from_translation(DVec3::new(x, 0.0, 0.0))
    }

    /// Creates a joint state where every joint has the same value.
    fn joints(value: f64) -> Joints {
        [value; 6]
    }

    /// Creates an output waypoint with repeated joint values for ranking tests.
    fn joint_step(value: f64, flags: PathFlags, move_into: MoveKind) -> AnnotatedJoints {
        AnnotatedJoints {
            joints: [value; 6],
            flags,
            move_into,
        }
    }
}
