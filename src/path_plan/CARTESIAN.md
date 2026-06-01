# Cartesian Stroke Planning

This note describes the planner implemented in `cartesian.rs`. The planner is for tasks where the tool center point
must follow a Cartesian stroke, such as welding, painting, washing, engraving, or similar surface work.

The hard part is not finding one inverse-kinematics solution for each TCP pose. A typical 6-axis robot can have
multiple IK branches for the same pose, and adjacent poses can choose different branches unless the planner controls
continuity. A path that looks correct in TCP space may still contain large joint jumps, collisions, or an unreachable
onboarding move from the robot's current state.

## Inputs and Output

`Cartesian::plan()` takes:

- `from`: the current joint state.
- `land`: a pose near the work surface where the Cartesian approach begins.
- `steps`: the requested stroke poses.
- `park`: a pose where the robot lifts away from the work surface.

It returns `Vec<AnnotatedJoints>`. Each output state includes:

- `joints`: the robot joint angles.
- `flags`: semantic path flags such as `LAND`, `LANDING`, `TRACE`, `PARKING`, `PARK`, `ONBOARDING`,
  `LIN_INTERP`, and `RECONFIGURING`.
- `move_into`: whether the move into this state is expected to be `Cartesian` or `Joint`.

`include_linear_interpolation` controls whether intermediate Cartesian check poses are emitted. Even when they are not
emitted, they are still used internally for IK, continuity, and collision checks.

## Main Planning Strategy

The current strategy is suffix-first:

1. Check that `from` is collision-free.
2. Compute IK strategies for `land` using `robot.inverse_continuing(land, from)`.
3. Build the annotated Cartesian pose sequence from `land`, `steps`, and `park`.
4. For each landing IK strategy, try to plan the Cartesian suffix first.
5. Stop suffix probing once `max_solutions_await` feasible suffixes have been collected.
6. Rank the feasible suffixes.
7. Try onboarding RRT only for the best few feasible suffixes.

This order matters. RRT is usually more expensive than checking a small Cartesian IK graph, so the planner avoids
spending RRT time reaching a landing configuration whose remaining Cartesian suffix is impossible.

Landing strategies are probed in parallel with Rayon. A suffix-stage `AtomicBool` cancellation flag is passed through
the Cartesian graph planner and suffix reconfiguration RRT. Once `max_solutions_await` feasible suffixes have been
collected, the flag is set and still-running suffix probes return early where they can.

## Pose Expansion

The planner does not check only the user-provided `steps`. It expands the Cartesian path into a denser sequence:

- `LAND`
- interpolated `LANDING` poses from `land` to the first trace pose
- every `TRACE` pose
- interpolated trace-to-trace poses
- interpolated `PARKING` poses from the last trace pose to `park`
- `PARK`

The spacing is controlled by:

- `check_step_m`: maximum translation spacing between sampled poses.
- `check_step_rad`: maximum angular spacing between sampled poses.

The interpolation flags preserve edge semantics. A midpoint inserted in a landing move remains `LANDING`, a midpoint
inside a parking move remains `PARKING`, and raster direction flags such as `FORWARDS` and `BACKWARDS` are carried
forward.

## Cartesian Graph Search

`plan_cartesian_graph()` plans the Cartesian suffix as a dynamic-programming graph:

- Each target pose becomes one graph layer.
- Each state in a layer is one IK solution for that pose.
- Edges connect previous-layer states to current-layer IK solutions.
- Edge cost is `transition_costs(previous, candidate, transition_coefficients)`.
- Edges whose cost is above `max_transition_cost` are rejected.

`KinematicsWithShape::inverse_continuing()` is used for target poses. That means IK solutions are filtered through the
robot's collision model before the graph sees them.

Near-duplicate joint states are deduplicated with `JOINT_DEDUP_EPSILON_RAD`. If the same layer gets numerically
equivalent states through different predecessors, only the cheaper predecessor is kept.

After each layer is built, states are sorted by accumulated prefix cost and truncated to `max_cartesian_layer_states`.
This makes the graph search a beam search. It is not complete, but it limits growth when IK returns many alternatives
or near-duplicates.

If all targets are planned, the planner reconstructs the cheapest final path by following predecessor indices backward
through the layers.

## Failure Candidates and Reconfiguration

If a Cartesian graph layer cannot be built, the graph planner returns several failure candidates instead of only the
single cheapest previous state. Each candidate contains:

- the Cartesian prefix that reached the previous state,
- the failed transition,
- the failed target pose's IK solutions from that previous state,
- the accumulated prefix cost.

The candidates are sorted by prefix cost and limited by `max_reconfiguration_prefix_candidates`.

The suffix planner handles a failed graph edge in this order:

1. Try adaptive refinement if the edge has not reached `linear_recursion_depth`.
2. If refinement is possible, insert the midpoint and run the graph again from there.
3. If refinement is exhausted and `allow_reconfigure` is false, fail the strategy.
4. If reconfiguration is allowed, try RRT from the best failure candidates.

A successful reconfiguration appends joint-space states with `MoveKind::Joint` and `RECONFIGURING`. `LIN_INTERP` is
removed from reconfiguration output flags, but landing, parking, and direction semantics are preserved where relevant.

Reconfiguration inside `LANDING`, `PARKING`, or `PARK` is treated as less serious than reconfiguration inside the
actual trace. Trace reconfiguration is considered a stroke interruption.

## Onboarding RRT

Once a Cartesian suffix is feasible, the planner tries to connect the original `from` joint state to the suffix's
landing joint state.

If `from` and the landing joint state are already equal within the joint deduplication tolerance, onboarding emits a
single `LAND` state. Otherwise it calls `RRTPlanner::plan_rrt()`.

RRT uses:

- `constraints()` from the robot model to sample valid random joint states,
- `kinematics.collides()` to reject colliding states,
- `step_size_joint_space` to control joint-space expansion resolution,
- `max_try` to limit search effort,
- `smooth` to spend a bounded number of shortcut checks simplifying a successful raw RRT path. A value of `0`
  disables this post-processing.

Only the best `max_onboarding_suffix_candidates` suffixes are tried for onboarding, and the feasible suffix pool is
itself capped by `max_solutions_await`. This keeps a large set of feasible suffixes from causing many expensive RRT
attempts.

## Ranking

Feasible suffixes and complete paths are ranked by `PlanRank`.

The comparison order is:

1. Fewer stroke-interrupting reconfiguration runs.
2. Fewer stroke-interrupting reconfiguration steps.
3. Lower total weighted transition cost.
4. Fewer output waypoints.

A path is considered good enough when it has no stroke-interrupting reconfiguration. Reconfiguration in onboarding,
landing, parking, or park moves does not count as a stroke interruption.

## Optimization Strategies

The planner uses several optimizations to keep Cartesian stroke planning practical when IK returns many branches or
when RRT would otherwise dominate runtime.

### Suffix Before Onboarding

The planner checks whether the Cartesian suffix can be followed before it runs onboarding RRT. This avoids spending RRT
time reaching a landing IK solution that cannot complete the actual stroke.

The workflow is:

1. Generate landing IK candidates.
2. Run Cartesian suffix planning from each landing candidate.
3. Keep only suffixes that are feasible.
4. Cancel still-running suffix probes after `max_solutions_await` feasible suffixes have been collected.
5. Rank those suffixes.
6. Run onboarding RRT only for the best `max_onboarding_suffix_candidates` suffixes.

This favors cheap deterministic IK graph work before expensive randomized joint-space search.

### Parallel Strategy Probing

Landing IK strategies are independent, so suffix planning for them is done with Rayon. The suffix-stage cancellation
flag is passed into Cartesian graph planning and suffix reconfiguration RRT so expensive workers can stop once the
planner has collected enough suffix solutions.

`max_solutions_await` controls how many feasible suffix solutions the planner waits for before setting that cancellation
flag. The default is 3. A smaller value returns from the parallel suffix phase earlier, while a larger value gives the
ranker more candidate suffixes to compare.

### Dynamic Programming Instead of Greedy IK

For the Cartesian suffix, the planner does not greedily choose the nearest IK solution at each pose. It builds a layered
graph where each layer is a sampled Cartesian pose and each state is an IK solution for that pose.

This lets the planner keep a more expensive state in an early layer if that state leads to a cheaper or feasible suffix
later. The final path is reconstructed from predecessor links after all layers are processed.

### IK Deduplication

Numerical IK can return near-identical joint states. Keeping all of them increases layer sizes without adding meaningful
choices. The planner merges states that are within `JOINT_DEDUP_EPSILON_RAD` and keeps the cheaper predecessor.

This reduces duplicate work in later layers and improves performance when the IK solver produces tiny numerical
variants of the same configuration.

### Beam-Limited Layers

After each dynamic-programming layer is built, states are sorted by accumulated prefix cost and truncated to
`max_cartesian_layer_states`.

This turns the full graph search into a beam search:

- Larger beams keep more alternatives and are less likely to prune a later-useful branch.
- Smaller beams are faster and use less memory.
- The default is intended as a practical bound, not a proof of completeness.

If a stroke fails unexpectedly, increasing `max_cartesian_layer_states` is one of the first tuning options to try.

### Multiple Reconfiguration Prefixes

When a Cartesian layer cannot be built, the planner keeps several previous-layer candidates sorted by prefix cost. It
does not assume the single cheapest prefix is the best RRT reconfiguration start.

This is important because a slightly more expensive IK branch may be much easier to reconfigure from. The number of
prefixes is controlled by `max_reconfiguration_prefix_candidates`.

### Adaptive Edge Refinement

Before using RRT for a failed Cartesian edge, the planner may split that edge by inserting an interpolated midpoint. The
inserted pose inherits semantic flags such as `LANDING`, `PARKING`, `FORWARDS`, and `BACKWARDS`.

Refinement helps distinguish a genuinely impossible transition from a transition that is simply too coarse for the
current sampling. `linear_recursion_depth` limits how many times this can happen.

### Ranked Fallbacks

The planner can return a path with reconfiguration, but it ranks paths so uninterrupted strokes win. Reconfiguration in
the actual trace is penalized more heavily than reconfiguration during onboarding, landing, parking, or park moves.

The ranking order deliberately prioritizes semantic path quality before raw joint cost:

1. Avoid interrupting the stroke.
2. Keep stroke interruptions short if they are unavoidable.
3. Prefer lower weighted transition cost.
4. Prefer fewer output waypoints.

### Early Cancellation

`plan_cartesian_graph()` checks the suffix-stage cancellation flag while processing target layers and previous-layer
states. Suffix reconfiguration RRT also checks the same flag. This prevents parallel workers from continuing expensive
graph or RRT work after the planner has already collected `max_solutions_await` feasible suffixes.

## Checks Made

The planner performs these checks:

- The starting joint state must be collision-free.
- The landing pose must have at least one IK solution from the start state.
- Each sampled Cartesian pose must have collision-free IK solutions.
- Each Cartesian graph edge must stay below `max_transition_cost`.
- Dense sampled poses from `check_step_m` and `check_step_rad` limit unobserved Cartesian motion between user poses.
- RRT states are checked for collision through the robot shape model.
- Cancellation is checked during Cartesian graph layers and by the RRT algorithm.

The planner checks sampled configurations, not every continuous point in space. Smaller `check_step_m`,
`check_step_rad`, and RRT step sizes increase coverage at higher cost.

## Important Parameters

- `check_step_m`: Cartesian translation sampling resolution.
- `check_step_rad`: Cartesian rotation sampling resolution.
- `max_transition_cost`: maximum allowed weighted joint change for one Cartesian edge.
- `transition_coefficients`: joint weights used for transition cost.
- `linear_recursion_depth`: maximum adaptive midpoint insertion depth for failed Cartesian edges.
- `allow_reconfigure`: enables joint-space RRT bridges inside the suffix when Cartesian continuity fails.
- `max_reconfiguration_prefix_candidates`: number of previous graph states to try for reconfiguration.
- `max_onboarding_suffix_candidates`: number of feasible Cartesian suffixes to try with onboarding RRT.
- `max_cartesian_layer_states`: beam width for dynamic-programming layers.
- `max_solutions_await`: number of feasible suffix solutions to collect before cancelling remaining suffix probes
  (default 3).
- `include_linear_interpolation`: controls whether internally checked interpolated poses appear in the output.
- `debug`: enables diagnostic output for failed transitions and planning choices.

## Practical Notes

- The planner depends heavily on good `land` and `park` poses. They should be close enough to the work surface that
  the Cartesian approach and lift are realistic, but far enough to leave room for safe onboarding and exit.
- A very small `max_transition_cost` may reject valid Cartesian movement. A very large value may allow branch jumps.
- A small `max_cartesian_layer_states` is faster but can prune a branch that later becomes necessary.
- RRT needs useful joint constraints. Without constraints, random sampling for onboarding or reconfiguration cannot
  explore the intended joint space well.
- Continuous revolute joints are currently deduplicated by raw joint differences. If continuous joints are important,
  shortest angular distance should eventually be used for deduplication and transition comparison.
