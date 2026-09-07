/*
  Copyright 2017 Takashi Ogura

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

use kdtree::distance::squared_euclidean;
use num_traits::float::Float;
use num_traits::identities::Zero;
use std::fmt::Debug;
use std::mem;
use std::sync::atomic::{AtomicBool, Ordering};
use tracing::debug;

fn check_cancelled(stop: &AtomicBool) -> Result<(), String> {
    if stop.load(Ordering::Relaxed) {
        Err("Cancelled".to_string())
    } else {
        Ok(())
    }
}

#[derive(Debug)]
enum ExtendStatus {
    Reached(usize),
    Advanced(usize),
    Trapped,
}

/// Node that contains user data
#[derive(Debug, Clone)]
struct Node<T> {
    parent_index: Option<usize>,
    data: T,
}

impl<T> Node<T> {
    fn new(data: T) -> Self {
        Node {
            parent_index: None,
            data,
        }
    }
}

/// RRT
#[derive(Debug)]
struct Tree<N>
where
    N: Float + Zero + Debug,
{
    kdtree: kdtree::KdTree<N, usize, Vec<N>>,
    vertices: Vec<Node<Vec<N>>>,
    name: &'static str,
}

impl<N> Tree<N>
where
    N: Float + Zero + Debug,
{
    fn new(name: &'static str, dim: usize) -> Self {
        Tree {
            kdtree: kdtree::KdTree::new(dim),
            vertices: Vec::new(),
            name,
        }
    }
    fn add_vertex(&mut self, q: &[N]) -> usize {
        let index = self.vertices.len();
        self.kdtree.add(q.to_vec(), index).unwrap();
        self.vertices.push(Node::new(q.to_vec()));
        index
    }
    fn add_edge(&mut self, q1_index: usize, q2_index: usize) {
        self.vertices[q2_index].parent_index = Some(q1_index);
    }
    fn get_nearest_index(&self, q: &[N]) -> usize {
        *self.kdtree.nearest(q, 1, &squared_euclidean).unwrap()[0].1
    }
    fn extend<FF>(
        &mut self,
        q_target: &[N],
        extend_length: N,
        is_free: &mut FF,
        stop: &AtomicBool,
    ) -> Result<ExtendStatus, String>
    where
        FF: FnMut(&[N]) -> bool,
    {
        check_cancelled(stop)?;
        assert!(extend_length > N::zero());
        let nearest_index = self.get_nearest_index(q_target);
        check_cancelled(stop)?;
        let nearest_q = &self.vertices[nearest_index].data;
        let diff_dist = squared_euclidean(q_target, nearest_q).sqrt();
        let q_new = if diff_dist < extend_length {
            q_target.to_vec()
        } else {
            nearest_q
                .iter()
                .zip(q_target)
                .map(|(near, target)| *near + (*target - *near) * extend_length / diff_dist)
                .collect::<Vec<_>>()
        };
        debug!("q_new={q_new:?}");
        check_cancelled(stop)?;
        let free = is_free(&q_new);
        check_cancelled(stop)?;
        if free {
            let new_index = self.add_vertex(&q_new);
            self.add_edge(nearest_index, new_index);
            check_cancelled(stop)?;
            if squared_euclidean(&q_new, q_target).sqrt() < extend_length {
                return Ok(ExtendStatus::Reached(new_index));
            }
            debug!("target = {q_target:?}");
            debug!("advanced to {q_target:?}");
            return Ok(ExtendStatus::Advanced(new_index));
        }
        Ok(ExtendStatus::Trapped)
    }
    fn connect<FF>(
        &mut self,
        q_target: &[N],
        extend_length: N,
        is_free: &mut FF,
        stop: &AtomicBool,
    ) -> Result<ExtendStatus, String>
    where
        FF: FnMut(&[N]) -> bool,
    {
        loop {
            debug!("connecting...{q_target:?}");
            match self.extend(q_target, extend_length, is_free, stop)? {
                ExtendStatus::Trapped => return Ok(ExtendStatus::Trapped),
                ExtendStatus::Reached(index) => return Ok(ExtendStatus::Reached(index)),
                ExtendStatus::Advanced(_) => {}
            };
        }
    }
    fn get_until_root(&self, index: usize, stop: &AtomicBool) -> Result<Vec<Vec<N>>, String> {
        let mut nodes = Vec::new();
        let mut cur_index = index;
        loop {
            check_cancelled(stop)?;
            nodes.push(self.vertices[cur_index].data.clone());
            if let Some(parent_index) = self.vertices[cur_index].parent_index {
                cur_index = parent_index;
            } else {
                break;
            }
        }
        check_cancelled(stop)?;
        Ok(nodes)
    }
}

/// Searches for a collision-free path from `start` to `goal` with bidirectional
/// RRT-Connect.
///
/// The planner grows two trees, one rooted at `start` and one rooted at `goal`.
/// On each iteration it extends one tree toward a configuration returned by
/// `random_sample`, then tries to connect the other tree to the newly added
/// configuration. The two trees are swapped after each unsuccessful iteration,
/// so both sides of the problem are explored.
///
/// `is_free` must return `true` only when a configuration is valid and
/// collision-free. Before growing the trees, it checks `start`, then `goal`,
/// stopping at the first invalid configuration. If the endpoints are exactly
/// equal, the shared configuration is checked once. It also checks each newly
/// proposed configuration during tree growth. `random_sample` must
/// return configurations with the same dimension as `start` and `goal`.
/// `extend_length` is the maximum distance, in configuration space, added to a
/// tree in one extension step.
///
/// Returns a path from `start` to `goal` when the trees connect. The returned
/// path includes the endpoints and may include the connecting configuration from
/// both trees as adjacent duplicate entries.
/// If `start == goal` and the configuration is valid, returns a single-state
/// path without sampling, even when `num_max_try` is zero.
///
/// Returns `Err("Cancelled")` if `stop` is set before the planning is finished or
/// `Err("failed")` when either endpoint is invalid or no connection is found
/// after `num_max_try` iterations.
/// Cancellation is checked between tree extensions, during path reconstruction,
/// and before returning the result. Sampling and collision-check callbacks must
/// return before cancellation can be observed.
///
/// # Panics
///
/// Panics if `start` and `goal` have different dimensions, if a tree is extended
/// with a non-positive `extend_length`, or if `random_sample` returns a
/// configuration with a dimension different from the tree dimension.
pub fn dual_rrt_connect<FF, FR, N>(
    start: &[N],
    goal: &[N],
    mut is_free: FF,
    random_sample: FR,
    extend_length: N,
    num_max_try: usize,
    stop: &AtomicBool,
) -> Result<Vec<Vec<N>>, String>
where
    FF: FnMut(&[N]) -> bool,
    FR: Fn() -> Vec<N>,
    N: Float + Debug,
{
    assert_eq!(start.len(), goal.len());
    for root in [start, goal] {
        check_cancelled(stop)?;
        let valid = is_free(root);
        check_cancelled(stop)?;
        if !valid {
            return Err("failed".to_string());
        }
        if start == goal {
            let path = vec![start.to_vec()];
            check_cancelled(stop)?;
            return Ok(path);
        }
    }

    let mut tree_a = Tree::new("start", start.len());
    let mut tree_b = Tree::new("goal", start.len());
    tree_a.add_vertex(start);
    tree_b.add_vertex(goal);
    for _ in 0..num_max_try {
        check_cancelled(stop)?;
        debug!("tree_a = {:?}", tree_a.vertices.len());
        debug!("tree_b = {:?}", tree_b.vertices.len());
        let q_rand = random_sample();
        let extend_status = tree_a.extend(&q_rand, extend_length, &mut is_free, stop)?;
        match extend_status {
            ExtendStatus::Trapped => {}
            ExtendStatus::Advanced(new_index) | ExtendStatus::Reached(new_index) => {
                let q_new = &tree_a.vertices[new_index].data;
                if let ExtendStatus::Reached(reach_index) =
                    tree_b.connect(q_new, extend_length, &mut is_free, stop)?
                {
                    let mut a_all = tree_a.get_until_root(new_index, stop)?;
                    let mut b_all = tree_b.get_until_root(reach_index, stop)?;
                    a_all.reverse();
                    check_cancelled(stop)?;
                    a_all.append(&mut b_all);
                    check_cancelled(stop)?;
                    if tree_b.name == "start" {
                        a_all.reverse();
                    }
                    check_cancelled(stop)?;
                    return Ok(a_all);
                }
            }
        }
        mem::swap(&mut tree_a, &mut tree_b);
    }
    check_cancelled(stop)?;
    Err("failed".to_string())
}

#[cfg(test)]
mod tests {
    use super::dual_rrt_connect;
    use std::cell::RefCell;
    use std::sync::atomic::{AtomicBool, Ordering};

    #[test]
    fn invalid_initial_roots_are_rejected_without_sampling() {
        for invalid_root in [0.0_f64, 10.0] {
            let mut checked = Vec::new();
            let result = dual_rrt_connect(
                &[0.0],
                &[10.0],
                |q| {
                    checked.push(q[0]);
                    q[0] != invalid_root
                },
                || panic!("invalid roots must be rejected before sampling"),
                1.0,
                10,
                &AtomicBool::new(false),
            );

            assert_eq!(result, Err("failed".to_string()));
            let expected = if invalid_root == 0.0 {
                vec![0.0]
            } else {
                vec![0.0, 10.0]
            };
            assert_eq!(checked, expected);
        }
    }

    #[test]
    fn valid_initial_roots_are_checked_before_sampling() {
        let checked = RefCell::new(Vec::new());
        let result = dual_rrt_connect(
            &[0.0_f64],
            &[10.0],
            |q| {
                checked.borrow_mut().push(q[0]);
                true
            },
            || {
                assert_eq!(*checked.borrow(), vec![0.0, 10.0]);
                vec![1.0]
            },
            1.0,
            1,
            &AtomicBool::new(false),
        )
        .expect("valid roots should connect");

        assert_eq!(result.first().unwrap(), &[0.0]);
        assert_eq!(result.last().unwrap(), &[10.0]);
    }

    #[test]
    fn initial_roots_respect_cancellation_during_validation() {
        for cancelled_root in [0.0_f64, 10.0] {
            for is_free in [true, false] {
                let stop = AtomicBool::new(false);
                let mut checked = Vec::new();
                let result = dual_rrt_connect(
                    &[0.0],
                    &[10.0],
                    |q| {
                        checked.push(q[0]);
                        if q[0] == cancelled_root {
                            stop.store(true, Ordering::Relaxed);
                            is_free
                        } else {
                            true
                        }
                    },
                    || panic!("cancelled planning must not sample"),
                    1.0,
                    10,
                    &stop,
                );

                assert_eq!(result, Err("Cancelled".to_string()));
                let expected = if cancelled_root == 0.0 {
                    vec![0.0]
                } else {
                    vec![0.0, 10.0]
                };
                assert_eq!(checked, expected);
            }
        }
    }

    #[test]
    fn identical_valid_endpoints_return_single_state_without_sampling() {
        for max_try in [0, 10] {
            let stop = AtomicBool::new(false);
            let mut checks = 0;
            let result = dual_rrt_connect(
                &[0.0_f64],
                &[0.0],
                |q| {
                    checks += 1;
                    q[0].abs() < 0.1
                },
                || panic!("stationary planning must not sample"),
                1.0,
                max_try,
                &stop,
            );

            assert_eq!(result, Ok(vec![vec![0.0]]));
            assert_eq!(checks, 1);
        }
    }

    #[test]
    fn identical_invalid_endpoints_are_rejected_without_sampling() {
        let result = dual_rrt_connect(
            &[0.0_f64],
            &[0.0],
            |_| false,
            || panic!("stationary planning must not sample"),
            1.0,
            10,
            &AtomicBool::new(false),
        );

        assert_eq!(result, Err("failed".to_string()));
    }

    #[test]
    fn initial_roots_respect_existing_cancellation() {
        for goal in [0.0_f64, 10.0] {
            for max_try in [0, 10] {
                let result = dual_rrt_connect(
                    &[0.0],
                    &[goal],
                    |_| panic!("cancelled planning must not check collisions"),
                    || panic!("cancelled planning must not sample"),
                    1.0,
                    max_try,
                    &AtomicBool::new(true),
                );

                assert_eq!(result, Err("Cancelled".to_string()));
            }
        }
    }

    #[test]
    fn identical_endpoints_respect_cancellation_during_validation() {
        for is_free in [true, false] {
            let stop = AtomicBool::new(false);
            let result = dual_rrt_connect(
                &[0.0_f64],
                &[0.0],
                |_| {
                    stop.store(true, Ordering::Relaxed);
                    is_free
                },
                || panic!("stationary planning must not sample"),
                1.0,
                10,
                &stop,
            );

            assert_eq!(result, Err("Cancelled".to_string()));
        }
    }

    #[test]
    fn nearby_distinct_endpoints_are_not_collapsed() {
        let result = dual_rrt_connect(
            &[0.0_f64],
            &[1e-10],
            |_| true,
            || vec![1e-10],
            1.0,
            1,
            &AtomicBool::new(false),
        )
        .expect("nearby endpoints should connect");

        assert_eq!(result.first().unwrap(), &[0.0]);
        assert_eq!(result.last().unwrap(), &[1e-10]);
    }

    #[test]
    fn cancellation_during_sampling_prevents_tree_extension() {
        let stop = AtomicBool::new(false);
        let mut checked = Vec::new();
        let result = dual_rrt_connect(
            &[0.0_f64],
            &[10.0],
            |q| {
                checked.push(q[0]);
                true
            },
            || {
                stop.store(true, Ordering::Relaxed);
                vec![1.0]
            },
            1.0,
            1,
            &stop,
        );

        assert_eq!(result, Err("Cancelled".to_string()));
        assert_eq!(checked, vec![0.0, 10.0]);
    }

    #[test]
    fn cancellation_during_first_extension_prevents_connecting() {
        for sample in [1.0_f64, 10.0] {
            for is_free in [true, false] {
                let stop = AtomicBool::new(false);
                let mut checked = Vec::new();
                let result = dual_rrt_connect(
                    &[0.0],
                    &[10.0],
                    |q| {
                        checked.push(q[0]);
                        if checked.len() == 3 {
                            stop.store(true, Ordering::Relaxed);
                            is_free
                        } else {
                            true
                        }
                    },
                    || vec![sample],
                    1.0,
                    1,
                    &stop,
                );

                assert_eq!(result, Err("Cancelled".to_string()));
                assert_eq!(checked, vec![0.0, 10.0, 1.0]);
            }
        }
    }

    #[test]
    fn cancellation_during_connect_overrides_extension_status() {
        for goal in [2.0_f64, 10.0] {
            for is_free in [true, false] {
                let stop = AtomicBool::new(false);
                let mut checked = Vec::new();
                let result = dual_rrt_connect(
                    &[0.0],
                    &[goal],
                    |q| {
                        checked.push(q[0]);
                        if checked.len() == 4 {
                            stop.store(true, Ordering::Relaxed);
                            is_free
                        } else {
                            true
                        }
                    },
                    || vec![1.0],
                    1.0,
                    1,
                    &stop,
                );

                assert_eq!(result, Err("Cancelled".to_string()));
                assert_eq!(checked, vec![0.0, goal, 1.0, goal - 1.0]);
            }
        }
    }

    #[test]
    fn cancellation_after_several_connect_steps_stops_collision_checks() {
        for is_free in [true, false] {
            let stop = AtomicBool::new(false);
            let mut checked = Vec::new();
            let result = dual_rrt_connect(
                &[0.0_f64],
                &[10.0],
                |q| {
                    checked.push(q[0]);
                    if checked.len() == 6 {
                        stop.store(true, Ordering::Relaxed);
                        is_free
                    } else {
                        true
                    }
                },
                || vec![1.0],
                1.0,
                1,
                &stop,
            );

            assert_eq!(result, Err("Cancelled".to_string()));
            assert_eq!(checked, vec![0.0, 10.0, 1.0, 9.0, 8.0, 7.0]);
        }
    }

    #[test]
    fn exhausted_search_without_cancellation_still_fails() {
        let mut checked = Vec::new();
        let result = dual_rrt_connect(
            &[0.0_f64],
            &[10.0],
            |q| {
                checked.push(q[0]);
                q[0] == 0.0 || q[0] == 10.0
            },
            || vec![1.0],
            1.0,
            1,
            &AtomicBool::new(false),
        );

        assert_eq!(result, Err("failed".to_string()));
        assert_eq!(checked, vec![0.0, 10.0, 1.0]);
    }
}
