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

#![warn(missing_debug_implementations, missing_docs, rust_2018_idioms)]

use kdtree::distance::squared_euclidean;
use num_traits::float::Float;
use num_traits::identities::Zero;
use rand::distributions::Uniform;
use rand::prelude::Distribution;
use std::fmt::Debug;
use std::mem;
use std::sync::atomic::{AtomicBool, Ordering};
use tracing::debug;

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
    fn extend<FF>(&mut self, q_target: &[N], extend_length: N, is_free: &mut FF) -> ExtendStatus
    where
        FF: FnMut(&[N]) -> bool,
    {
        assert!(extend_length > N::zero());
        let nearest_index = self.get_nearest_index(q_target);
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
        if is_free(&q_new) {
            let new_index = self.add_vertex(&q_new);
            self.add_edge(nearest_index, new_index);
            if squared_euclidean(&q_new, q_target).sqrt() < extend_length {
                return ExtendStatus::Reached(new_index);
            }
            debug!("target = {q_target:?}");
            debug!("advanced to {q_target:?}");
            return ExtendStatus::Advanced(new_index);
        }
        ExtendStatus::Trapped
    }
    fn connect<FF>(&mut self, q_target: &[N], extend_length: N, is_free: &mut FF) -> ExtendStatus
    where
        FF: FnMut(&[N]) -> bool,
    {
        loop {
            debug!("connecting...{q_target:?}");
            match self.extend(q_target, extend_length, is_free) {
                ExtendStatus::Trapped => return ExtendStatus::Trapped,
                ExtendStatus::Reached(index) => return ExtendStatus::Reached(index),
                ExtendStatus::Advanced(_) => {}
            };
        }
    }
    fn get_until_root(&self, index: usize) -> Vec<Vec<N>> {
        let mut nodes = Vec::new();
        let mut cur_index = index;
        while let Some(parent_index) = self.vertices[cur_index].parent_index {
            cur_index = parent_index;
            nodes.push(self.vertices[cur_index].data.clone())
        }
        nodes
    }
}

/// search the path from start to goal which is free, using random_sample function
/// Changes were made for this fuction 21 Dec 2024, 28 January 2025 by Bourumir Wyngs, making this
/// function Rayon friendly
pub fn dual_rrt_connect<FF, FR, N>(
    start: &[N],
    goal: &[N],
    mut is_free: FF,
    random_sample: FR,
    extend_length: N,
    num_max_try: usize,
    smoothed: bool,
    stop: &AtomicBool,
) -> Result<Vec<Vec<N>>, String>
where
    FF: FnMut(&[N]) -> bool,
    FR: Fn() -> Vec<N>,
    N: Float + Debug,
{
    assert_eq!(start.len(), goal.len());
    let mut tree_a = Tree::new("start", start.len());
    let mut tree_b = Tree::new("goal", start.len());
    tree_a.add_vertex(start);
    tree_b.add_vertex(goal);
    for _ in 0..num_max_try {
        if stop.load(Ordering::Relaxed) {
            return Err("Cancelled".to_string());
        }
        debug!("tree_a = {:?}", tree_a.vertices.len());
        debug!("tree_b = {:?}", tree_b.vertices.len());
        let q_rand = random_sample();
        let extend_status = tree_a.extend(&q_rand, extend_length, &mut is_free);
        match extend_status {
            ExtendStatus::Trapped => {}
            ExtendStatus::Advanced(new_index) | ExtendStatus::Reached(new_index) => {
                let q_new = &tree_a.vertices[new_index].data;
                if let ExtendStatus::Reached(reach_index) =
                    tree_b.connect(q_new, extend_length, &mut is_free)
                {
                    let mut a_all = tree_a.get_until_root(new_index);
                    let mut b_all = tree_b.get_until_root(reach_index);
                    a_all.reverse();
                    a_all.append(&mut b_all);
                    if tree_b.name == "start" {
                        a_all.reverse();
                    }

                    if smoothed {
                        smooth_path_looped(&mut a_all, extend_length, stop);
                    }
                    return Ok(a_all);
                }
            }
        }
        mem::swap(&mut tree_a, &mut tree_b);
    }
    Err("failed".to_string())
}


/// A smoother that does not rely on randomness but has the length limit.
fn smooth_path_looped<N>(
    path: &mut Vec<Vec<N>>,
    extend_length: N,
    stop: &AtomicBool,
) where
    N: Float + Debug,
{
    // We cannot really smooth long paths this way, it is worst case O(path.len()^3)
    if path.len() < 3 || path.len() > 24 {
        return;
    }

    // Continue smoothing until no changes are made
    'outer: loop {
        let mut smoothed = false; // Flag to track smoothing in this iteration

        // Iterate through all pairs of points (a, b)
        for a in 0..path.len() - 2 {
            for b in a + 2..path.len() {
                // Stop immediately if "stop" flag is set
                if stop.load(Ordering::Relaxed) {
                    return;
                }

                // Check if two points are close enough to smooth
                if squared_euclidean(&path[a], &path[b]).sqrt() < extend_length {
                    // Remove intermediate points between a and b
                    path.drain(a + 1..b);
                    smoothed = true; // Mark that a change was made
                    break 'outer; // Exit both loops, start a new pass
                }
            }
        }

        // If no smoothing was done, terminate the loop
        if !smoothed {
            break; // Exit the outer loop
        }
    }
}

#[test]
fn it_works() {
    use rand::distributions::{Distribution, Uniform};
    let stop = AtomicBool::new(false);
    let mut result = dual_rrt_connect(
        &[-1.2, 0.0],
        &[1.2, 0.0],
        |p: &[f64]| !(p[0].abs() < 1.0 && p[1].abs() < 1.0),
        || {
            let between = Uniform::new(-2.0, 2.0);
            let mut rng = rand::thread_rng();
            vec![between.sample(&mut rng), between.sample(&mut rng)]
        },
        0.2,
        1000,
        true,
        &stop, // never stops
    )
    .unwrap();
    println!("{result:?}");
    assert!(result.len() >= 4);
}
