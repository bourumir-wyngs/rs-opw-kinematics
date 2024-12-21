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
use rand::distributions::{Distribution, Uniform};
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
/// Changes were made for this fuction 21 Dec 2024 by Bourumir Wyngs, making this
/// function Rayon friendly
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
                    return Ok(a_all);
                }
            }
        }
        mem::swap(&mut tree_a, &mut tree_b);
    }
    Err("failed".to_string())
}

/// select random two points, and try to connect.
pub fn smooth_path<FF, N>(
    path: &mut Vec<Vec<N>>,
    mut is_free: FF,
    extend_length: N,
    num_max_try: usize,
    stop: &AtomicBool,
) where
    FF: FnMut(&[N]) -> bool,
    N: Float + Debug,
{
    if path.len() < 3 {
        return;
    }
    let mut rng = rand::thread_rng();
    for _ in 0..num_max_try {
        if stop.load(Ordering::Relaxed) {
            break
        }
        let range1 = Uniform::new(0, path.len() - 2);
        let ind1 = range1.sample(&mut rng);
        let range2 = Uniform::new(ind1 + 2, path.len());
        let ind2 = range2.sample(&mut rng);
        let mut base_point = path[ind1].clone();
        let point2 = path[ind2].clone();
        let mut is_searching = true;
        while is_searching {
            let diff_dist = squared_euclidean(&base_point, &point2).sqrt();
            if diff_dist < extend_length {
                // reached!
                // remove path[ind1+1] ... path[ind2-1]
                let remove_index = ind1 + 1;
                for _ in 0..(ind2 - ind1 - 1) {
                    path.remove(remove_index);
                }
                if path.len() == 2 {
                    return;
                }
                is_searching = false;
            } else {
                let check_point = base_point
                    .iter()
                    .zip(point2.iter())
                    .map(|(near, target)| *near + (*target - *near) * extend_length / diff_dist)
                    .collect::<Vec<_>>();
                if !is_free(&check_point) {
                    // trapped
                    is_searching = false;
                } else {
                    // continue to extend
                    base_point = check_point;
                }
            }
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
        &stop, // never stops
    )
        .unwrap();
    println!("{result:?}");
    assert!(result.len() >= 4);
    smooth_path(
        &mut result,
        |p: &[f64]| !(p[0].abs() < 1.0 && p[1].abs() < 1.0),
        0.2,
        100,
    );
    println!("{result:?}");
    assert!(result.len() >= 3);
}
