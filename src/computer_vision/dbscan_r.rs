//! # A Density-Based Algorithm for Discovering Clusters
//!
//! This algorithm finds all points within `eps` distance of each other and
//! attempts to cluster them. If there are at least `mpt` points reachable
//! (within distance `eps`) from a given point P, then all reachable points are
//! clustered together. The algorithm then attempts to expand the cluster,
//! finding all border points reachable from each point in the cluster
//!
//!
//! See `Ester, Martin, et al. "A density-based algorithm for discovering
//! clusters in large spatial databases with noise." Kdd. Vol. 96. No. 34.
//! 1996.` for the original paper
//! 
//! Changes maked 8 February by Bourumir Wyngs: 
//! - Switching into parry3d::math::Point<f32>
//! - Reducing the number of categories till 256 (u8) that looks enough (we only use one)
//! - Do not pull the root for the distance, compare squared values. 
//! 
//! Thanks to the rusty_machine implementation for inspiration

use std::collections::HashMap;
use Classification::{Core, Edge, Noise};
use crate::organized_point::OrganizedPoint;

/// Classification according to the DBSCAN algorithm
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd)]
pub enum Classification {
    /// A point with at least `min_points` neighbors within `eps` diameter
    Core(u8),
    /// A point within `eps` of a core point, but has less than `min_points` neighbors
    Edge(u8),
    /// A point with no connections
    Noise,
}

/// Cluster datapoints using the DBSCAN algorithm
///
/// # Arguments
/// * `eps` - maximum distance between datapoints within a cluster
/// * `min_points` - minimum number of datapoints to make a cluster
/// * `input` - datapoints
pub fn cluster(eps: f32, min_points: usize, input: &Vec<OrganizedPoint>) -> Vec<Classification> {
    Model::new(eps, min_points).run(input)
}

pub fn largest_cluster(
    output: Vec<Classification>,
    points: &Vec<OrganizedPoint>,
) -> Vec<OrganizedPoint> {
    let mut cluster_counts: HashMap<u8, usize> = HashMap::new();

    // Count occurrences of each cluster (excluding Noise)
    for classification in output.iter() {
        if let Core(cluster_id) = classification {
            *cluster_counts.entry(*cluster_id).or_insert(0) += 1;
        }
    }

    // Find the largest cluster
    let largest_cluster_id = cluster_counts
        .into_iter()
        .max_by_key(|&(_, count)| count)
        .map(|(id, _)| id);

    // Filter output to keep only the largest cluster
    if let Some(largest_cluster_id) = largest_cluster_id {
        points
            .iter() // Iterate over borrowed points
            .zip(output.into_iter()) // Combine points and output into an iterator of pairs
            .filter_map(|(point, classification)| match classification {
                Core(cluster_id) if cluster_id == largest_cluster_id => {
                    Some(point.clone()) // Clone the point to collect it into a Vec
                }
                _ => None,
            })
            .collect() // Collect the filtered points into a Vec<OrganizedPoint>
    } else {
        Vec::new()
    }
}


/// DBSCAN parameters
pub struct Model {
    /// Epsilon value - maximum distance between points in a cluster
    pub eps_squared: f32,
    /// Minimum number of points in a cluster
    pub min_points: usize,

    c: Vec<Classification>,
    v: Vec<bool>,
}

impl Model {
    /// Create a new `Model` with a set of parameters
    ///
    /// # Arguments
    /// * `eps` - maximum distance between datapoints within a cluster
    /// * `min_points` - minimum number of datapoints to make a cluster
    pub fn new(eps: f32, min_points: usize) -> Model {
        Model {
            eps_squared: eps * eps,
            min_points: min_points,
            c: Vec::new(),
            v: Vec::new(),
        }
    }

    fn expand_cluster(
        &mut self,
        population: &Vec<OrganizedPoint>,
        queue: &mut Vec<usize>,
        cluster: u8,
    ) -> bool {
        let mut new_cluster = false;
        while let Some(ind) = queue.pop() {
            let neighbors = self.range_query(&population[ind], population);
            if neighbors.len() < self.min_points {
                continue;
            }
            new_cluster = true;
            self.c[ind] = Core(cluster);
            for n_idx in neighbors {
                // n_idx is at least an edge point
                if self.c[n_idx] == Noise {
                    self.c[n_idx] = Edge(cluster);
                }

                if self.v[n_idx] {
                    continue;
                }

                self.v[n_idx] = true;
                queue.push(n_idx);
            }
        }
        new_cluster
    }
    
    #[inline]
    fn range_query(&self, sample: &OrganizedPoint, population: &Vec<OrganizedPoint>) -> Vec<usize> {
        #[inline]
        fn squared_euclidean_distance(a: &OrganizedPoint, b: &OrganizedPoint) -> f32 {
            let x = a.point.x - b.point.x;
            let y = a.point.y - b.point.y;
            let z = a.point.z - b.point.z;
            x * x + y * y + z * z
        }

        // Parallelizing this does only marginally increases performance for very large queries.
        // For anything smaller it is a disaster. KDTree was also tried, for some reason does not help
        population
            .iter()
            .enumerate()
            .filter(|(_, pt)| squared_euclidean_distance(sample, pt) < self.eps_squared)
            .map(|(idx, _)| idx)
            .collect()
    }

    /// Run the DBSCAN algorithm on a given population of datapoints.
    ///
    /// A vector of [`Classification`] enums is returned, where each element
    /// corresponds to a row in the input matrix.
    ///
    /// # Arguments
    /// * `population` - a matrix of datapoints, organized by rows
    ///
    pub fn run(mut self, population: &Vec<OrganizedPoint>) -> Vec<Classification> {
        self.c = vec![Noise; population.len()];
        self.v = vec![false; population.len()];

        let mut cluster = 0;
        let mut queue = Vec::new();

        for idx in 0..population.len() {
            if self.v[idx] {
                continue;
            }

            self.v[idx] = true;

            queue.push(idx);

            if self.expand_cluster(population, &mut queue, cluster) {
                cluster += 1;
            }
        }
        self.c
    }
}

