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

type ParryPoint = parry3d::math::Point<f32>;

use Classification::{Core, Edge, Noise};

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
pub fn cluster(eps: f32, min_points: usize, input: &Vec<ParryPoint>) -> Vec<Classification> {
    Model::new(eps, min_points).run(input)
}

/// DBSCAN parameters
pub struct Model {
    /// Epsilon value - maximum distance between points in a cluster
    pub eps_squared: f32,
    /// Minimum number of points in a cluster
    pub mpt: usize,

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
            mpt: min_points,
            c: Vec::new(),
            v: Vec::new(),
        }
    }

    fn expand_cluster(
        &mut self,
        population: &Vec<ParryPoint>,
        queue: &mut Vec<usize>,
        cluster: u8,
    ) -> bool {
        let mut new_cluster = false;
        while let Some(ind) = queue.pop() {
            let neighbors = self.range_query(&population[ind], population);
            if neighbors.len() < self.mpt {
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
    fn range_query(&self, sample: &ParryPoint, population: &Vec<ParryPoint>) -> Vec<usize> {
        #[inline]
        fn squared_euclidean_distance(a: &ParryPoint, b: &ParryPoint) -> f32 {
            let x = a.x - b.x;
            let y = a.y - b.y;
            let z = a.z - b.z;
            x * x + y * y + z * z
        }

        // Parallelizing this does only marginally increases performance for very large queries.
        // For anything smaller it is a disaster.
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
    pub fn run(mut self, population: &Vec<ParryPoint>) -> Vec<Classification> {
        self.c = vec![Noise; population.len()];
        self.v = vec![false; population.len()];

        let mut cluster = 0;
        let mut queue = Vec::with_capacity(population.len() / 2);

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
