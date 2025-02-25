use anyhow::{anyhow, Result};
use arrsac::Arrsac;
use rand::thread_rng;
use sample_consensus::{Consensus, Estimator, Model};

use crate::organized_point::OrganizedPoint;
use crate::plane::Plane;

#[derive(Debug)]
pub struct PlaneBuilder {
    // Number of times the whole algorithm runs (in parallel threads). Due randomness involved,
    // the plane builder may not succee from the first time. Default is 64
    pub build_iterations: usize,

    // When sampling points for the plane, nearby points will be preferred to discover
    // local planes better (and not broad "compromised" solutions over many planes).
    // Default value 0.05
    pub nearby_radius: f32,

    // Arrsac algorithm parameter. Default 512
    pub initialization_hypotheses: usize,

    // Arrsac algorithm parameter. Default 128
    pub max_candidate_hypotheses: usize,

    // Arrsac algorithm parameter. Default 128
    pub block_size: usize,
}

impl Default for PlaneBuilder {
    fn default() -> Self {
        Self {
            build_iterations: 64,
            nearby_radius: 0.05,
            initialization_hypotheses: 512,
            max_candidate_hypotheses: 128,
            block_size: 128,
        }
    }
}

/// Implements the `Model` trait for `Plane`, enabling it to be used in ARRSAC.
impl Model<OrganizedPoint> for Plane {
    /// The absolute distance of the point from the plane
    fn residual(&self, point: &OrganizedPoint) -> f64 {
        // Compute the signed distance from the plane using the plane equation:
        // Distance = |(A*x + B*y + C*z + D)|
        let distance = self.normal.dot(&point.point.coords) + self.d;
        distance.abs() as f64
    }
}

struct PlaneEstimator;

impl Estimator<OrganizedPoint> for PlaneEstimator {
    type Model = Plane;
    type ModelIter = Option<Self::Model>;
    const MIN_SAMPLES: usize = 3;

    fn estimate<I: Iterator<Item = OrganizedPoint>>(&self, data: I) -> Self::ModelIter {
        fn generate_nearby_samples(points: &[OrganizedPoint], radius: f32) -> Vec<OrganizedPoint> {
            use rand::seq::SliceRandom;

            if points.is_empty() {
                return vec![];
            }

            // Pick a random center point
            let center = points.choose(&mut rand::thread_rng()).unwrap();

            // Select points within the specified radius
            let nearby_points: Vec<OrganizedPoint> = points
                .iter()
                .filter(|p| p.distance(center) < radius)
                .cloned()
                .collect();

            nearby_points
        }

        let points: Vec<_> = data.take(Self::MIN_SAMPLES).collect();
        if points.len() < Self::MIN_SAMPLES {
            return None;
        }

        // Select a subset of nearby points to prevent steep plane selection
        let nearby_points = generate_nearby_samples(&points, 0.05);

        if nearby_points.len() < 3 {
            return None;
        }

        let p1 = nearby_points[0];
        let p2 = nearby_points[1];
        let p3 = nearby_points[2];

        let normal = (p2.point - p1.point)
            .cross(&(p3.point - p1.point))
            .normalize();
        let d = -normal.dot(&p1.point.coords);

        Some(Plane { normal, d })
    }
}

impl PlaneBuilder {
    pub fn build_plane(
        &self,
        points_to_fit: &Vec<OrganizedPoint>,
        max_distance_till_plane: f32,
    ) -> Result<Plane> {
        if points_to_fit.len() < 3 {
            return Err(anyhow!(
                "Not enough points to fit a plane. Need at least 3 points, got {}.",
                points_to_fit.len()
            ));
        }

        use rayon::prelude::*;
        let best_inliers = (0..self.build_iterations)
            .into_par_iter() // Convert the range to a parallel iterator
            .filter_map(|_| {
                let mut arrsac = Arrsac::new(max_distance_till_plane as f64, thread_rng())
                    .initialization_hypotheses(self.initialization_hypotheses)
                    .max_candidate_hypotheses(self.max_candidate_hypotheses)
                    .block_size(self.block_size);
                if let Some((_plane, inliers)) =
                    arrsac.model_inliers(&PlaneEstimator, points_to_fit.iter().copied())
                {
                    Some(inliers)
                } else {
                    None
                }
            })
            .max_by_key(|inliers| inliers.len()) // Find the set of inliers with the maximum length
            .unwrap_or_default(); // If no inliers are found, return an empty set
        
        if best_inliers.is_empty() {
            return Err(anyhow!("No inliers found after {} iterations.", self.build_iterations));
        }

        let filtered_vertices: Vec<_> = best_inliers.iter().map(|&i| points_to_fit[i]).collect();
        Plane::fit(&filtered_vertices)
    }
}
