use parry3d::bounding_volume::Aabb;
use parry3d::math::Point;

use rayon::prelude::*;

/// A mesh for that row/column information (so adjacency information) is preseved
/// and color information is retains.
#[derive(Clone, Copy, Debug)]
pub struct OrganizedPoint {
    pub point: Point<f32>,
    pub row: usize,
    pub col: usize,
    pub color: [u8; 3],
}

impl OrganizedPoint {
    pub fn from_point(point_data: Point<f32>) -> OrganizedPoint {
        Self {
            point: point_data,
            row: usize::MAX,
            col: usize::MAX,
            color: [0, 0, 0],
        }
    }
    
    pub fn distance(&self, other: &OrganizedPoint) -> f32 {
        (self.point.coords - other.point.coords).norm()
    }
}

pub fn filter_points_in_aabb(points: &[OrganizedPoint], aabb: &Aabb) -> Vec<OrganizedPoint> {
    points
        .par_iter()                             
        .filter(|point| is_point_in_aabb(&point.point, aabb))
        .cloned()                                 
        .collect()                                
}
pub fn filter_points_not_in_aabb(points: &Vec<OrganizedPoint>, aabb: &Aabb) -> Vec<OrganizedPoint> {
    points
        .par_iter()
        .filter(|&point| !is_point_in_aabb(&point.point, aabb)) // Filter points inside the AABB
        .cloned() 
        .collect() 
}

/// Check if a point is inside an AABB
fn is_point_in_aabb(point: &Point<f32>, aabb: &Aabb) -> bool {
    point.x >= aabb.mins.x
        && point.x <= aabb.maxs.x
        && point.y >= aabb.mins.y
        && point.y <= aabb.maxs.y
        && point.z >= aabb.mins.z
        && point.z <= aabb.maxs.z
}

