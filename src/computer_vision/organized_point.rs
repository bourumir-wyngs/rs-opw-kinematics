use parry3d::math::Point;

/// A mesh for that row/column information (so adjacency information) is preseved
/// and color information is retains.
#[derive(Clone, Debug)]
pub struct OrganizedPoint {
    pub point: Point<f32>,
    pub row: usize,
    pub col: usize,
    //pub color: [f32; 3],
}

impl OrganizedPoint {
    pub fn from_point(point_data: Point<f32>) -> OrganizedPoint {
        Self {
            point: point_data,
            row: usize::MAX,
            col: usize::MAX,
            //color: [0.0, 0.0, 0.0],
        }
    }
}
