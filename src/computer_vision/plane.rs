use crate::organized_point::OrganizedPoint;
use nalgebra::{Matrix3, Point3, Vector3};
use sample_consensus::Model;

// Represent a plane in 3D space
#[derive(Clone, Copy, Debug)]
pub struct Plane {
    /// Nornam vector
    pub normal: Vector3<f32>,
    /// Distance from origin
    pub d: f32,
}

// Represent a projected point in 2D space
#[derive(Clone, Copy, Debug)]
pub struct ProjectedPoint {
    pub x: f32,
    pub y: f32,

    /// If vector of projected points has been derived from the vector of 3D points,
    /// id is the index of the parent point in the parent vector. Points are considered
    /// equal if they have the equal id.
    pub id: u32,
}

impl PartialEq for ProjectedPoint {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for ProjectedPoint {}

impl Plane {
    pub fn fit(points: &[OrganizedPoint]) -> Option<Plane> {
        let centroid = points
            .iter()
            .fold(Vector3::zeros(), |acc, p| acc + p.point.coords)
            / points.len() as f32;

        let mut covariance_matrix = Matrix3::zeros();
        for point in points {
            let diff = point.point.coords - centroid;
            covariance_matrix += diff * diff.transpose();
        }

        let svd = covariance_matrix.svd(true, true);
        if let Some(v_t) = svd.v_t {
            let normal = v_t.row(2).transpose();
            Some(Plane {
                normal: normal.normalize(),
                d: -normal.dot(&centroid),
            })
        } else {
            None
        }
    }

    pub fn filter(
        &self,
        points: &Vec<OrganizedPoint>,
        max_distance_till_plane: f32,
    ) -> Vec<OrganizedPoint> {
        let distance_threshold = max_distance_till_plane as f64; // Use a scaling factor
        let filtered_vertices: Vec<_> = points
            .iter()
            .cloned()
            .filter(|p| self.residual(&p) < distance_threshold)
            .collect();

        filtered_vertices
    }

    /// Takes a slice of `OrganizedPoint` and projects all points onto the plane,
    /// returning a vector of `OrganizedPoint` that lie on the plane.
    pub fn flatten(&self, points: &[OrganizedPoint]) -> Vec<OrganizedPoint> {
        points
            .iter()
            .map(|point| {
                // Calculate the scalar distance from the point to the plane
                let distance = point.point.coords.dot(&self.normal) + self.d;

                // Shift the point onto the plane using the plane's normal
                let projected_point = point.point.coords - distance * self.normal;

                OrganizedPoint {
                    point: Point3::from(projected_point),
                    ..point.clone()
                }
            })
            .collect() // Collect all projected points into a vector
    }

    // Project all points onto the plane and generate 2D projections
    pub fn project(&self, points: &Vec<OrganizedPoint>) -> Vec<ProjectedPoint> {
        // Ensure a robust selection of a reference vector
        let reference = if self.normal.x.abs() < 0.9 {
            Vector3::x_axis() // Prefer X unless normal is mostly X
        } else {
            Vector3::z_axis() // Otherwise, use Z
        };

        // Compute two orthonormal basis vectors spanning the plane
        let u = self.normal.cross(&reference).normalize();
        let v = self.normal.cross(&u).normalize();

        points
            .iter()
            .enumerate()
            .map(|(id, point)| {
                // Project onto the plane
                let distance = point.point.coords.dot(&self.normal) + self.d;
                let projected_point_3d = point.point.coords - distance * self.normal;

                // Convert to 2D using the plane's basis vectors
                let x = projected_point_3d.dot(&u);
                let y = projected_point_3d.dot(&v);

                ProjectedPoint {
                    x,
                    y,
                    id: id as u32,
                }
            })
            .collect()
    }
}
