use parry3d::math::{Point, Vector};
use parry3d::na;
use parry3d::query::{PointProjection, PointQuery, Ray, RayCast, RayIntersection};
use parry3d::shape::{FeatureId, HeightField};

#[derive(Debug)]
pub enum FieldOrientation {
    /// Just a height field
    Z,
    /// Z axis in the inner world is the same as -Z axis in the outer world.
    AntiZ,
    /// Z axis in the inner world is the same as X axis in the outer world.
    X,
    /// Z axis in the inner world is the same as -X axis in the outer world.
    AntiX,
    /// Z axis in the inner world is the same as Y axis in the outer world.
    Y,
    /// Z axis in the inner world is the same as -Y axis in the outer world.
    AntiY,
}

/// The depth field is Parry "turned height field", where there is a transform between the
/// outer world and the field world (Parry field only supports elevation, but with transform
/// we can work with depths produced by the camera in any orientation).
pub struct DepthField {
    /// The orientation of the depth field.
    orientation: FieldOrientation,

    /// The height field that accepts the transformed input.
    /// HeightField internally assumes Y as elevation axis!!!!
    field: HeightField,
}

impl FieldOrientation {
    /// Convert from outer coordinate to the system used by the HeightField
    fn to_inner(&self, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
        match self {
            FieldOrientation::Z => (x, z, y),
            FieldOrientation::AntiZ => (x, -z, y),
            FieldOrientation::X => (y, x, z),
            FieldOrientation::AntiX => (y, -x, z),
            FieldOrientation::Y => (x, y, z),
            FieldOrientation::AntiY => (x, -y, z),
        }
    }

    fn to_outer(&self, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
        match self {
            FieldOrientation::Z => (x, z, y),
            FieldOrientation::AntiZ => (x, z, -y),
            FieldOrientation::X => (y, x, z),
            FieldOrientation::AntiX => (-y, x, z),
            FieldOrientation::Y => (x, y, z),
            FieldOrientation::AntiY => (x, -y, z),
        }
    }

    fn to_inner_point(&self, pt: &Point<f32>) -> Point<f32> {
        let (x, y, z) = self.to_inner(pt.x, pt.y, pt.z);
        Point::new(x, y, z)
    }

    fn to_outer_point(&self, pt: &Point<f32>) -> Point<f32> {
        let (x, y, z) = self.to_outer(pt.x, pt.y, pt.z);
        Point::new(x, y, z)
    }

    fn to_outer_projection(&self, from: PointProjection) -> PointProjection {
        PointProjection {
            is_inside: from.is_inside,
            point: self.to_outer_point(&from.point),
        }
    }
}

impl DepthField {
    /// Creates a new `DepthField`.
    ///
    /// # Arguments
    /// - `heights`: A 2D matrix of heights for the depth field, oriented in the space as
    /// specified in the orientation.
    /// - `scale`: The scale factors to apply to the field.
    /// - `orientation`: The orientation of the field in space.
    ///
    /// # Returns
    /// A new `DepthField` instance.
    pub fn new(
        heights: na::DMatrix<f32>,
        scale: Vector<f32>,
        orientation: FieldOrientation,
    ) -> Self {
        Self {
            orientation,
            field: HeightField::new(heights, scale),
        }
    }

    fn transform_ray(&self, ray: &&Ray) -> Ray {
        let t_origin = self.orientation.to_inner_point(&ray.origin);
        let (vx, vy, vz) = self.orientation.to_inner(ray.dir.x, ray.dir.y, ray.dir.z);
        let t_ray = Ray { origin: t_origin, dir: Vector::new(vx, vy, vz) };
        t_ray
    }
}

impl PointQuery for DepthField {
    fn project_local_point(&self, pt: &Point<f32>, solid: bool) -> PointProjection {
        self.orientation.to_outer_projection(
            self.field
                .project_local_point(&self.orientation.to_inner_point(pt), solid),
        )
    }

    fn project_local_point_and_get_feature(&self, pt: &Point<f32>) -> (PointProjection, FeatureId) {
        let (projection, feature) = self
            .field
            .project_local_point_and_get_feature(&self.orientation.to_inner_point(pt));
        (self.orientation.to_outer_projection(projection), feature)
    }
}

impl RayCast for DepthField {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray,
        max_time_of_impact: f32,
        solid: bool,
    ) -> Option<RayIntersection> {
        let intersection = self.field.cast_local_ray_and_get_normal(
            &self.transform_ray(&ray), max_time_of_impact, solid);
        if let Some(intersection) = intersection {
            let n = intersection.normal;
            let (nx, ny, nz) = self.orientation.to_outer(n.x, n.y, n.z);
            Some(RayIntersection {
                time_of_impact: intersection.time_of_impact,
                normal: Vector::new(nx, ny, nz),
                feature: intersection.feature,
            })
        } else {
            None
        }
    }

    fn cast_local_ray(&self, ray: &Ray, max_time_of_impact: f32, solid: bool) -> Option<f32> {
        // No need to transform the normal.
        self.field.cast_local_ray(&self.transform_ray(&ray), max_time_of_impact, solid)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_inner_to_outer_reversibility() {
        let x = 1.0;
        let y = 2.0;
        let z = 3.0;

        let orientations = [
            FieldOrientation::Z,
            FieldOrientation::AntiZ,
            FieldOrientation::X,
            FieldOrientation::AntiX,
            FieldOrientation::Y,
            FieldOrientation::AntiY,
        ];

        for orientation in orientations.iter() {
            // to_inner and then to_outer should return the original values
            let inner = orientation.to_inner(x, y, z);
            let outer = orientation.to_outer(inner.0, inner.1, inner.2);
            assert_eq!((x, y, z), outer, "Failed for {:?}", orientation);

            // to_outer and then to_inner should also return the original values
            let outer = orientation.to_outer(x, y, z);
            let inner = orientation.to_inner(outer.0, outer.1, outer.2);
            assert_eq!((x, y, z), inner, "Failed for {:?}", orientation);
        }
    }
}
