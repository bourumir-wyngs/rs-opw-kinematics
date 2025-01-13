use crate::projector::ThreadSafeRayCast;
use nalgebra::{Isometry3, Point3, Quaternion, Unit, UnitQuaternion, Vector3};
use parry3d::bounding_volume::{Aabb, BoundingSphere};
use parry3d::mass_properties::MassProperties;
use parry3d::math::{Isometry, Point, Vector};
use parry3d::na;
use parry3d::query::{Contact, PointProjection, PointQuery, Ray, RayCast, RayIntersection};
use parry3d::shape::{FeatureId, HeightField, Shape, ShapeType, Triangle, TypedShape};

/// The depth field is Parry "turned height field", where there is a transform between the
/// outer world and the field world (Parry field only supports elevation, but with transform
/// we can work with depths produced by the camera in any orientation).
pub struct DepthField {
    /// Transform from an outer world to an inner world.
    pub to_inner: Isometry3<f32>,

    /// Transform from an inner world to an outer world.
    pub to_outer: Isometry3<f32>,

    /// The height field that accepts the transformed input.
    field: HeightField,
}

pub enum FieldOrientation {
    /// Just a height field
    Z,
    /// Z axis in the inner world is the same as -Z axis in the outer world.
    AntiZ,
    /// Z axis in the inner world is the same as X axis in the outer world.
    X,
    /// Z axis in the inner world is the same as -X axis in the outer world.
    AntiXm,
    /// Z axis in the inner world is the same as Y axis in the outer world.
    Y,
    /// Z axis in the inner world is the same as -Y axis in the outer world.
    AntiY,
}

impl FieldOrientation {
    pub fn to_inner_isometry3(&self) -> Isometry3<f32> {
        match self {
            FieldOrientation::Z => Isometry3::identity(),
            FieldOrientation::AntiZ => Isometry3::face_towards(
                &Point3::origin(),
                &Point3::new(0.0, 0.0, -1.0),
                &Vector3::new(0.0, 1.0, 0.0),
            ),
            FieldOrientation::X => Isometry3::face_towards(
                &Point3::origin(),
                &Point3::new(1.0, 0.0, 0.0),
                &Vector3::new(0.0, 1.0, 0.0),
            ),
            FieldOrientation::AntiXm => Isometry3::face_towards(
                &Point3::origin(),
                &Point3::new(-1.0, 0.0, 0.0),
                &Vector3::new(0.0, 1.0, 0.0),
            ),
            FieldOrientation::Y => Isometry3::face_towards(
                &Point3::origin(),
                &Point3::new(0.0, 1.0, 0.0),
                &Vector3::new(0.0, 0.0, 1.0),
            ),
            FieldOrientation::AntiY => Isometry3::face_towards(
                &Point3::origin(),
                &Point3::new(0.0, -1.0, 0.0),
                &Vector3::new(0.0, 0.0, 1.0),
            ),
        }
    }
}

impl DepthField {
    /// Creates a new `DepthField`.
    ///
    /// # Arguments
    /// - `heights`: A 2D matrix of heights for the depth field.
    /// - `scale`: The scale factors to apply to the field.
    /// - `to_inner`: The isometry3 (transformation) used to convert from outer space to inner space.
    ///
    /// # Returns
    /// A new `DepthField` instance.
    pub fn new(heights: na::DMatrix<f32>, scale: Vector<f32>, to_inner: Isometry3<f32>) -> Self {
        let field = HeightField::new(heights, scale);
        let to_outer = to_inner.inverse();

        Self {
            to_inner,
            to_outer,
            field,
        }
    }
}

/// Implement RayCast for DepthField. The Ray is first transformed into "inner" form.
impl RayCast for DepthField {
    fn cast_local_ray(&self, ray: &Ray, max_time_of_impact: f32, solid: bool) -> Option<f32> {
        let ray = &ray.transform_by(&self.to_inner);
        self.field.cast_local_ray(ray, max_time_of_impact, solid)
    }

    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray,
        max_time_of_impact: f32,
        solid: bool,
    ) -> Option<RayIntersection> {
        let ray = &ray.transform_by(&self.to_inner);
        self.field
            .cast_local_ray_and_get_normal(ray, max_time_of_impact, solid)
    }

    fn intersects_local_ray(&self, ray: &Ray, max_time_of_impact: f32) -> bool {
        let ray = &ray.transform_by(&self.to_inner);
        self.field.intersects_local_ray(ray, max_time_of_impact)
    }

    fn cast_ray(
        &self,
        m: &Isometry<f32>,
        ray: &Ray,
        max_time_of_impact: f32,
        solid: bool,
    ) -> Option<f32> {
        let ray = &ray.transform_by(&self.to_inner);
        self.field.cast_ray(m, ray, max_time_of_impact, solid)
    }

    fn cast_ray_and_get_normal(
        &self,
        m: &Isometry<f32>,
        ray: &Ray,
        max_time_of_impact: f32,
        solid: bool,
    ) -> Option<RayIntersection> {
        let ray = &ray.transform_by(&self.to_inner);
        self.field
            .cast_ray_and_get_normal(m, ray, max_time_of_impact, solid)
    }

    fn intersects_ray(&self, m: &Isometry<f32>, ray: &Ray, max_time_of_impact: f32) -> bool {
        let ray = &ray.transform_by(&self.to_inner);
        self.field.intersects_ray(m, ray, max_time_of_impact)
    }
}

impl PointQuery for DepthField {
    /// Delegates `project_local_point` to the inner field, applying `to_inner` and `to_outer`
    fn project_local_point(&self, pt: &Point<f32>, solid: bool) -> PointProjection {
        let transformed_point = self.to_inner.transform_point(pt); // Apply to_inner to input
        let projection = self.field.project_local_point(&transformed_point, solid); // Delegate work
        PointProjection {
            point: self.to_outer.transform_point(&projection.point), // Apply to_outer to output
            is_inside: projection.is_inside,
        }
    }

    /// Delegates `project_local_point_and_get_feature` to the inner field, applying `to_inner` and `to_outer`
    fn project_local_point_and_get_feature(
        &self,
        pt: &Point<f32>,
    ) -> (PointProjection, FeatureId) {
        let transformed_point = self.to_inner.transform_point(pt); // Apply to_inner to input
        let (projection, feature) = self.field.project_local_point_and_get_feature(&transformed_point); // Delegate work
        (
            PointProjection {
                point: self.to_outer.transform_point(&projection.point), // Apply to_outer to output
                is_inside: projection.is_inside,
            },
            feature,
        )
    }
}

impl Shape for DepthField {
    /// Delegates `compute_local_aabb`, applying `to_outer` to the result
    fn compute_local_aabb(&self) -> Aabb {
        let inner_aabb = self.field.compute_local_aabb(); // Compute inner AABB
        inner_aabb.transform_by(&self.to_outer) // Apply to_outer to AABB
    }

    /// Delegates `compute_local_bounding_sphere`, applying `to_outer` to the result
    fn compute_local_bounding_sphere(&self) -> BoundingSphere {
        let inner_sphere = self.field.compute_local_bounding_sphere(); // Compute inner bounding sphere
        BoundingSphere {
            center: self.to_outer.transform_point(&inner_sphere.center), // Apply to_outer to center
            radius: inner_sphere.radius, // Radius is unchanged
        }
    }

    /// Delegates `clone_dyn`
    fn clone_dyn(&self) -> Box<dyn Shape> {
        Box::new(DepthField {
            field: self.field.clone(), // Delegate to clone_dyn for the inner field
            to_inner: self.to_inner,
            to_outer: self.to_outer,
        })
    }

    /// Delegates `scale_dyn`
    fn scale_dyn(&self, scale: &Vector<f32>, num_subdivisions: u32) -> Option<Box<dyn Shape>> {
        self.field
            .scale_dyn(scale, num_subdivisions)
            .and_then(|scaled_field| scaled_field.downcast::<HeightField>().ok()) // Attempt downcast
            .map(|scaled_height_field| {
                // Return as Box<dyn Shape>
                Box::new(DepthField {
                    field: *scaled_height_field, // Extract HeightField
                    to_inner: self.to_inner,
                    to_outer: self.to_outer,
                }) as Box<dyn Shape> // Explicit coercion to trait object
            })
    }

    /// Delegates `mass_properties`
    fn mass_properties(&self, density: f32) -> MassProperties {
        self.field.mass_properties(density)
    }

    /// Delegates `shape_type`
    fn shape_type(&self) -> ShapeType {
        ShapeType::Custom
    }

    /// Delegates `as_typed_shape`
    fn as_typed_shape(&self) -> TypedShape {
        TypedShape::Custom(self)
    }

    /// Delegates `ccd_thickness`
    fn ccd_thickness(&self) -> f32 {
        self.field.ccd_thickness()
    }

    /// Delegates `ccd_angular_thickness`
    fn ccd_angular_thickness(&self) -> f32 {
        self.field.ccd_angular_thickness()
    }
}

mod tests {
    #[test]
    fn test_new() {}
}
