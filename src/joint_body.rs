use nalgebra::{Isometry3, Point3};
use parry3d::bounding_volume::{Aabb};
use parry3d::shape::{TriMesh};

/// Struct representing a joint body, which contains a `TriMesh` shape and local transformation, as well as a simplified version of the shape (as an AABB).
pub struct JointBody {
    /// The shape with the local_transform applied
    pub transformed_shape: TriMesh,
    /// The simplified triangular mesh (AABB as TriMesh) with local transform applied.
    pub simplified_shape: TriMesh,
}

impl JointBody {
    /// Constructor to initialize a joint body with a given mesh and local transform.
    /// This constructor applies the local transformation, computes the bounding volume, and converts it into a simplified triangular mesh.
    ///
    /// # Arguments
    /// * `shape` - A `TriMesh` that defines the joint's shape.
    /// * `local_transform` - The local transformation of the joint's shape.
    ///
    /// # Returns
    /// A new instance of `JointBody`.
    pub fn new(shape: TriMesh, local_transform: Isometry3<f32>) -> Self {
        // Apply the local transformation to the shape
        let transformed_shape =
            TriMesh::new(shape
                             .vertices()
                             .iter()
                             .map(|v|
                                 local_transform.transform_point(v))
                             .collect(),
                         shape.indices().to_vec());

        // Compute the axis aligned bounding box from the transformed shape
        let aabb = transformed_shape.qbvh().root_aabb().clone();

        // Return the new JointBody with the original, transformed, and simplified shapes
        JointBody {
            transformed_shape,
            simplified_shape: Self::aabb_to_trimesh(&aabb),
        }
    }

    /// Convert an AABB into a TriMesh (a triangular mesh).
    /// Unlike AABB, TriMesh will rotate following global transform later.
    ///
    /// # Arguments
    /// * `aabb` - The AABB to convert into a mesh.
    ///
    /// # Returns
    /// A `TriMesh` representing the AABB.
    fn aabb_to_trimesh(aabb: &Aabb) -> TriMesh {
        let mins = &aabb.mins;
        let maxs = &aabb.maxs;

        // Define the 8 vertices of the box (corners)
        let vertices = vec![
            // Bottom face
            Point3::new(mins.x, mins.y, mins.z), // 0: Bottom-left-back
            Point3::new(maxs.x, mins.y, mins.z), // 1: Bottom-right-back
            Point3::new(mins.x, maxs.y, mins.z), // 2: Top-left-back
            Point3::new(maxs.x, maxs.y, mins.z), // 3: Top-right-back
            // Top face
            Point3::new(mins.x, mins.y, maxs.z), // 4: Bottom-left-front
            Point3::new(maxs.x, mins.y, maxs.z), // 5: Bottom-right-front
            Point3::new(mins.x, maxs.y, maxs.z), // 6: Top-left-front
            Point3::new(maxs.x, maxs.y, maxs.z), // 7: Top-right-front
        ];

        // Define the 12 triangles (6 faces, 2 triangles per face)
        let indices = vec![
            // Back face
            [0, 1, 2], [1, 3, 2],
            // Front face
            [4, 5, 6], [5, 7, 6],
            // Left face
            [0, 2, 4], [2, 6, 4],
            // Right face
            [1, 5, 3], [5, 7, 3],
            // Top face
            [2, 3, 6], [3, 7, 6],
            // Bottom face
            [0, 1, 4], [1, 5, 4],
        ];

        // Create a TriMesh using the vertices and indices
        TriMesh::new(vertices, indices)
    }
}
