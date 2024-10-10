use nalgebra::{Isometry3, Point3};
use parry3d::bounding_volume::{Aabb, BoundingVolume};
use parry3d::shape::{SharedShape, TriMesh};

/// Struct representing a shape (using TriMesh here, but it can be other shapes)
pub struct CollisionShape {
    pub name: String,
    pub local_transform: Isometry3<f32>,
    pub shape: SharedShape,
}

/// Struct representing a joint, which contains multiple shapes and a simplified version of the joint's shape (as an AABB).
pub struct JointBody {
    pub shapes: Vec<CollisionShape>,
    pub simplified_shape: TriMesh, // The simplified shape (AABB as TriMesh)
}

impl JointBody {
    /// Constructor to initialize a joint with a given list of collision shapes.
    /// The constructor also computes an AABB around all the shapes and creates a simplified triangular mesh.
    ///
    /// # Arguments
    /// * `shapes` - A vector of `CollisionShape`s that belong to the joint.
    ///
    /// # Returns
    /// A new instance of `JointBody`.
    pub fn new(shapes: Vec<CollisionShape>) -> Self {
        // Compute the AABB for the entire joint
        let aabb = Self::compute_aabb(&shapes);

        // Return the new JointBody with the simplified shape assigned
        JointBody {
            shapes,
            simplified_shape: Self::aabb_to_trimesh(&aabb),
        }
    }

    /// Compute the AABB (Axis-Aligned Bounding Box) that surrounds all the shapes in the joint.
    ///
    /// # Arguments
    /// * `shapes` - A reference to the vector of shapes to compute the bounding box from.
    ///
    /// # Returns
    /// An `Aabb` that fits all the shapes in the joint.
    fn compute_aabb(shapes: &[CollisionShape]) -> Aabb {
        let mut overall_aabb = Aabb::new_invalid();

        // Loop over each shape and expand the AABB to include its local bounding box
        for shape in shapes {
            // Cast the Isometry3<f64> to Isometry3<f32> for parry3d
            let local_aabb = shape.shape.compute_aabb(&shape.local_transform.cast::<f32>());
            overall_aabb.merge(&local_aabb);
        }

        overall_aabb
    }

    /// Convert an AABB into a TriMesh (a triangular mesh).
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

