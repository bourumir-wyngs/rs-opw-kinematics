use nalgebra::{Isometry3};
use parry3d::shape::{TriMesh};


/// Optional structure attached to the robot base joint. It has its own global transform
/// that brings the robot to the location. This structure includes two transforms,
/// one bringing us to the base of the stand supporting the robot (and this is the 
/// pose of the stand itself), and then another defining the point where J1 rotating part
/// begins.
pub struct BaseBody {
    pub mesh: TriMesh,
    pub base_pose: Isometry3<f32>,
    pub robot_pose: Isometry3<f32>,
}


/// Static object against that we check the robot does not collide.
/// Unlike robot joint, it has the global transform allowing to place it
/// where desired.
pub struct CollisionBody {
    /// Mesh representing this collision object
    pub mesh: TriMesh,
    /// Global transform of this collision object.
    pub pose: Isometry3<f32>    
}

/// Pre-apply local transform for the mesh if needed. This may be needed
/// for the robot joint if it is defined in URDF with some local transform
pub fn transform_mesh(shape: &TriMesh, local_transform: &Isometry3<f32>) -> TriMesh {
    // Apply the local transformation to the shape
    TriMesh::new(shape
                     .vertices()
                     .iter()
                     .map(|v|
                         local_transform.transform_point(v))
                     .collect(),
                 shape.indices().to_vec())
}
