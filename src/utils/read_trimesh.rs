//! The functionality for loading Parry TriMesh from STL or PLY files, now deprecated.
//! Use our crate rs_read_trimesh that reads more formats and handles errors better

use parry3d::shape::{TriMesh};
use rs_read_trimesh::load_trimesh;

/// Function to load a TriMesh from a PLY file
#[deprecated(since = "1.8.4", note = "Use rs_read_trimesh::load_trimesh instead")]
pub fn load_trimesh_from_ply(ply_file_path: &str) -> TriMesh {
    load_trimesh(ply_file_path, 1.0).expect("Could not load mesh")
}

/// Function to load a TriMesh from an STL file
#[deprecated(since = "1.8.4", note = "Use rs_read_trimesh::load_trimesh instead")]
pub fn load_trimesh_from_stl(stl_file_path: &str) -> TriMesh {
    load_trimesh(stl_file_path, 1.0).expect("Could not load mesh")    
}
