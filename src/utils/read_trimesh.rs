//! The functionality for loading Parry TriMesh from STL or PLY files, now deprecated.
//! Use our crate rs_read_trimesh that reads more formats and handles errors better

use parry3d::shape::TriMesh;
use rs_read_trimesh::load_trimesh;

fn fallback_trimesh() -> TriMesh {
    // Build a tiny valid triangle instead of panicking on malformed input.
    // This keeps the deprecated API panic-free for attacker-controlled files.
    TriMesh::new(
        vec![
            [0.0_f32, 0.0, 0.0].into(),
            [1.0e-6_f32, 0.0, 0.0].into(),
            [0.0_f32, 1.0e-6, 0.0].into(),
        ],
        vec![[0_u32, 1, 2].into()],
    )
    .expect("Failed to build fallback trimesh")
}

/// Function to load a TriMesh from a PLY file
#[deprecated(since = "1.8.4", note = "Use rs_read_trimesh::load_trimesh instead")]
pub fn load_trimesh_from_ply(ply_file_path: &str) -> TriMesh {
    match load_trimesh(ply_file_path, 1.0) {
        Ok(mesh) => mesh,
        Err(error) => {
            eprintln!("Could not load PLY mesh '{}': {error}", ply_file_path);
            fallback_trimesh()
        }
    }
}

/// Function to load a TriMesh from an STL file
#[deprecated(since = "1.8.4", note = "Use rs_read_trimesh::load_trimesh instead")]
pub fn load_trimesh_from_stl(stl_file_path: &str) -> TriMesh {
    match load_trimesh(stl_file_path, 1.0) {
        Ok(mesh) => mesh,
        Err(error) => {
            eprintln!("Could not load STL mesh '{}': {error}", stl_file_path);
            fallback_trimesh()
        }
    }
}
