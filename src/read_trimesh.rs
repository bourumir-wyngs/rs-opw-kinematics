//! Functionality for loading Parry TriMesh from STL or PLY files.

use parry3d::shape::{TriMesh};
use stl_io::{read_stl};
use std::fs::File;
use std::io::BufReader;
use nalgebra::Point3;

/// Function to load a TriMesh from an STL file
pub fn load_trimesh_from_stl(stl_file_path: &str) -> TriMesh {
    // Open the STL file
    let file = File::open(stl_file_path)
        .expect(&format!("Could not open STL file: {}", stl_file_path));
    let mut reader = BufReader::new(file);

    // Read the STL file into IndexedMesh
    let stl = read_stl(&mut reader)
        .expect(&format!("Could not open STL file: {}", stl_file_path));

    // Extract vertices and convert them to Point3<f32>
    let vertices: Vec<Point3<f32>> = stl
        .vertices
        .into_iter()
        .map(|vertex| Point3::new(vertex[0], vertex[1], vertex[2]))
        .collect();

    // Convert face indices from `usize` to `u32`
    let indices: Vec<[u32; 3]> = stl
        .faces
        .into_iter()
        .map(|face| [
            face.vertices[0] as u32,
            face.vertices[1] as u32,
            face.vertices[2] as u32,
        ])
        .collect();

    TriMesh::new(vertices, indices)
}

