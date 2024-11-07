//! Functionality for loading Parry TriMesh from STL or PLY files.

use parry3d::shape::{TriMesh, TriMeshFlags};
use ply_rs::parser::Parser;
use ply_rs::ply::{DefaultElement, Property};
use stl_io::{read_stl};
use std::fs::File;
use std::io::BufReader;
use nalgebra::Point3;

/// Function to load a TriMesh from a PLY file
#[allow(dead_code)]
pub fn load_trimesh_from_ply(ply_file_path: &str) -> TriMesh {
    // Open the file
    let file = File::open(ply_file_path)
        .expect(&format!("Could not open PLY file: {}", ply_file_path));
    let mut reader = BufReader::new(file);

    // Create a PLY parser and parse the header
    let parser = Parser::<DefaultElement>::new();
    let ply = parser.read_ply(&mut reader)
        .expect(&format!("Could not parse PLY file: {}", ply_file_path));

    // Extract vertices and faces from the PLY file
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Extract vertices
    if let Some(vertices_elem) = ply.payload.get("vertex") {
        for vertex in vertices_elem {
            let x = match vertex.get("x").unwrap() {
                Property::Float(val) => *val,
                Property::Double(val) => *val as f32,
                _ => panic!("Unexpected type for vertex x"),
            };
            let y = match vertex.get("y").unwrap() {
                Property::Float(val) => *val,
                Property::Double(val) => *val as f32,
                _ => panic!("Unexpected type for vertex y"),
            };
            let z = match vertex.get("z").unwrap() {
                Property::Float(val) => *val,
                Property::Double(val) => *val as f32,
                _ => panic!("Unexpected type for vertex z"),
            };

            vertices.push([x, y, z].into()); // Push vertex to the vertices list
        }
    }

    // Extract faces (indices)
    if let Some(faces_elem) = ply.payload.get("face") {
        for face in faces_elem {
            if let Property::ListUInt(indices_list) = face.get("vertex_indices").unwrap() {
                let i1 = indices_list[0];
                let i2 = indices_list[1];
                let i3 = indices_list[2];
                indices.push([i1, i2, i3].into()); // Push triangle indices
            } else {
                panic!("Unexpected type for face indices");
            }
        }
    }

    // Create a TriMesh from vertices and indices
    TriMesh::with_flags(vertices, indices, 
                        TriMeshFlags::FIX_INTERNAL_EDGES | TriMeshFlags::MERGE_DUPLICATE_VERTICES)
}

/// Function to load a TriMesh from an STL file
#[allow(dead_code)] 
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

