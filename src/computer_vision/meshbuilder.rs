use std::collections::HashMap;
use parry3d::shape::TriMesh;
use crate::organized_point::OrganizedPoint;

pub fn construct_parry_trimesh_sequential(points: Vec<OrganizedPoint>) -> TriMesh {
    // Map grid (row, col) -> index in the vertices list
    let mut grid = HashMap::with_capacity(points.len());
    let mut vertices = Vec::with_capacity(points.len());

    // Step 1: Insert all points into grid and build the vertices list
    for (index, point) in points.iter().enumerate() {
        grid.insert((point.row, point.col), index as u32);
        vertices.push(point.point);
    }

    let mut indices = Vec::new();

    // Helper function to add a triangle if all vertices exist
    let mut add_triangle = |v0: Option<&u32>, v1: Option<&u32>, v2: Option<&u32>| {
        if v0.is_some() && v1.is_some() && v2.is_some() {
            indices.push([*v0.unwrap(), *v1.unwrap(), *v2.unwrap()]);
        }
    };

    // Step 2: Form triangles based on adjacency in the grid
    for point in points.iter() {
        let row = point.row;
        let col = point.col;

        // Primary triangles for the current cell
        let v0 = grid.get(&(row, col));
        let v1 = grid.get(&(row, col + 1));
        let v2 = grid.get(&(row + 1, col));

        add_triangle(v0, v1, v2); // Primary triangle (v0 -> v1 -> v2)

        let v3 = grid.get(&(row + 1, col + 1));

        add_triangle(v1, v3, v2); // Primary triangle (v1 -> v3 -> v2)

        // Fallback: Neighbor-based checks if primary triangles are not possible
        if v0.is_none() || v1.is_none() || v2.is_none() {
            let alt_v1 = grid.get(&(row, col - 1)); // Left neighbor
            let alt_v2 = grid.get(&(row - 1, col)); // Top neighbor

            add_triangle(v0, alt_v1, alt_v2); // Alternate triangle using top-left corner
        }
        if v1.is_none() || v2.is_none() || v3.is_none() {
            let alt_v3 = grid.get(&(row + 2, col + 1)); // Two rows below
            let alt_v1 = grid.get(&(row, col - 1));     // Left neighbor

            add_triangle(v3, alt_v1, alt_v3); // Alternate triangle for diagonal recovery
        }
    }

    // Step 3: Construct the TriMesh
    TriMesh::new(vertices, indices)
}


pub fn construct_parry_trimesh(points: Vec<OrganizedPoint>) -> TriMesh {
    use rayon::prelude::*;

    fn add_triangle(
        v0: Option<&u32>,
        v1: Option<&u32>,
        v2: Option<&u32>,
        triangles: &mut Vec<[u32; 3]>,
    ) {
        if v0.is_some() && v1.is_some() && v2.is_some() {
            triangles.push([*v0.unwrap(), *v1.unwrap(), *v2.unwrap()]);
        }
    }    
    
    // Step 1: Build the grid and vertices (sequential)
    let mut grid = HashMap::with_capacity(points.len());
    let mut vertices = Vec::with_capacity(points.len());

    for (index, point) in points.iter().enumerate() {
        grid.insert((point.row, point.col), index as u32);
        vertices.push(point.point);
    }

    // Step 2: Parallel triangle creation using per-thread buffers
    let thread_triangle_vectors: Vec<Vec<[u32; 3]>> = points
        .par_iter() // Parallel iteration over points
        .map(|point| {
            let row = point.row;
            let col = point.col;
            let mut local_triangles = Vec::new();

            // Generate primary triangles
            let v0 = grid.get(&(row, col));
            let v1 = grid.get(&(row, col + 1));
            let v2 = grid.get(&(row + 1, col));
            add_triangle(v0, v1, v2, &mut local_triangles);

            let v3 = grid.get(&(row + 1, col + 1));
            add_triangle(v1, v3, v2, &mut local_triangles);

            // Generate fallback triangles
            if v0.is_none() || v1.is_none() || v2.is_none() {
                let alt_v1 = grid.get(&(row, col - 1)); // Left neighbor
                let alt_v2 = grid.get(&(row - 1, col)); // Top neighbor
                add_triangle(v0, alt_v1, alt_v2, &mut local_triangles);
            }

            if v1.is_none() || v2.is_none() || v3.is_none() {
                let alt_v3 = grid.get(&(row + 2, col + 1)); // Two rows below
                let alt_v1 = grid.get(&(row, col - 1)); // Left neighbor
                add_triangle(v3, alt_v1, alt_v3, &mut local_triangles);
            }

            local_triangles // Return thread-local triangles
        })
        .collect(); // Aggregate all thread-local triangle vectors

    // Step 3: Merge all thread-local triangle vectors
    let indices: Vec<[u32; 3]> = thread_triangle_vectors.into_iter().flatten().collect();

    // Step 4: Construct the final TriMesh
    TriMesh::new(vertices, indices)
}

