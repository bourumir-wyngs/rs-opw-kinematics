use crate::organized_point::OrganizedPoint;
use parry3d::shape::TriMesh;
use std::collections::HashMap;

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
            let alt_v1 = grid.get(&(row, col - 1)); // Left neighbor

            add_triangle(v3, alt_v1, alt_v3); // Alternate triangle for diagonal recovery
        }
    }

    // Step 3: Construct the TriMesh
    TriMesh::new(vertices, indices)
}

pub fn construct_parry_trimesh(points: Vec<OrganizedPoint>) -> TriMesh {
    use rayon::prelude::*;

    // Step 1: Build the grid and vertices (sequential)
    let mut grid = HashMap::with_capacity(points.len());
    let mut vertices = Vec::with_capacity(points.len());

    for (index, point) in points.iter().enumerate() {
        grid.insert((point.row as i32, point.col as i32), index as u32);
        vertices.push(point.point);
    }

    let thread_triangle_vectors: Vec<Vec<[u32; 3]>> = points
        .par_iter()
        .enumerate() // Access point with its index
        .map(|(index, point)| {
            let mut local_triangles = Vec::with_capacity(2);

            let row = point.row as i32;
            let col = point.col as i32;

            // Current point
            let v = index as u32; // Current point is the `index`

            // This will not work for the bottom edge row but there are one less gaps between 
            // points than points, so all triangles will be generated.
            if let Some(&v_uf) = grid.get(&(row + 1, col + 1)) {
                if let Some(&v_u) = grid.get(&(row + 1, col)) {
                    local_triangles.push([v_uf, v_u, v]);
                }

                if let Some(&v_f) = grid.get(&(row, col + 1)) {
                    local_triangles.push([v, v_f, v_uf]);
                }
            } else {
                // If v_uf not available, one triangle still can be made trough v_u and v_f. 
                // Otherwise will not be any
                if let (Some(&v_u), Some(&v_f)) =
                    (grid.get(&(row + 1, col)), grid.get(&(row, col + 1)))
                {
                    local_triangles.push([v_f, v_u, v]);
                }
            }
            local_triangles
        })
        .collect();

    let indices: Vec<[u32; 3]> = thread_triangle_vectors.into_iter().flatten().collect();
    TriMesh::new(vertices, indices)
}
