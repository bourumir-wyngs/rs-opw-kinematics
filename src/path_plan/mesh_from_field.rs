use nalgebra::{Isometry3, Point3};
use parry3d::shape::TriMesh;
use std::ops::Range;

/// Represents a camera with intrinsic parameters and pose.
pub struct Camera {
    pub focal_length_x: f32,    // Focal length in the x direction
    pub focal_length_y: f32,    // Focal length in the y direction
    pub cx: f32,               // Principal point x
    pub cy: f32,               // Principal point y
    pub pose: Isometry3<f32>,   // Camera pose (extrinsics)
    pub depth_range: Range<f32>, // Valid depth range
}

impl Camera {
    /// Reconstructs a `TriMesh` from a depth field using the camera parameters.
    pub fn depth_field_to_trimesh(
        &self,
        depth_field: &[f32], // Depth field as a 1D array
        width: usize,        // Width of the depth field
        height: usize,       // Height of the depth field
    ) -> TriMesh {
        // Predict the number of vertices and indices
        let num_vertices = width * height;
        let num_triangles = (width - 1) * (height - 1) * 2;
        let num_indices = num_triangles * 3;

        // Use `Vec::with_capacity` for efficient memory allocation
        let mut vertices = Vec::with_capacity(num_vertices);
        let mut indices = Vec::with_capacity(num_indices);

        // Allocate a 2D array to track valid vertex indices (initialized with None)
        let mut valid_points = vec![None; width * height]; // Flattened 2D array

        // Step 1: Back-project depth values to 3D points
        for v in 0..height {
            for u in 0..width {
                let z = depth_field[v * width + u];

                // Skip invalid depth values (NaN or outside range)
                if !z.is_finite() || !self.depth_range.contains(&z) {
                    continue;
                }

                // Camera coordinates
                let x_c = (u as f32 - self.cx) * z / self.focal_length_x;
                let y_c = (v as f32 - self.cy) * z / self.focal_length_y;
                let point_camera = Point3::new(x_c, y_c, z);

                // Transform to world coordinates
                let point_world = self.pose.transform_point(&point_camera);

                // Push the valid vertex and store its index
                valid_points[v * width + u] = Some(vertices.len()); // Mark this grid point as valid
                vertices.push(point_world);
            }
        }

        for v in 0..(height - 1) {
            for u in 0..(width - 1) {
                // Collect the 4 vertices of the current grid cell
                let top_left = valid_points[v * width + u];
                let top_right = valid_points[v * width + (u + 1)];
                let bottom_left = valid_points[(v + 1) * width + u];
                let bottom_right = valid_points[(v + 1) * width + (u + 1)];

                // Check all combinations of 3 valid vertices to form a single triangle
                match (
                    top_left,
                    top_right,
                    bottom_left,
                    bottom_right,
                ) {
                    // Case 1: All 4 vertices valid - form 2 triangles
                    (
                        Some(top_left),
                        Some(top_right),
                        Some(bottom_left),
                        Some(bottom_right),
                    ) => {
                        indices.push([top_left as u32, top_right as u32, bottom_left as u32]);
                        indices.push([top_right as u32, bottom_right as u32, bottom_left as u32]);
                    }

                    // Case 2: Exactly 3 valid vertices
                    (
                        Some(top_left),
                        Some(top_right),
                        Some(bottom_left),
                        None,
                    ) => {
                        indices.push([top_left as u32, top_right as u32, bottom_left as u32]);
                    }

                    (
                        Some(top_left),
                        Some(top_right),
                        None,
                        Some(bottom_right),
                    ) => {
                        indices.push([top_left as u32, top_right as u32, bottom_right as u32]);
                    }

                    (
                        Some(top_left),
                        None,
                        Some(bottom_left),
                        Some(bottom_right),
                    ) => {
                        indices.push([top_left as u32, bottom_right as u32, bottom_left as u32]);
                    }

                    (
                        None,
                        Some(top_right),
                        Some(bottom_left),
                        Some(bottom_right),
                    ) => {
                        // Triangle from top_right, bottom_right, bottom_left
                        indices.push([top_right as u32, bottom_right as u32, bottom_left as u32]);
                    }

                    // Other cases: Fewer than 3 valid vertices (skip the cell)
                    _ => {}
                }
            }
        }
        // Step 3: Create a Parry TriMesh
        TriMesh::new(vertices, indices)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FCL: f32 = 10.0; // Focal length

    #[test]
    fn test_depth_field_2x2() {
        let camera = Camera {
            focal_length_x: FCL,
            focal_length_y: FCL,
            cx: 0.5, // Principal point centered in 2x2 grid
            cy: 0.5,
            depth_range: 0.0..100.0,
            pose: Isometry3::identity(),
        };

        // 2x2 depth field 
        let depth_field = vec![
            1.0,  2.0, 
            3.0,  4.0, 
        ];
        
        let width = 2;
        let height = 2;

        // Generate the mesh
        let trimesh = camera.depth_field_to_trimesh(&depth_field, width, height);

        // Check the vertices and ensure proper alignment with principal point
        let vertices = trimesh.vertices();
        assert_eq!(
            vertices[0],
            Point3::new(
                (0.0 - camera.cx) * 1.0 / camera.focal_length_x,  // x-coordinate Top-left
                (0.0 - camera.cy) * 1.0 / camera.focal_length_y,  // y-coordinate Top-left
                1.0                                              // z-coordinate
            )
        ); // Top-left

        assert_eq!(
            vertices[1],
            Point3::new(
                (1.0 - camera.cx) * 2.0 / camera.focal_length_x,  // x-coordinate Top-right
                (0.0 - camera.cy) * 2.0 / camera.focal_length_y,  // y-coordinate Top-right
                2.0                                              // z-coordinate
            )
        ); // Top-right

        assert_eq!(
            vertices[2],
            Point3::new(
                (0.0 - camera.cx) * 3.0 / camera.focal_length_x,  // x-coordinate Bottom-left
                (1.0 - camera.cy) * 3.0 / camera.focal_length_y,  // y-coordinate Bottom-left
                3.0                                              // z-coordinate
            )
        ); // Bottom-left

        assert_eq!(
            vertices[3],
            Point3::new(
                (1.0 - camera.cx) * 4.0 / camera.focal_length_x,  // x-coordinate Bottom-right
                (1.0 - camera.cy) * 4.0 / camera.focal_length_y,  // y-coordinate Bottom-right
                4.0                                              // z-coordinate
            )
        ); // Bottom-right

        // Check the number of vertices and indices
        assert_eq!(trimesh.vertices().len(), 4); // 2x2 grid
        assert_eq!(trimesh.indices().len(), 2); // 2 triangles, 6 indices total        
    }

    #[test]
    fn test_depth_field_with_partial_grid() {
        let camera = Camera {
            focal_length_x: FCL,
            focal_length_y: FCL,
            cx: 0.5, // Principal point centered in 2x2 grid
            cy: 0.5,
            depth_range: 0.0..100.0, // Valid depth range
            pose: Isometry3::identity(),
        };

        // A 2x2 grid with three valid points (one invalid point: -1.0)
        let depth_field = vec![
            1.0,  -1.0, // Top row: Valid, Invalid
            3.0,  2.0,  // Bottom row: Valid, Valid
        ];
        let width = 2;
        let height = 2;

        // Generate the mesh
        let trimesh = camera.depth_field_to_trimesh(&depth_field, width, height);

        // Check the vertices
        let vertices = trimesh.vertices();
        assert_eq!(vertices.len(), 3); // Only three valid points present

        // Ensure the vertices are projected correctly
        assert_eq!(
            vertices[0],
            Point3::new(
                (0.0 - camera.cx) * 1.0 / camera.focal_length_x,  // x-coordinate Top-left
                (0.0 - camera.cy) * 1.0 / camera.focal_length_y,  // y-coordinate Top-left
                1.0                                              // z-coordinate
            )
        ); // Top-left

        assert_eq!(
            vertices[1],
            Point3::new(
                (0.0 - camera.cx) * 3.0 / camera.focal_length_x,  // x-coordinate Bottom-left
                (1.0 - camera.cy) * 3.0 / camera.focal_length_y,  // y-coordinate Bottom-left
                3.0                                              // z-coordinate
            )
        ); // Bottom-left

        assert_eq!(
            vertices[2],
            Point3::new(
                (1.0 - camera.cx) * 2.0 / camera.focal_length_x,  // x-coordinate Bottom-right
                (1.0 - camera.cy) * 2.0 / camera.focal_length_y,  // y-coordinate Bottom-right
                2.0                                              // z-coordinate
            )
        ); // Bottom-right

        // Check the triangle indices
        let indices = trimesh.indices();
        assert_eq!(indices.len(), 1); // Only one triangle
        assert_eq!(indices[0], [0, 2, 1]); // Triangle formed by Top-left, Bottom-right, Bottom-left
    }

    #[test]
    fn test_depth_field_with_translation() {
        const TX: f32 = 10.0; // Translation along x-axis
        const TY: f32 = 20.0; // Translation along y-axis
        const TZ: f32 = 30.0; // Translation along z-axis

        let camera = Camera {
            focal_length_x: FCL,
            focal_length_y: FCL,
            cx: 0.5, // Principal point centered in 2x2 grid
            cy: 0.5,
            depth_range: 0.0..100.0,
            pose: Isometry3::new([TX, TY, TZ].into(), [0.0; 3].into()), // Camera translation (no rotation)
        };

        // 2x2 depth field
        let depth_field = vec![
            1.0, 2.0, // Top row
            3.0, 4.0, // Bottom row
        ];
        let width = 2;
        let height = 2;

        // Generate the mesh
        let trimesh = camera.depth_field_to_trimesh(&depth_field, width, height);

        // Check the vertices
        let vertices = trimesh.vertices();
        assert_eq!(vertices.len(), 4); // Four points in the grid, all valid.

        // Ensure the vertices are projected correctly, including translation (TX, TY, TZ)
        assert_eq!(
            vertices[0],
            Point3::new(
                (0.0 - camera.cx) * 1.0 / camera.focal_length_x + TX,  // x-coordinate Top-left
                (0.0 - camera.cy) * 1.0 / camera.focal_length_y + TY,  // y-coordinate Top-left
                1.0 + TZ                                               // z-coordinate
            )
        ); // Top-left

        assert_eq!(
            vertices[1],
            Point3::new(
                (1.0 - camera.cx) * 2.0 / camera.focal_length_x + TX,  // x-coordinate Top-right
                (0.0 - camera.cy) * 2.0 / camera.focal_length_y + TY,  // y-coordinate Top-right
                2.0 + TZ                                               // z-coordinate
            )
        ); // Top-right

        assert_eq!(
            vertices[2],
            Point3::new(
                (0.0 - camera.cx) * 3.0 / camera.focal_length_x + TX,  // x-coordinate Bottom-left
                (1.0 - camera.cy) * 3.0 / camera.focal_length_y + TY,  // y-coordinate Bottom-left
                3.0 + TZ                                               // z-coordinate
            )
        ); // Bottom-left

        assert_eq!(
            vertices[3],
            Point3::new(
                (1.0 - camera.cx) * 4.0 / camera.focal_length_x + TX,  // x-coordinate Bottom-right
                (1.0 - camera.cy) * 4.0 / camera.focal_length_y + TY,  // y-coordinate Bottom-right
                4.0 + TZ                                               // z-coordinate
            )
        ); // Bottom-right

        // Check the triangle indices
        let indices = trimesh.indices();
        assert_eq!(indices.len(), 2); // Two triangles in a 2x2 grid.

        // Verify the indices of the triangles
        assert_eq!(indices[0], [0, 1, 2]); // Triangle formed by Top-left, Top-right, Bottom-left
        assert_eq!(indices[1], [1, 3, 2]); // Triangle formed by Top-right, Bottom-right, Bottom-left
    }

    #[test]
    fn test_depth_field_with_translation_and_rotation() {
        fn compute_transformed_point(
            u: f32,           // Column index in the grid
            v: f32,           // Row index in the grid
            z: f32,           // Depth value
            camera: &Camera, // Pass the Camera by reference to access its parameters
        ) -> Point3<f32> {
            // Step 1: Compute camera-space coordinates
            let x_c = (u - camera.cx) * z / camera.focal_length_x;
            let y_c = (v - camera.cy) * z / camera.focal_length_y;

            // Step 2: Apply rotation around Z-axis (using the Rotation part of the pose)
            let rotation = camera.pose.rotation; // Extract the rotation part (Quaternion)
            let translation = camera.pose.translation.vector; // Extract the translation part

            let rotated_point = rotation.transform_point(&Point3::new(x_c, y_c, z));

            // Step 3: Apply translation (TX, TY, TZ from pose translation vector)
            Point3::new(
                rotated_point.x + translation.x,
                rotated_point.y + translation.y,
                rotated_point.z + translation.z,
            )
        }
        
        const TX: f32 = 10.0; // Translation along x-axis
        const TY: f32 = 20.0; // Translation along y-axis
        const TZ: f32 = 30.0; // Translation along z-axis
        const ROT_Z: f32 = std::f32::consts::FRAC_PI_4; // 45 degrees in radians

        let camera = Camera {
            focal_length_x: FCL,
            focal_length_y: FCL,
            cx: 0.5, // Principal point centered in 2x2 grid
            cy: 0.5,
            depth_range: 0.0..100.0,
            pose: Isometry3::new(
                [TX, TY, TZ].into(),                       // Translation
                nalgebra::Vector3::z() * ROT_Z,   // 45-degree rotation around Z-axis
            ),
        };

        // 2x2 depth field
        let depth_field = vec![
            1.0, 2.0, // Top row
            3.0, 4.0, // Bottom row
        ];
        let width = 2;
        let height = 2;

        // Generate the mesh
        let trimesh = camera.depth_field_to_trimesh(&depth_field, width, height);

        // Check the vertices
        let vertices = trimesh.vertices();
        assert_eq!(vertices.len(), 4); // Four points in the grid, all valid.

        // Ensure the vertices are projected correctly (with translation + rotation)
        assert_eq!(
            vertices[0],
            compute_transformed_point(0.0, 0.0, 1.0, &camera) // Top-left
        );

        assert_eq!(
            vertices[1],
            compute_transformed_point(1.0, 0.0, 2.0, &camera) // Top-right
        );

        assert_eq!(
            vertices[2],
            compute_transformed_point(0.0, 1.0, 3.0, &camera) // Bottom-left
        );

        assert_eq!(
            vertices[3],
            compute_transformed_point(1.0, 1.0, 4.0, &camera) // Bottom-right
        );

        // Check the triangle indices
        let indices = trimesh.indices();
        assert_eq!(indices.len(), 2); // Two triangles in a 2x2 grid.

        // Verify the indices of the triangles
        assert_eq!(indices[0], [0, 1, 2]); // Triangle formed by Top-left, Top-right, Bottom-left
        assert_eq!(indices[1], [1, 3, 2]); // Triangle formed by Top-right, Bottom-right, Bottom-left
    }    
}
