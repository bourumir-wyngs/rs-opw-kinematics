use crate::projector::Axis;
use nalgebra::{Point3, Vector3};
use parry3d::shape::TriMesh;
use std::f32::consts::PI;

/// Create a cylindrical mesh aligned to the given axis.
pub fn cylinder_mesh(
    radius: f32,
    height: f32,
    segments: usize, // Number of segments for approximating the cylinder
    axis: Axis,
) -> TriMesh {
    let segments = segments as u32;
    let h2 = height / 2.0;
    assert!(
        segments >= 3,
        "There must be at least 3 segments to form a cylinder."
    );

    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let angle_step = 2.0 * PI / segments as f32;

    // Generate the base and top circle vertices
    for i in 0..segments {
        let angle = i as f32 * angle_step;
        let (sin, cos) = angle.sin_cos();

        // Base circle vertex
        let base_vertex = match axis {
            Axis::X => Point3::new(-h2, radius * cos, radius * sin),
            Axis::Y => Point3::new(radius * cos, -h2, radius * sin),
            Axis::Z => Point3::new(radius * cos, radius * sin, -h2),
        };
        vertices.push(base_vertex);

        // Top circle vertex
        let top_vertex = match axis {
            Axis::X => Point3::new(h2, radius * cos, radius * sin),
            Axis::Y => Point3::new(radius * cos, h2, radius * sin),
            Axis::Z => Point3::new(radius * cos, radius * sin, h2),
        };
        vertices.push(top_vertex);
    }

    // Generate the triangle indices for the side faces of the cylinder
    for i in 0..segments as u32 {
        let next = (i + 1) % segments;

        let base_curr = i * 2;
        let base_next = next * 2;
        let top_curr = base_curr + 1;
        let top_next = base_next + 1;

        // Create two triangles for the quad side face
        indices.push([base_curr, base_next, top_curr].into());
        indices.push([top_curr, base_next, top_next].into());
    }

    // Generate the circle caps (triangles forming the top and base circles)
    let base_center = vertices.len() as u32;
    let top_center = base_center + 1 as u32;

    // Add center points for the top and base circles
    vertices.push(match axis {
        Axis::X => Point3::new(-h2, 0.0, 0.0),
        Axis::Y => Point3::new(0.0, -h2, 0.0),
        Axis::Z => Point3::new(0.0, 0.0, -h2),
    });
    vertices.push(match axis {
        Axis::X => Point3::new(h2, 0.0, 0.0),
        Axis::Y => Point3::new(0.0, h2, 0.0),
        Axis::Z => Point3::new(0.0, 0.0, h2),
    });

    for i in 0..segments as u32 {
        let next = (i + 1) % segments;

        // Base circle
        indices.push([i * 2, base_center, next * 2]);

        // Top circle
        indices.push([top_center, i * 2 + 1, next * 2 + 1].into());
    }

    // Create and return the triangular mesh
    TriMesh::new(vertices, indices)
}

pub fn sphere_mesh(radius: f32, resolution: usize) -> TriMesh {
    // Each point in the grid
    let mut vertices = Vec::with_capacity((resolution + 1) * (resolution + 1));

    // 6 indices per quad (2 triangles)
    let mut indices = Vec::with_capacity(resolution * resolution * 6);

    // Generate vertices using spherical coordinates
    for i in 0..=resolution {
        let theta = PI * i as f32 / resolution as f32; // Latitude
        let (sin_theta, cos_theta) = theta.sin_cos();

        for j in 0..=resolution {
            let phi = 2.0 * PI * j as f32 / resolution as f32; // Longitude
            let (sin_phi, cos_phi) = phi.sin_cos();

            let x = radius * sin_theta * cos_phi;
            let y = radius * sin_theta * sin_phi;
            let z = radius * cos_theta;

            vertices.push(Point3::new(x, y, z));
        }
    }
    
    // Generate triangle indices
    let resolution = resolution as u32;
    for i in 0..resolution {
        for j in 0..resolution {
            let current = i * (resolution + 1) + j;
            let next = current + resolution + 1;

            // First triangle of the quad
            indices.push([current, next, current + 1]);

            // Second triangle of the quad
            indices.push([next, next + 1, current + 1]);
        }
    }

    // Create the TriMesh from vertices and triangle indices
    TriMesh::new(vertices, indices)
}

/// Generate a sphere with six recessions (North/South Poles and ±X/±Y axes).
///
/// # Parameters:
/// - `radius`: Radius of the sphere.
/// - `recession_depth`: Depth of each recession.
/// - `recession_radius`: Radius of the area affected by the recession.
/// - `resolution`: Resolution of the sphere (number of latitude and longitude divisions).
///
/// # Returns:
/// A `TriMesh` representing the sphere with six recessions.
pub fn sphere_with_recessions(
    radius: f32,
    recession_depth: f32,
    recession_radius: f32,
    resolution: usize,
) -> TriMesh {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    for i in 0..=resolution {
        let theta = std::f32::consts::PI * i as f32 / resolution as f32; // Latitude angle
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();

        for j in 0..=resolution {
            let phi = 2.0 * std::f32::consts::PI * j as f32 / resolution as f32; // Longitude angle
            let sin_phi = phi.sin();
            let cos_phi = phi.cos();

            // Convert spherical coordinates to Cartesian coordinates
            let mut x = radius * sin_theta * cos_phi;
            let mut y = radius * sin_theta * sin_phi;
            let mut z = radius * cos_theta;

            // Apply recession effect for the six recessions
            // North Pole Recession
            if z > radius - recession_radius {
                let distance_to_pole = (x.powi(2) + y.powi(2)).sqrt();
                if distance_to_pole < recession_radius {
                    let recession_factor = 1.0 - (distance_to_pole / recession_radius);
                    z -= recession_depth * recession_factor;
                }
            }
            // South Pole Recession
            else if z < -(radius - recession_radius) {
                let distance_to_pole = (x.powi(2) + y.powi(2)).sqrt();
                if distance_to_pole < recession_radius {
                    let recession_factor = 1.0 - (distance_to_pole / recession_radius);
                    z += recession_depth * recession_factor;
                }
            }
            // +X Axis Recession
            else if x > radius - recession_radius {
                let distance_to_axis = (y.powi(2) + z.powi(2)).sqrt();
                if distance_to_axis < recession_radius {
                    let recession_factor = 1.0 - (distance_to_axis / recession_radius);
                    x -= recession_depth * recession_factor;
                }
            }
            // -X Axis Recession
            else if x < -(radius - recession_radius) {
                let distance_to_axis = (y.powi(2) + z.powi(2)).sqrt();
                if distance_to_axis < recession_radius {
                    let recession_factor = 1.0 - (distance_to_axis / recession_radius);
                    x += recession_depth * recession_factor;
                }
            }
            // +Y Axis Recession
            else if y > radius - recession_radius {
                let distance_to_axis = (x.powi(2) + z.powi(2)).sqrt();
                if distance_to_axis < recession_radius {
                    let recession_factor = 1.0 - (distance_to_axis / recession_radius);
                    y -= recession_depth * recession_factor;
                }
            }
            // -Y Axis Recession
            else if y < -(radius - recession_radius) {
                let distance_to_axis = (x.powi(2) + z.powi(2)).sqrt();
                if distance_to_axis < recession_radius {
                    let recession_factor = 1.0 - (distance_to_axis / recession_radius);
                    y += recession_depth * recession_factor;
                }
            }

            vertices.push(Point3::new(x, y, z));
        }
    }

    // Create triangles (faces)
    for i in 0..resolution {
        for j in 0..resolution {
            let current = i * (resolution + 1) + j;
            let next_row = (i + 1) * (resolution + 1) + j;

            // Two triangles for each quad
            indices.push([current as u32, next_row as u32, (current + 1) as u32]);
            indices.push([
                next_row as u32,
                (next_row + 1) as u32,
                (current + 1) as u32,
            ]);
        }
    }

    // Build the TriMesh
    TriMesh::new(vertices, indices)
}
