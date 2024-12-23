use nalgebra::{Point3};
use parry3d::shape::TriMesh;
use std::f32::consts::PI;
use crate::projector::Axis;

/// Create a cylindrical mesh aligned to the given axis.
pub fn create_cylindric_mesh(
    radius: f32,
    height: f32,
    segments: usize, // Number of segments for approximating the cylinder
    axis: Axis,
) -> TriMesh {
    let segments = segments as u32;
    let h2 = height / 2.0;
    assert!(segments >= 3, "There must be at least 3 segments to form a cylinder.");

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
            Axis::Cylinder => unreachable!(),
        };
        vertices.push(base_vertex);

        // Top circle vertex
        let top_vertex = match axis {
            Axis::X => Point3::new(h2, radius * cos, radius * sin),
            Axis::Y => Point3::new(radius * cos, h2, radius * sin),
            Axis::Z => Point3::new(radius * cos, radius * sin, h2),
            Axis::Cylinder => unreachable!(),
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
        Axis::Cylinder => unreachable!(),
    });
    vertices.push(match axis {
        Axis::X => Point3::new(h2, 0.0, 0.0),
        Axis::Y => Point3::new(0.0, h2, 0.0),
        Axis::Z => Point3::new(0.0, 0.0, h2),
        Axis::Cylinder => unreachable!(),
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