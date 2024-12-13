use nalgebra::{Isometry3};

use crate::calipers::largest_fitting_rectangle;
use geo::{ConcaveHull, LineString, Point as GeoPoint, Polygon};
use parry3d::math::Point as ParryPoint;
use parry3d::shape::TriMesh;

use crate::projector::{Axis, Projector, RayDirection};

const MARGIN: f32 = 0.04 / 0.005; // Margin for the engraving area
const PROJECTOR_CHECK_POINTS: usize = 16;
const PROJECTOR_RADIUS: f32 = 0.02 / 0.005; //0.04 / 0.005;

/// Build a series of poses for "engraving" over the mesh.
pub fn build_engraving_path(
    mesh: &TriMesh,
    path: &Vec<(f32, f32)>,
    axis: Axis,
    ray_direction: RayDirection,
) -> Result<Vec<Isometry3<f32>>, String> {
    // Determine a suitable rectangle where we could engrave on the mesh.
    let (bottom_left, top_right) = axis_aligned_bounding_rectangle(mesh, axis)?;

    // Calculate dimensions of the bounding rectangle
    let rect_width = top_right.y - bottom_left.y;
    let rect_height = top_right.z - bottom_left.z;

    // Calculate dimensions of the path
    let path_min_x = path.iter().map(|(x, _)| *x).fold(f32::INFINITY, f32::min);
    let path_max_x = path
        .iter()
        .map(|(x, _)| *x)
        .fold(f32::NEG_INFINITY, f32::max);
    let path_min_y = path.iter().map(|(_, y)| *y).fold(f32::INFINITY, f32::min);
    let path_max_y = path
        .iter()
        .map(|(_, y)| *y)
        .fold(f32::NEG_INFINITY, f32::max);

    let path_width = path_max_x - path_min_x;
    let path_height = path_max_y - path_min_y;

    // Compute scaling factors to stretch the path into the bounding rectangle with margins
    let scale_x = (rect_width - 2.0 * MARGIN) / path_width;
    let scale_y = (rect_height - 2.0 * MARGIN) / path_height;

    // Transform the path to fit into the bounding rectangle based on the axis
    let transformed_path: Vec<ParryPoint<f32>> = path
        .iter()
        .map(|(x, y)| match axis {
            Axis::X => ParryPoint::new(
                0.0,
                (x - path_min_x) * scale_x + bottom_left.y + MARGIN,
                (y - path_min_y) * scale_y + bottom_left.z + MARGIN,
            ),
            Axis::Y => ParryPoint::new(
                (x - path_min_x) * scale_x + bottom_left.y + MARGIN,
                0.0,
                (y - path_min_y) * scale_y + bottom_left.z + MARGIN,
            ),
            Axis::Z => ParryPoint::new(
                (x - path_min_x) * scale_x + bottom_left.y + MARGIN,
                (y - path_min_y) * scale_y + bottom_left.z + MARGIN,
                0.0,
            ),
        })
        .collect();

    let projector = Projector {
        check_points: PROJECTOR_CHECK_POINTS,
        radius: PROJECTOR_RADIUS,
    };

    // Project transformed points onto the mesh
    Ok(transformed_path
        .iter()
        .filter_map(|point| projector.project(mesh, point, ray_direction, axis))
        .collect())
}

pub fn axis_aligned_bounding_rectangle(
    mesh: &TriMesh,
    axis: Axis,
) -> Result<(ParryPoint<f32>, ParryPoint<f32>), String> {
    let concavity = 0.1; // the mesh we have is is mm so value may need adjustment later

    // Step 1: Project all mesh points onto the relevant plane based on the axis
    let projected_points: Vec<GeoPoint<f32>> = mesh
        .vertices()
        .iter()
        .map(|vertex| match axis {
            Axis::X => GeoPoint::new(vertex.y, vertex.z), // YZ plane
            Axis::Y => GeoPoint::new(vertex.x, vertex.z), // XZ plane
            Axis::Z => GeoPoint::new(vertex.x, vertex.y), // XY plane
        })
        .collect();

    // Step 2: Compute the concave hull of the projected points

    let concave_hull: Polygon<f32> =
        Polygon::new(LineString::from(projected_points), vec![]).concave_hull(concavity);

    // Step 3: Compute the largest fitting rectangle in the relevant plane
    if let Some((bottom_left, top_right)) = largest_fitting_rectangle(&concave_hull) {
        Ok(match axis {
            Axis::X => (
                ParryPoint::new(0.0, bottom_left.x(), bottom_left.y()),
                ParryPoint::new(0.0, top_right.x(), top_right.y()),
            ),
            Axis::Y => (
                ParryPoint::new(bottom_left.x(), 0.0, bottom_left.y()),
                ParryPoint::new(top_right.x(), 0.0, top_right.y()),
            ),
            Axis::Z => (
                ParryPoint::new(bottom_left.x(), bottom_left.y(), 0.0),
                ParryPoint::new(top_right.x(), top_right.y(), 0.0),
            ),
        })
    } else {
        Err("Failed to compute the largest fitting rectangle.".to_string())
    }
}

#[cfg(test)]
mod tests {}
