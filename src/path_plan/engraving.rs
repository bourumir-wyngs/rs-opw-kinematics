use std::ops::Range;

use crate::calipers::largest_fitting_rectangle;
use geo::{ConcaveHull, LineString, Point as GeoPoint, Polygon};
use parry3d::math::Point as ParryPoint;
use parry3d::shape::TriMesh;
use crate::annotations::{AnnotatedPathStep, AnnotatedPose, PathFlags};
use crate::projector::{Axis, Projector, RayDirection};

const MARGIN: f32 = 0.04;
const PROJECTOR_CHECK_POINTS: usize = 16;
const PROJECTOR_RADIUS: f32 = 0.02;

pub fn generate_raster_points(r: usize, n: usize) -> Vec<AnnotatedPathStep> {
    let mut points = Vec::with_capacity(r * n);
    let y_step = 2.0 / (r as f32 - 1.0); // Vertical spacing between rows
    let x_step = 2.0 / (n as f32 - 1.0); // Horizontal spacing between points in a row

    for i in 0..r {
        let y = -1.0 + i as f32 * y_step; // Calculate the y-coordinate for the row

        // Conditional to alternate row directions
        if i % 2 == 0 {
            // Even row: left-to-right
            for j in 0..n {
                let x = -1.0 + j as f32 * x_step;
                points.push(AnnotatedPathStep {
                    point: (x, y),
                    flags: PathFlags::FORWARDS,
                });
            }
        } else {
            // Odd row: right-to-left
            for j in (0..n).rev() {
                let x = -1.0 + j as f32 * x_step;
                points.push(AnnotatedPathStep {
                    point: (x, y),
                    flags: PathFlags::BACKWARDS,
                });
            }
        }
    }
    points
}

/// Build a series of poses for "engraving" over the mesh.
pub fn build_engraving_path_side_projected(
    mesh: &TriMesh,
    path: &Vec<AnnotatedPathStep>,
    axis: Axis,
    ray_direction: RayDirection,
) -> Result<Vec<AnnotatedPose>, String> {
    // Determine a suitable rectangle where we could engrave on the mesh.
    let (bottom_left, top_right) = axis_aligned_bounding_rectangle(mesh, axis)?;

    println!(
        "top_rights: {:?}, bottom_left: {:?}",
        bottom_left, top_right
    );

    // Calculate dimensions of the bounding rectangle
    let rect_width = top_right.x() - bottom_left.x();
    let rect_height = top_right.y() - bottom_left.y();

    // Calculate dimensions of the path
    let path_min_x = path
        .iter()
        .map(|step| step.point.0)  // Access point and its x coordinate
        .fold(f32::INFINITY, f32::min);
    let path_max_x = path
        .iter()
        .map(|step| step.point.0)  // Access point and its x coordinate
        .fold(f32::NEG_INFINITY, f32::max);

    let path_min_y = path
        .iter()
        .map(|step| step.point.1)  // Access point and its y coordinate
        .fold(f32::INFINITY, f32::min);
    let path_max_y = path
        .iter()
        .map(|step| step.point.1)  // Access point and its y coordinate
        .fold(f32::NEG_INFINITY, f32::max);

    // Compute path width and height
    let path_width = path_max_x - path_min_x;
    let path_height = path_max_y - path_min_y;

    // Compute scaling factors to stretch the path into the bounding rectangle with margins
    let scale_x = (rect_width - 2.0 * MARGIN) / path_width;
    let scale_y = (rect_height - 2.0 * MARGIN) / path_height;

    // Transform the path to fit into the bounding rectangle based on the axis
    let transformed_path: Vec<ParryPoint<f32>> = path
        .iter()
        .map(|step| {
            let x_2d = (step.point.0 - path_min_x) * scale_x + bottom_left.x() + MARGIN;
            let y_2d = (step.point.1 - path_min_y) * scale_y + bottom_left.y() + MARGIN;

            // Place in 3D space
            match axis {
                Axis::X => ParryPoint::new(0.0, x_2d, y_2d),
                Axis::Y => ParryPoint::new(x_2d, 0.0, y_2d),
                Axis::Z => ParryPoint::new(x_2d, y_2d, 0.0),
            }
        })
        .collect();

    let projector = Projector {
        check_points: PROJECTOR_CHECK_POINTS,
        radius: PROJECTOR_RADIUS,
    };

    // Use our own implementation for Y axis (Parry fails on normal parallel to Y)
    Ok(transformed_path
        .iter()
        .filter_map(|point| projector.project_flat(mesh, point, ray_direction, axis))
        .collect())
}

pub fn axis_aligned_bounding_rectangle(
    mesh: &TriMesh,
    axis: Axis,
) -> Result<(GeoPoint<f32>, GeoPoint<f32>), String> {
    // THIS CODE IS SENSITIVE TO ABSOLUTE DIMENSIONS
    // Concavity cannot be scaled with 'scale'
    let concavity = 0.1;
    let scale = 1.0 / 0.005;
    // Step 1: Project all mesh points onto the relevant plane based on the axis
    let projected_points: Vec<GeoPoint<f32>> = mesh
        .vertices()
        .iter()
        .map(|vertex| match axis {
            Axis::X => GeoPoint::new(vertex.y * scale, vertex.z * scale), // YZ plane
            Axis::Y => GeoPoint::new(vertex.x * scale, vertex.z * scale), // XZ plane
            Axis::Z => GeoPoint::new(vertex.x * scale, vertex.y * scale), // XY plane
        })
        .collect();

    // Step 2: Compute the concave hull of the projected points

    let concave_hull: Polygon<f32> =
        Polygon::new(LineString::from(projected_points), vec![]).concave_hull(concavity);

    // Step 3: Compute the largest fitting rectangle in the relevant plane
    let lff = largest_fitting_rectangle(&concave_hull)?;

    Ok((lff.0 / scale, lff.1 / scale))
}

fn find_min_max(path: &Vec<AnnotatedPathStep>) -> ((f32, f32), (f32, f32)) {
    // Initialize min and max values to INFINITY and NEG_INFINITY.
    let (mut path_min_x, mut path_max_x) = (f32::INFINITY, f32::NEG_INFINITY);
    let (mut path_min_y, mut path_max_y) = (f32::INFINITY, f32::NEG_INFINITY);

    // Iterate over all points in the path to determine min and max values for x and y.
    for step in path {
        let x = step.point.0; // Access the x coordinate
        let y = step.point.1; // Access the y coordinate

        if x < path_min_x {
            path_min_x = x;
        }
        if x > path_max_x {
            path_max_x = x;
        }
        if y < path_min_y {
            path_min_y = y;
        }
        if y > path_max_y {
            path_max_y = y;
        }
    }

    // Return the computed min/max values as tuples ((min_x, min_y), (max_x, max_y)).
    ((path_min_x, path_min_y), (path_max_x, path_max_y))
}

pub fn project_from_cylinder_to_mesh(
    mesh: &TriMesh,
    path: &Vec<AnnotatedPathStep>,
    projection_radius: f32,
    height: Range<f32>,
    angle: Range<f32>,
    axis: Axis,
) -> Result<Vec<AnnotatedPose>, String> {
    use std::f32::consts::PI;
    // Validate inputs using range methods.
    if height.start >= height.end {
        return Err("Invalid height range specified.".to_string());
    }
    if angle.start < 0.0 || angle.end > 2.0 * PI {
        return Err("Angle values must be in the range [0, 2π].".to_string());
    }

    // Compute angle range directly using range-inclusive methods.
    let angle_range = if angle.start > angle.end {
        (2.0 * PI - angle.start) + angle.end
    } else {
        angle.end - angle.start
    };

    // Access height bounds.
    let height_range = height.end - height.start;

    println!(
        "angle_range: {}, height range: {}",
        angle_range.to_degrees(),
        height_range
    );

    let ((path_min_x, path_min_y), (path_max_x, path_max_y)) = find_min_max(&path);

    let path_width = path_max_x - path_min_x;
    let path_height = path_max_y - path_min_y;

    // Compute scaling factors and offsets
    let scale_x = angle_range / path_width;
    let scale_y = height_range / path_height;

    // Transform the path into cylindrical coordinates
    let transformed_path: Vec<GeoPoint<f32>> = path
        .iter()
        .map(|step| {
            let x = step.point.0; // Access the x coordinate
            let y = step.point.1; // Access the y coordinate

            // The direction here is selected so that the image would not be mirrored when projected
            let theta = (x - path_min_x) * scale_x;
            let z = (y - path_min_y) * scale_y + height.start;

            GeoPoint::new(theta, z)
        })
        .collect();

    // Step 3: Create a projector for projecting transformed path points
    let projector = Projector {
        check_points: 24,     // Defined number of normals to check
        radius: 0.02,         // Radius, defined as PROJECTOR_RADIUS
    };

    // Step 4: Project each point on the transformed path to the mesh and collect Isometry3
    let isometries: Vec<AnnotatedPose> = transformed_path
        .iter()
        .filter_map(|point| {
            projector.project_cylindric(&mesh, point, projection_radius, axis)
        })
        .collect();

    // Step 5: Ensure the result contains valid projections
    if isometries.is_empty() {
        return Err("Failed to project any points onto the mesh.".to_string());
    }

    Ok(isometries)
}

#[cfg(test)]
mod tests {}
