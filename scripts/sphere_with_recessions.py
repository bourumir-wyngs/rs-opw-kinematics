import numpy as np
import open3d as o3d


def generate_sphere_with_recessions(radius, recession_depth, recession_radius, resolution):
    """
    Generate a sphere with recessions at the North Pole, South Pole, and ±X, ±Y axes.

    Parameters:
    - radius: Radius of the sphere.
    - recession_depth: The depth of the recessions (reduction of radius).
    - recession_radius: The radius area of the recessions at each axis or pole.
    - resolution: Sphere resolution (number of steps for latitude/longitude).

    Returns:
    - vertices: NumPy array of vertex coordinates.
    - triangles: NumPy array of triangle indices.
    """
    vertices = []
    triangles = []

    for i in range(resolution + 1):
        theta = np.pi * i / resolution  # Latitude angle
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        for j in range(resolution + 1):
            phi = 2.0 * np.pi * j / resolution  # Longitude angle
            sin_phi = np.sin(phi)
            cos_phi = np.cos(phi)

            # Convert spherical coordinates to Cartesian coordinates
            x = radius * sin_theta * cos_phi
            y = radius * sin_theta * sin_phi
            z = radius * cos_theta

            # Apply recession effect near the North Pole (z ≈ radius)
            if z > (radius - recession_radius):  # Near North Pole
                distance_to_pole = np.sqrt(x ** 2 + y ** 2)  # Distance from North Pole
                if distance_to_pole < recession_radius:  # Within the recession radius
                    recession_factor = (1 - (distance_to_pole / recession_radius))  # Proportional reduction
                    z -= recession_depth * recession_factor  # Reduce z coordinate

            # Apply recession effect near the South Pole (z ≈ -radius)
            elif z < -(radius - recession_radius):  # Near South Pole
                distance_to_pole = np.sqrt(x ** 2 + y ** 2)  # Distance from South Pole
                if distance_to_pole < recession_radius:  # Within the recession radius
                    recession_factor = (1 - (distance_to_pole / recession_radius))  # Proportional reduction
                    z += recession_depth * recession_factor  # Increase z coordinate (reduce depth)

            # Apply recession effect near the +X axis
            elif x > (radius - recession_radius):  # Near +X
                distance_to_axis = np.sqrt(y ** 2 + z ** 2)  # Distance to +X axis
                if distance_to_axis < recession_radius:  # Within the recession radius
                    recession_factor = (1 - (distance_to_axis / recession_radius))  # Proportional reduction
                    x -= recession_depth * recession_factor  # Reduce x coordinate

            # Apply recession effect near the -X axis
            elif x < -(radius - recession_radius):  # Near -X
                distance_to_axis = np.sqrt(y ** 2 + z ** 2)  # Distance to -X axis
                if distance_to_axis < recession_radius:  # Within the recession radius
                    recession_factor = (1 - (distance_to_axis / recession_radius))  # Proportional reduction
                    x += recession_depth * recession_factor  # Increase x coordinate (reduce depth)

            # Apply recession effect near the +Y axis
            elif y > (radius - recession_radius):  # Near +Y
                distance_to_axis = np.sqrt(x ** 2 + z ** 2)  # Distance to +Y axis
                if distance_to_axis < recession_radius:  # Within the recession radius
                    recession_factor = (1 - (distance_to_axis / recession_radius))  # Proportional reduction
                    y -= recession_depth * recession_factor  # Reduce y coordinate

            # Apply recession effect near the -Y axis
            elif y < -(radius - recession_radius):  # Near -Y
                distance_to_axis = np.sqrt(x ** 2 + z ** 2)  # Distance to -Y axis
                if distance_to_axis < recession_radius:  # Within the recession radius
                    recession_factor = (1 - (distance_to_axis / recession_radius))  # Proportional reduction
                    y += recession_depth * recession_factor  # Increase y coordinate (reduce depth)

            vertices.append([x, y, z])

    # Create triangles (faces)
    for i in range(resolution):
        for j in range(resolution):
            current = i * (resolution + 1) + j
            next_row = (i + 1) * (resolution + 1) + j

            # Two triangles for each quad
            triangles.append([current, next_row, current + 1])
            triangles.append([next_row, next_row + 1, current + 1])

    return np.array(vertices), np.array(triangles)


def save_mesh_as_ply(vertices, triangles, file_path):
    """
    Save the generated sphere mesh to a PLY file.

    Parameters:
    - vertices: List of vertex coordinates (Nx3 array).
    - triangles: List of triangle indices (Mx3 array).
    - file_path: Path to the output PLY file.
    """
    # Create an Open3D TriangleMesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)

    # Save the mesh as a PLY file
    o3d.io.write_triangle_mesh(file_path, mesh)
    print(f"Mesh saved to {file_path}")


if __name__ == "__main__":
    # Sphere parameters
    radius = 1.0  # Sphere radius
    recession_depth = 0.3  # Depth of the recessions
    recession_radius = 0.4  # The radius of recession areas
    resolution = 128  # Higher resolution provides smoother geometry

    # Generate a sphere with six recessions (North/South Poles and ±X/±Y axes)
    vertices, triangles = generate_sphere_with_recessions(radius, recession_depth, recession_radius, resolution)

    # Save the sphere as a PLY file
    save_mesh_as_ply(vertices, triangles, "sphere_with_six_recessions.ply")