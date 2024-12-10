import json
import matplotlib.pyplot as plt

def visualize_rectangle_and_mesh(json_file):
    # Load rectangle and mesh data from JSON
    with open(json_file, 'r') as f:
        data = json.load(f)

    corners = data['corners']
    mesh_points = data['mesh_points']

    # Extract YZ coordinates for the corners
    corner_x = [corners[0]['y'], corners[1]['y'], corners[3]['y'], corners[2]['y'], corners[0]['y']]
    corner_y = [corners[0]['z'], corners[1]['z'], corners[3]['z'], corners[2]['z'], corners[0]['z']]

    # Extract YZ coordinates for all mesh points
    mesh_x = [point['y'] for point in mesh_points]
    mesh_y = [point['z'] for point in mesh_points]

    # Plot the mesh points
    plt.figure(figsize=(8, 8))
    plt.scatter(mesh_x, mesh_y, color='blue', s=1, label='Mesh Points')

    # Plot the rectangle
    plt.plot(corner_x, corner_y, 'b-', linewidth=2, label='Bounding Rectangle')
    plt.scatter(corner_x, corner_y, color='red', s=20, label='Corners')

    # Add labels, legend, and grid
    plt.title("YZ Bounding Rectangle and Mesh Points")
    plt.xlabel("Y Coordinate")
    plt.ylabel("Z Coordinate")
    plt.legend()
    plt.grid()

    # Show the plot
    plt.show()

if __name__ == "__main__":
    visualize_rectangle_and_mesh("work/rectangle_and_mesh_data.json")

