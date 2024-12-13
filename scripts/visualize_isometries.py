import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray

# Define a global scale factor
SCALE_FACTOR = 0.005

def load_json(file_path):
    """Load the JSON file and parse it into a list of poses."""
    with open(file_path, 'r') as file:
        data = json.load(file)

    poses = []
    for entry in data:
        pose = Pose()
        pose.position.x = entry['position']['x'] * SCALE_FACTOR
        pose.position.y = entry['position']['y'] * SCALE_FACTOR
        pose.position.z = entry['position']['z'] * SCALE_FACTOR
        pose.orientation.x = float(entry['rotation']['x']) 
        pose.orientation.y = float(entry['rotation']['y'])
        pose.orientation.z = float(entry['rotation']['z'])
        pose.orientation.w = float(entry['rotation']['w'])

        # Expected close to correct rotation
        if False:
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 1.0
            pose.orientation.w = 0.0        
        
        poses.append(pose)
    return poses

class PoseVisualizer(Node):
    def __init__(self, poses):
        super().__init__('pose_visualizer')
        self.pose_publisher = self.create_publisher(PoseArray, 'pose_array', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'line_markers', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.poses = poses

    def timer_callback(self):
        # Publish PoseArray
        pose_array = PoseArray()
        pose_array.header.frame_id = 'world'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = self.poses
        self.pose_publisher.publish(pose_array)

        # Publish MarkerArray for lines
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lines'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05 * SCALE_FACTOR  # Line width scaled
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for pose in self.poses:
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z
            marker.points.append(point)

        marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)

if __name__ == '__main__':
    # Load poses from the JSON file
    json_file_path = '/home/audrius/opw/rs-opw-kinematics/isometries.json'  # Replace with your JSON file path
    poses = load_json(json_file_path)

    rclpy.init()
    node = PoseVisualizer(poses)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
