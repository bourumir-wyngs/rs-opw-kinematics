import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
# Define a global scale factor
SCALE_FACTOR = 0.005

def load_json(file_path, scale_factor=SCALE_FACTOR):
    """Load the JSON file and parse it into a list of poses."""
    import time
    while True:
        try:
            with open(file_path, 'r') as file:
                data = json.load(file)
            break
        except (OSError, json.JSONDecodeError) as e:
            print(f"Error reading JSON file: {e}")
            time.sleep(2)
    data = data or []
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
        if False:  # Example modification logic for orientation
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 1.0
            pose.orientation.w = 0.0        
        
        poses.append(pose)
    return poses

class PoseVisualizer(Node, FileSystemEventHandler):
    """ROS2 Node that visualizes poses and reloads JSON on file modification."""

    def __init__(self, file_path):
        self.file_path = file_path
        self.poses = load_json(file_path)
        super().__init__('pose_visualizer')
        self.pose_publisher = self.create_publisher(PoseArray, 'pose_array', 10)

        # Setup file monitoring
        self.observer = Observer()
        self.observer.schedule(self, path=self.file_path, recursive=False)
        self.observer.start()
        self.marker_publisher = self.create_publisher(MarkerArray, 'line_markers', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.file_lock = False
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
    def on_modified(self, event):
        """Callback for file modification events."""
        if event.src_path == self.file_path and not self.file_lock:
            try:
                self.get_logger().info('File modified, reloading poses.')
                self.file_lock = True
                self.poses = load_json(self.file_path)
                self.file_lock = False
            except Exception as e:
                self.get_logger().error(f"Error while reloading poses: {e}")
            finally:
                self.file_lock = False
if __name__ == '__main__':
    # Load poses from the JSON file
    json_file_path = 'work/isometries.json'  # Replace with your JSON file path

    rclpy.init()
    node = PoseVisualizer(json_file_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.observer.stop()
        node.observer.join()
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
