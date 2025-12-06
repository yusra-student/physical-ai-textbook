import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class VslamDemo(Node):
    def __init__(self):
        super().__init__('isaac_ros_vslam_demo')
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/visual_slam/tracking/slam_pose', # Typical topic for VSLAM pose output
            self.pose_callback,
            10
        )
        self.marker_publisher = self.create_publisher(MarkerArray, 'vslam_path_markers', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.path_markers = MarkerArray()
        self.path_marker_id = 0
        self.get_logger().info('Isaac ROS VSLAM Demo node started. Waiting for VSLAM pose data...')

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.get_logger().info(f'Received VSLAM pose: x={msg.pose.pose.position.x:.2f}, '
                               f'y={msg.pose.pose.position.y:.2f}, z={msg.pose.pose.position.z:.2f}')

        # Add a marker to visualize the path
        marker = Marker()
        marker.header = msg.header
        marker.ns = "vslam_path"
        marker.id = self.path_marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = msg.pose.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.path_markers.markers.append(marker)
        self.path_marker_id += 1

        # Limit the number of markers to keep visualization efficient
        if len(self.path_markers.markers) > 500:
            self.path_markers.markers.pop(0)
            for i, m in enumerate(self.path_markers.markers):
                m.id = i # Re-ID markers after pop

        self.marker_publisher.publish(self.path_markers)


def main(args=None):
    rclpy.init(args=args)
    vslam_demo = VslamDemo()
    rclpy.spin(vslam_demo)
    vslam_demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
