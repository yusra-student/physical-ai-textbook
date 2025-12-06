import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import cv2
from cv_bridge import CvBridge

class IsaacRosObjectDetection(Node):
    def __init__(self):
        super().__init__('isaac_ros_object_detection_node')
        self.image_subscription = self.create_subscription(
            Image,
            '/front/stereo_camera/left/image_rect', # Example Isaac Sim camera topic
            self.image_callback,
            10
        )
        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/isaac_ros_detectnet/detections', # Example Isaac ROS DetectNet topic
            self.detection_callback,
            10
        )
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_detections = None
        self.get_logger().info('Isaac ROS Object Detection Node started. Waiting for image and detection data...')
        self.timer = self.create_timer(0.05, self.process_and_visualize) # ~20 fps visualization

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def detection_callback(self, msg):
        self.latest_detections = msg

    def process_and_visualize(self):
        if self.latest_image is not None and self.latest_detections is not None:
            display_image = self.latest_image.copy()
            for detection in self.latest_detections.detections:
                bbox = detection.bbox
                # Draw bounding box
                x_min = int(bbox.center.x - bbox.size_x / 2)
                y_min = int(bbox.center.y - bbox.size_y / 2)
                x_max = int(bbox.center.x + bbox.size_x / 2)
                y_max = int(bbox.center.y + bbox.size_y / 2)
                
                cv2.rectangle(display_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                # Add label and score
                for result in detection.results:
                    label = result.id
                    score = result.score
                    cv2.putText(display_image, f"{label}: {score:.2f}", (x_min, y_min - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            cv2.imshow("Isaac ROS Detections", display_image)
            cv2.waitKey(1)
            # Optionally clear detections to only show fresh ones, or let them persist
            # self.latest_detections = None 
        elif self.latest_image is not None:
            # If no detections but image is present, show raw image
            cv2.imshow("Isaac ROS Detections", self.latest_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    isaac_ros_object_detection = IsaacRosObjectDetection()
    rclpy.spin(isaac_ros_object_detection)
    isaac_ros_object_detection.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows() # Close any OpenCV windows

if __name__ == '__main__':
    main()
