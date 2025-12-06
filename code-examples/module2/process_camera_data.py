import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.get_logger().info('Image subscriber node started. Waiting for image data...')

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        try:
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
            cv2.imshow("Camera Feed", current_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows() # Ensure all OpenCV windows are closed

if __name__ == '__main__':
    main()
