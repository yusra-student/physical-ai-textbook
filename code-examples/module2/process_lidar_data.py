import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('LiDAR subscriber node started.')

    def listener_callback(self, msg):
        # Process LiDAR data here
        self.get_logger().info(f'Received LiDAR scan:')
        self.get_logger().info(f'  Header: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}s, Frame ID: {msg.header.frame_id}')
        self.get_logger().info(f'  Angle Min: {msg.angle_min}, Max: {msg.angle_max}, Increment: {msg.angle_increment}')
        self.get_logger().info(f'  Range Min: {msg.range_min}, Max: {msg.range_max}')
        # Print a few sample range readings
        if len(msg.ranges) > 0:
            self.get_logger().info(f'  First 5 Ranges: {msg.ranges[0:5]}')
            self.get_logger().info(f'  Last 5 Ranges: {msg.ranges[-5:]}')
        else:
            self.get_logger().info('  No range data available.')

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
