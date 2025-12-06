import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Simple Robot Controller Node started.')
        self.timer = None

    def move_robot(self, linear_x, angular_z, duration_sec):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        self.get_logger().info(f'Sending command: Linear X={linear_x}, Angular Z={angular_z} for {duration_sec} seconds')

        end_time = self.get_clock().now().nanoseconds / 1e9 + duration_sec
        
        # Publish commands at a fixed rate
        self.timer = self.create_timer(0.1, lambda: self._publish_command(msg, end_time))

    def _publish_command(self, msg, end_time):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time < end_time:
            self.publisher_.publish(msg)
        else:
            # Stop the robot after duration
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            self.get_logger().info('Motion complete. Stopping robot.')
            if self.timer:
                self.timer.cancel()
            # Shut down the node after stopping the robot
            rclpy.shutdown()
            sys.exit(0) # Exit cleanly after command

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleRobotController()

    try:
        # Example: Move forward for 5 seconds
        controller.move_robot(0.2, 0.0, 5.0) 
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
