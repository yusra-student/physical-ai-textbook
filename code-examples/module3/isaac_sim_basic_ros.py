import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class IsaacSimJointStatePublisher(Node):
    def __init__(self):
        super().__init__('isaac_sim_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # Publish every 100ms
        self.joint_names = ['joint1', 'joint2'] # Example joint names
        self.joint_positions = [0.0, 0.0]
        self.start_time = time.time()
        self.get_logger().info(f'Isaac Sim Joint State Publisher node started for joints: {self.joint_names}')

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()
        elapsed_time = time.time() - self.start_time

        # Simulate joint movement
        self.joint_positions[0] = math.sin(elapsed_time * 0.5) * 0.5 # Oscillate joint1
        self.joint_positions[1] = math.cos(elapsed_time * 0.7) * 0.3 # Oscillate joint2

        msg = JointState()
        msg.header.stamp = current_time
        msg.name = self.joint_names
        msg.position = self.joint_positions
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint states: {self.joint_positions}')

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = IsaacSimJointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
