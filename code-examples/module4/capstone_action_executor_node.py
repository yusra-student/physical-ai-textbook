import rclpy
from rclpy.node import Node
from std_msgs.msg import String # For receiving action plans
from geometry_msgs.msg import Twist # For differential drive robot movement
from rcl_interfaces.msg import ParameterDescriptor
import json
import time

class CapstoneActionExecutorNode(Node):
    def __init__(self):
        super().__init__('capstone_action_executor_node')
        self.declare_parameter('robot_type', 'differential_drive', 
                               ParameterDescriptor(description='Type of robot to control (e.g., differential_drive, humanoid)'))
        self.robot_type = self.get_parameter('robot_type').value

        self.action_subscription = self.create_subscription(
            String,
            '/robot_actions', # Topic from LLM Task Planner
            self.action_callback,
            10
        )
        
        # Publishers based on robot type
        if self.robot_type == 'differential_drive':
            self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # elif self.robot_type == 'humanoid':
        #     self.joint_command_publisher = self.create_publisher(JointState, '/joint_commands', 10)
        #     self.nav_goal_publisher = self.create_publisher(PoseStamped, '/navigate_to_pose', 10)

        self.get_logger().info(f'Capstone Action Executor Node started for {self.robot_type} robot. Waiting for action plans...')
        self.current_action_plan = []
        self.action_timer = None
        self.action_start_time = 0.0

    def action_callback(self, msg: String):
        self.get_logger().info(f'Received action plan: {msg.data}')
        try:
            self.current_action_plan = json.loads(msg.data)
            self.execute_next_action()
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse action plan JSON: {e}')
        
    def execute_next_action(self):
        if not self.current_action_plan:
            self.get_logger().info('Action plan completed.')
            return

        action_data = self.current_action_plan.pop(0)
        action_name = action_data.get("action")
        parameters = action_data.get("parameters", {})
        
        self.get_logger().info(f'Executing action: {action_name} with params: {parameters}')

        if self.robot_type == 'differential_drive':
            if action_name == "move_to":
                location = parameters.get("location")
                self._move_differential_drive_robot(location)
            elif action_name == "pick_up" or action_name == "place_down" or action_name == "open" or action_name == "close" or action_name == "detect_object":
                self.get_logger().warn(f'Action "{action_name}" not supported for differential_drive robot. Skipping.')
                self.execute_next_action() # Continue with next action
            else:
                self.get_logger().warn(f'Unknown action "{action_name}". Skipping.')
                self.execute_next_action() # Continue with next action
        # elif self.robot_type == 'humanoid':
        #     # Implement humanoid specific actions here
        #     pass

    def _move_differential_drive_robot(self, location: str):
        # This is a very simplified movement simulation
        self.get_logger().info(f'Simulating moving to {location} for a differential drive robot.')
        twist_msg = Twist()
        twist_msg.linear.x = 0.5 # Move forward
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Simulate movement duration, then stop and execute next action
        self.action_start_time = self.get_clock().now().nanoseconds / 1e9
        if self.action_timer:
            self.action_timer.cancel()
        self.action_timer = self.create_timer(1.0, self._check_move_completion) # Check every second

    def _check_move_completion(self):
        elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.action_start_time
        simulated_duration = 5.0 # Assume 5 seconds to "move_to" for simplicity

        if elapsed_time < simulated_duration:
            self.get_logger().info(f'Moving... {simulated_duration - elapsed_time:.1f} seconds remaining.')
            # Keep publishing twist_msg if needed, or rely on a persistent command
        else:
            self.get_logger().info('Movement completed.')
            twist_msg = Twist() # Stop the robot
            self.cmd_vel_publisher.publish(twist_msg)
            if self.action_timer:
                self.action_timer.cancel()
                self.action_timer = None
            self.execute_next_action() # Execute the next action in the plan

def main(args=None):
    rclpy.init(args=args)
    action_executor_node = CapstoneActionExecutorNode()
    rclpy.spin(action_executor_node)
    action_executor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
