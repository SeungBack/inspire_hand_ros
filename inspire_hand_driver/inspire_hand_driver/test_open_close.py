import numpy as np
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time, Duration

from sensor_msgs.msg import JointState

from inspire_hand_driver.hand_wrapper import InspireHand
from inspire_hand_msgs.msg import InspireHandCmd, InspireHandStat


class OpenCloseHandNode(Node):
    
    def __init__(self):
        super().__init__('open_close_hand_node')
        
        self.declare_parameter('speed', 1000)
        self.declare_parameter('force', 100)
        
        self.speed = self.get_parameter('speed').value
        self.force = self.get_parameter('force').value
        
        self._hand_pub = self.create_publisher(InspireHandCmd, '/hand/cmd', 10)
        
        self.timer = self.create_timer(1.0, self._timer_callback)
        
    def sleep_until(self, target_time: Time):
        """
        Sleep until the specified ROS Time is reached while still processing callbacks.
        
        Args:
            target_time: The target ROS Time to sleep until
            
        Returns:
            bool: True when the target time has been reached
        """
        while self.get_clock().now() < target_time:
            # Process any pending callbacks with a small timeout
            rclpy.spin_once(self, timeout_sec=0.01)
        return True
        
    def _timer_callback(self):
        """Timer callback to send open and close commands."""
        open_cmd = InspireHandCmd()
        open_cmd.angle = [1000, 1000, 1000, 1000, 1000, 1000] 
        open_cmd.force = [self.force] * 6
        open_cmd.speed = [self.speed] * 6
        self._hand_pub.publish(open_cmd)
        self.get_logger().info("Sending open command")
        self.sleep_until(self.get_clock().now() + Duration(seconds=2))
        
        close_cmd = InspireHandCmd()
        val = 0
        close_cmd.angle = [val, val, val, val, val, 1000] 
        close_cmd.force = [self.force] * 6
        close_cmd.speed = [self.speed] * 6
        self._hand_pub.publish(close_cmd)
        self.get_logger().info("Sending close command")
        self.sleep_until(self.get_clock().now() + Duration(seconds=2))


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    node = OpenCloseHandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()