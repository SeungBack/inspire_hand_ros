import numpy as np
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from rclpy.action import ActionServer, CancelResponse, GoalResponse


from sensor_msgs.msg import JointState

from inspire_hand_driver.hand_wrapper import InspireHand
from inspire_hand_msgs.msg import InspireHandCmd, InspireHandStat

class InspireHandActionServer(Node):
    def __init__(self):
        """Initialize the ROS2 node and robotic hand connection."""
        super().__init__('inspire_hand_action_server')
        
        
        # self.create_subscription() # !TODO:
        self.create_subscription(InspireHandStat, '/hand/stat', self._update_hand_stat, 10)
        self._hand_pub = self.create_publisher(InspireHandCmd, '/hand/cmd', 10)

        self._stat = None        
        
        self._action_server = ActionServer()

    
    def get_time(self):
        time_msg = self.get_clock().now().to_msg()
        return float(time_msg.sec) + (float(time_msg.nanosec) * 1e-9)
    
    def _clip_cmd(self, cmd, lower, upper):
        """
        Clip command values to a specified range.
        
        Args:
            cmd (list): Command values
            lower (int): Lower bound
            upper (int): Upper bound
            
        Returns:
            list: Clipped command values
        """
        return [max(lower, min(upper, c)) for c in cmd]
    
    def _update_hand_stat(self, stat):
        self._stat = stat
       
        
    
    def _timer_callback(self):
        
        stat = self._update_hand_stat()
        js = self._update_hand_joint_state()
        self._hand_pub.publish(stat)
        self._hand_joint_state_pub.publish(js)
    
    def _update_hand_stat(self):
        
        stat = InspireHandStat()
        stat.header.stamp = self.get_clock().now().to_msg()
        # !TODO: support status, position
        stat.angle = self._hand_wrapper.get_angle()
        return stat
        
    def _update_hand_joint_state(self, dev=0):
        
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = "" #!TODO: check this
        js.name = [self.joint_prefix + j for j in self.joint_names]
        angle = self._hand_wrapper.get_angle()
        angle_rad = self.angle_val_to_rad(angle)
        js.position = angle_rad
        dt = self.get_time() - self._prev_js_time
        self._prev_js_time = self.get_time()
        js.velocity = [(angle_rad[i] - self._prev_js_angle[i]) / dt for i in range(len(angle_rad))]
        self._prev_js_angle = angle_rad
        return js
        
    
    def angle_val_to_rad(self, angle):
        # pinky to thumb
        # for angle 0-3: 0-1000 to 0-1.47
        # for angle 4 -> 0-0.6
        # for angle 5 -> 0-1.308
        angle[0] = angle[0] / 1000 * 1.47
        angle[1] = angle[1] / 1000 * 1.47
        angle[2] = angle[2] / 1000 * 1.47
        angle[3] = angle[3] / 1000 * 1.47
        angle[4] = angle[4] / 1000 * 0.6
        angle[5] = angle[5] / 1000 * 1.308
        return angle
        
    
    def destroy_node(self):
        """Clean up when the node is shutting down."""
        self.get_logger().info("Shutting down robotic hand controller")
        self._hand_wrapper.disconnect()
        super().destroy_node()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    node = InspireHandDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()