import numpy as np
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import JointState

from inspire_hand_driver.hand_wrapper import InspireHand
from inspire_hand_msgs.msg import InspireHandCmd, InspireHandStat

class InspireHandDriver(Node):
    """
    ROS2 Node to control a robotic hand device via serial communication.
    Publishes joint states and provides control services.
    Compatible with Inspire Hand and similar models.
    """
    
    def __init__(self):
        """Initialize the ROS2 node and robotic hand connection."""
        super().__init__('inspire_hand_driver')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('hand_id', 1)
        self.declare_parameter('publish_rate', 50)  # Hz
        self.declare_parameter('hand_type', 'left') # List of hand types ['left', 'right']
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.hand_id = self.get_parameter('hand_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.hand_type = self.get_parameter('hand_type').value
        self.prefix = '/' + self.hand_type
        
        
        self._hand_wrapper = InspireHand(self.port, self.baudrate, self.hand_id)
        success = self._hand_wrapper.connect()
        if not success:
            self.get_logger().error("Failed to connect to hand. Exiting...")
            sys.exit(1)
        else:
            self.get_logger().info("Hand is connected using port: {}, baud: {}, id: {}".format(self.port, self.baudrate, self.hand_id))
        
        self._speed_cmd = [0, 0, 0, 0, 0, 0]
        self._force_cmd = [0, 0, 0, 0, 0, 0]
        self._prev_js_angle = [0, 0, 0, 0, 0, 0]
        self._prev_js_time = self.get_time()
        self.joint_names = ['pinky_proximal_joint', 'ring_proximal_joint', 'middle_proximal_joint', 'index_proximal_joint', 'thumb_proximal_pitch_joint', 'thumb_proximal_yaw_joint']
        
        
        # self.create_subscription() # !TODO:
        self.create_subscription(InspireHandCmd, self.prefix  + '/hand/cmd', self._update_hand_cmd, self.publish_rate)
        self._hand_pub = self.create_publisher(InspireHandStat, self.prefix + '/hand/stat', self.publish_rate)
        self._hand_joint_state_pub = self.create_publisher(JointState, self.prefix + '/hand/joint_states', self.publish_rate)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._timer_callback)
        

    
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
    
    def _update_hand_cmd(self, cmd):
        # !TODO: add emergency stop
        angle = self._clip_cmd(cmd.angle, 0, 1000)
        force = self._clip_cmd(cmd.force, 0, 1000)
        speed = self._clip_cmd(cmd.speed, 0, 1000)
        
        # update only if changed
        if force != self._force_cmd:
            self._hand_wrapper.set_force(force)
            self._force_cmd = force
        if speed != self._speed_cmd:
            self._hand_wrapper.set_speed(speed)
            self._speed_cmd = speed
        
        self._hand_wrapper.set_angle(angle)
        
    
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
        js.name = [self.hand_type + j for j in self.joint_names]
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
        angle[0] = 1.47 - angle[0] / 1000 * 1.47
        angle[1] = 1.47 - angle[1] / 1000 * 1.47
        angle[2] = 1.47 - angle[2] / 1000 * 1.47
        angle[3] = 1.47 - angle[3] / 1000 * 1.47
        angle[4] = 0.6 - angle[4] / 1000 * 0.6
        angle[5] = 1.308 - angle[5] / 1000 * 1.308
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