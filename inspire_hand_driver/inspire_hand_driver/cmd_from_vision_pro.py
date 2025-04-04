import os
import numpy as np
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time, Duration
import time
from sensor_msgs.msg import JointState

from inspire_hand_driver.hand_wrapper import InspireHand
from inspire_hand_msgs.msg import InspireHandCmd, InspireHandStat
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .utils.retargeting_utils import initialize_retargeting, retarget_timestamp, filter_data

from avp_stream import VisionProStreamer
import time


class VisionPro(Node):
    
    def __init__(self):
        super().__init__('vision_pro_node')
        
        self.declare_parameter('speed', 1000)
        self.declare_parameter('force', 100)
        self.declare_parameter('publish_rate', 30)  # Hz
        self.declare_parameter('avp_ip', "192.168.1.104")
        self.declare_parameter('hand_type', 'both') # List of hand types ['left', 'right', 'both']

        self.speed = self.get_parameter('speed').value
        self.force = self.get_parameter('force').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.avp_ip = self.get_parameter('avp_ip').value
        self.hand_type = self.get_parameter('hand_type').value
        
        # initialize the joint retargeting
        robot_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'inspire_hand_description')
        
        if self.hand_type in ['left', 'both']:      
            self._left_hand_pub = self.create_publisher(InspireHandCmd, '/left/hand/cmd', self.publish_rate)
            self.left_retargeting = initialize_retargeting('inspire', 'left', robot_dir)
            
        if self.hand_type in ['right', 'both']:
            self._right_hand_pub = self.create_publisher(InspireHandCmd, '/right/hand/cmd', self.publish_rate)
            self.right_retargeting = initialize_retargeting('inspire', 'right', robot_dir)
        
        self.timer = self.create_timer(1.0/self.publish_rate, self._timer_callback)
        self.ros_joint_names = ['pinky_proximal_joint', 'ring_proximal_joint', 'middle_proximal_joint', 'index_proximal_joint', 'thumb_proximal_pitch_joint', 'thumb_proximal_yaw_joint']
    
        # vision pro stream
        self.stream = VisionProStreamer(ip = self.avp_ip, record = True)
        
        # tf
        # self.tf_broadcaster = TransformBroadcaster(self)
        # # For static transforms (optional)
        # self.static_broadcaster = StaticTransformBroadcaster(self)
        # # Set up a timer to publish the transform at a fixed rate
        # self.counter = 0

        
    def rad_to_angle_val(self, rad):
        # for angle 0-3: 1.47-0 to 0-1000
        # for angle 4: 0.6-0.0 to 0-1000
        # for angle 5: 1.308-0.0 to 0-1000
        rad[0:4] = np.clip(rad[0:4], 0, 1.47)
        rad[4] = np.clip(rad[4], 0, 0.6)
        rad[5] = np.clip(rad[5], 0, 1.308)
        rad[0] = int((1.47 - rad[0])/1.47 * 1000)
        rad[1] = int((1.47 - rad[1])/1.47 * 1000)
        rad[2] = int((1.47 - rad[2])/1.47 * 1000)
        rad[3] = int((1.47 - rad[3])/1.47 * 1000)
        rad[4] = int((0.6 - rad[4])/0.6 * 1000)
        rad[5] = int((1.308 - rad[5])/1.308 * 1000)
        return rad
        
    def _timer_callback(self):
        """Timer callback to send open and close commands."""
        
        single_data = self.stream.latest
        if self.hand_type == 'left':
            result = retarget_timestamp(self.left_retargeting, single_data=single_data)
        elif self.hand_type == 'right':
            result = retarget_timestamp(self.right_retargeting, single_data=single_data)
        elif self.hand_type == 'both':
            result = retarget_timestamp(self.left_retargeting, self.right_retargeting, single_data=single_data)
        
        if self.hand_type in ['left', 'both']:
            # convert the left_qpos to the ros joint names
            left_qpos = result['left_qpos'].tolist() # (12,)
            left_qpos_idx = [
                self.left_retargeting.joint_names.index(name) for name in self.ros_joint_names
            ]
            left_qpos_ros = [left_qpos[i] for i in left_qpos_idx]
            left_angle = self.rad_to_angle_val(left_qpos_ros)
            
            # publish the command
            cmd = InspireHandCmd()
            cmd.angle = left_angle
            cmd.force = [self.force] * 6
            cmd.speed = [self.speed] * 6
            self._left_hand_pub.publish(cmd)
            
        if self.hand_type in ['right', 'both']:
            # convert the right_qpos to the ros joint names
            right_qpos = result['right_qpos'].tolist()
            right_qpos_idx = [
                self.right_retargeting.joint_names.index(name) for name in self.ros_joint_names
            ]
            right_qpos_ros = [right_qpos[i] for i in right_qpos_idx]
            right_angle = self.rad_to_angle_val(right_qpos_ros)
            # publish the command
            cmd = InspireHandCmd()
            cmd.angle = right_angle
            cmd.force = [self.force] * 6
            cmd.speed = [self.speed] * 6
            self._right_hand_pub.publish(cmd)
        

        
        # open_cmd = InspireHandCmd()
        # open_cmd.angle = [1000, 1000, 1000, 1000, 1000, 1000] 
        # open_cmd.force = [self.force] * 6
        # open_cmd.speed = [self.speed] * 6
        # self._hand_pub.publish(open_cmd)
        # self.get_logger().info("Sending open command")
        # self.sleep_until(self.get_clock().now() + Duration(seconds=2))
        
        # close_cmd = InspireHandCmd()
        # val = 500
        # close_cmd.angle = [val, val, val, val, val, 1000] 
        # close_cmd.force = [self.force] * 6
        # close_cmd.speed = [self.speed] * 6
        # self._hand_pub.publish(close_cmd)
        # self.get_logger().info("Sending close command")
        # self.sleep_until(self.get_clock().now() + Duration(seconds=2))


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    node = VisionPro()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()