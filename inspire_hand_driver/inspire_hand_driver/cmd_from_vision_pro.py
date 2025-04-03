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

from .utils.retargeting_utils import initialize_retargeting, retarget_timestamp, filter_data

from avp_stream import VisionProStreamer


class VisionPro(Node):
    
    def __init__(self):
        super().__init__('test')
        
        self.declare_parameter('speed', 1000)
        self.declare_parameter('force', 1000)
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('avp_ip', "192.168.1.104")

        self.speed = self.get_parameter('speed').value
        self.force = self.get_parameter('force').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.avp_ip = self.get_parameter('avp_ip').value
        
        self._hand_pub = self.create_publisher(InspireHandCmd, '/hand/cmd', 50)
        
        self.timer = self.create_timer(1.0/self.publish_rate, self._timer_callback)
        
        
        self.ros_joint_names = ['pinky_proximal_joint', 'ring_proximal_joint', 'middle_proximal_joint', 'index_proximal_joint', 'thumb_proximal_pitch_joint', 'thumb_proximal_yaw_joint']
    
        
        # initialize the joint retargeting
        robot_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'inspire_hand_description')
        self.left_retargeting = initialize_retargeting('inspire', 'left', robot_dir)
    
        self.index = 0
        self.sample_data = np.load('/home/seung/ros2_ws/src/inspire_hand_ros/inspire_hand_driver/data/offline_avp_stream.pkl', allow_pickle=True)
        self.sample_data = filter_data(self.sample_data, fps=30, duration=15)
        
        self.stream = VisionProStreamer(ip = self.avp_ip, record = True)

        
        
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
        # single_data = self.sample_data[self.index]
        result = retarget_timestamp(self.left_retargeting, single_data=single_data)
        
        # right_qpos = self.data['right_qpos'][self.index] # (4, 4)
        # left_pose = self.data['left_pose'][self.index]
        # right_pose = self.data['right_qvel'][self.index]
        
        # convert the left_qpos to the ros joint names
        left_qpos = result['left_qpos'].tolist() # (12,)
        left_qpos_idx = [
            self.left_retargeting.joint_names.index(name) for name in self.ros_joint_names
        ]
        left_qpos_ros = [left_qpos[i] for i in left_qpos_idx]
        angle = self.rad_to_angle_val(left_qpos_ros)
        
        # publish the command
        cmd = InspireHandCmd()
        # cmd.angle = [angle[0], angle[1], angle[2], angle[3], 1000, 1000]
        cmd.angle = angle
        cmd.force = [self.force] * 6
        cmd.speed = [self.speed] * 6
        self._hand_pub.publish(cmd)
        self.index += 1
        
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