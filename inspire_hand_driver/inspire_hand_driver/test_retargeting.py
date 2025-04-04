import numpy as np
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time, Duration

from sensor_msgs.msg import JointState

from inspire_hand_driver.hand_wrapper import InspireHand
from inspire_hand_msgs.msg import InspireHandCmd, InspireHandStat


class Test(Node):
    
    def __init__(self):
        super().__init__('test')
        
        self.declare_parameter('speed', 1000)
        self.declare_parameter('force', 10)
        self.declare_parameter('publish_rate', 30.0)  # Hz

        self.speed = self.get_parameter('speed').value
        self.force = self.get_parameter('force').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        self._hand_pub = self.create_publisher(InspireHandCmd, '/hand/cmd', 30)
        
        self.timer = self.create_timer(1.0/self.publish_rate, self._timer_callback)
        
        data_path = '/home/seung/ros2_ws/src/inspire_hand_ros/inspire_hand_driver/data/retargeted.pkl'
        self.data = np.load(data_path, allow_pickle=True)['data']
        
        self.left_joint_names = np.load(data_path, allow_pickle=True)['meta_data']['joint_names'][0]
        self.right_joint_names = np.load(data_path, allow_pickle=True)['meta_data']['joint_names'][1]
        
        self.ros_joint_names = ['pinky_proximal_joint', 'ring_proximal_joint', 'middle_proximal_joint', 'index_proximal_joint', 'thumb_proximal_pitch_joint', 'thumb_proximal_yaw_joint']
        
        self.length = len(self.data)
        self.index = 0
        
        
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
        
        left_qpos = self.data['left_qpos'][self.index].tolist() # (12,)
        # right_qpos = self.data['right_qpos'][self.index] # (4, 4)
        # left_pose = self.data['left_pose'][self.index]
        # right_pose = self.data['right_qvel'][self.index]
        
        left_qpos_idx = [
            self.left_joint_names.index(name) for name in self.ros_joint_names
        ]
        left_qpos_ros = [left_qpos[i] for i in left_qpos_idx]
        
        angle = self.rad_to_angle_val(left_qpos_ros)
        cmd = InspireHandCmd()
        cmd.angle = angle
        # cmd.angle = [angle[0], angle[1], angle[2], angle[3], 1000, 1000]
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
    node = Test()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()