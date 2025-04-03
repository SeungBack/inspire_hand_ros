import pickle
from pathlib import Path
from typing import List

import numpy as np
import tqdm
import tyro
from dex_retargeting.constants import (
    RobotName,
    RetargetingType,
    HandType,
    get_default_config_path,
)
from dex_retargeting.retargeting_config import RetargetingConfig
from dex_retargeting.seq_retarget import SeqRetargeting

OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ]
)

OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0],
    ]
)

OPERATOR2AVP_RIGHT = OPERATOR2MANO_RIGHT

OPERATOR2AVP_LEFT = np.array(
    [
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0],
    ]
)


def three_mat_mul(left_rot: np.ndarray, mat: np.ndarray, right_rot: np.ndarray):
    result = np.eye(4)
    rotation = left_rot @ mat[:3, :3] @ right_rot
    pos = left_rot @ mat[:3, 3]
    result[:3, :3] = rotation
    result[:3, 3] = pos
    return result


def two_mat_batch_mul(batch_mat: np.ndarray, left_rot: np.ndarray):
    result = np.tile(np.eye(4), [batch_mat.shape[0], 1, 1])
    result[:, :3, :3] = np.matmul(left_rot[None, ...], batch_mat[:, :3, :3])
    result[:, :3, 3] = batch_mat[:, :3, 3] @ left_rot.T
    return result


def joint_avp2hand(finger_mat: np.ndarray):
    finger_index = np.array([0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24])
    finger_mat = finger_mat[finger_index]
    return finger_mat


def filter_data(data: List, fps, duration):
    init_time = data[0]["time"]
    all_times = np.array([d["time"] for d in data]) - init_time
    step = 1.0 / fps
    new_data = []
    for i in range(fps * duration):
        current_time = i * step
        diff = np.abs(all_times - current_time)
        best_match = np.argmin(diff)
        new_data.append(data[best_match])
    return new_data

def retarget_timestamp(
    left_retargeting: SeqRetargeting = None,
    right_retargeting: SeqRetargeting = None,
    single_data: dict = None
):
    """
    Retargets a single timestamp of hand pose data from Apple VisionPro to robot joint positions.
    Handles cases where only left or right hand data is available.

    Args:
        left_retargeting: The left hand retargeting object. Can be None if not needed.
        right_retargeting: The right hand retargeting object. Can be None if not needed.
        single_data: A dictionary containing the pose data for a single timestamp.
            Expected to have keys like 'left_fingers', 'right_fingers', 'left_wrist', 'right_wrist'.

    Returns:
        dict: A dictionary containing the retargeted data with keys:
            - 'left_qpos': Left hand joint positions (if left hand is processed)
            - 'right_qpos': Right hand joint positions (if right hand is processed)
            - 'left_pose': Left wrist pose (if left hand is processed)
            - 'right_pose': Right wrist pose (if right hand is processed)
    """
    result = {}
    
    # Process left hand if data and retargeting object are available
    if left_retargeting is not None and single_data is not None and 'left_fingers' in single_data and 'left_wrist' in single_data:
        # Apply transformation to finger joints
        joint_pose = two_mat_batch_mul(single_data["left_fingers"], OPERATOR2AVP_LEFT.T)
        joint_pos = joint_avp2hand(joint_pose)[:, :3, 3]
        
        # Get reference values for retargeting
        indices = left_retargeting.optimizer.target_link_human_indices
        origin_indices = indices[0, :]
        task_indices = indices[1, :]
        ref_value = joint_pos[task_indices, :] - joint_pos[origin_indices, :]
        # Retarget to robot joint positions
        left_qpos = left_retargeting.retarget(ref_value)
        result['left_qpos'] = left_qpos
        
        # Process left wrist pose
        wrist_pose = single_data["left_wrist"][0].copy()
        wrist_pose[:3, :3] = wrist_pose[:3, :3] @ OPERATOR2AVP_LEFT
        result['left_pose'] = wrist_pose.copy()
    
    # Process right hand if data and retargeting object are available
    if right_retargeting is not None and single_data is not None and 'right_fingers' in single_data and 'right_wrist' in single_data:
        # Apply transformation to finger joints
        joint_pose = two_mat_batch_mul(single_data["right_fingers"], OPERATOR2AVP_RIGHT.T)
        joint_pos = joint_avp2hand(joint_pose)[:, :3, 3]
        
        # Get reference values for retargeting
        indices = right_retargeting.optimizer.target_link_human_indices
        origin_indices = indices[0, :]
        task_indices = indices[1, :]
        ref_value = joint_pos[task_indices, :] - joint_pos[origin_indices, :]
        
        # Retarget to robot joint positions
        right_qpos = right_retargeting.retarget(ref_value)
        result['right_qpos'] = right_qpos
        
        # Process right wrist pose
        wrist_pose = single_data["right_wrist"][0].copy()
        wrist_pose[:3, :3] = wrist_pose[:3, :3] @ OPERATOR2AVP_RIGHT
        result['right_pose'] = wrist_pose.copy()
    
    return result


# Function to initialize retargeting for left or right hand or both
def initialize_retargeting(robot_name, hand_type=None, robot_dir=None):
    """
    Initialize the retargeting objects for specified hand(s).
    
    Args:
        robot_name: The identifier for the robot.
        hand_type: String ('left', 'right'), list of strings, or None for both hands.
        robot_dir: Optional directory for the robot URDF files.
        
    Returns:
        If hand_type is None or contains both 'left' and 'right': tuple of (left_retargeting, right_retargeting)
        If hand_type is 'left': left_retargeting only
        If hand_type is 'right': right_retargeting only
        If hand_type is ['left', 'right']: tuple of (left_retargeting, right_retargeting)
    """
    if isinstance(robot_name, str):
        robot_name = RobotName[robot_name]
    if robot_dir is not None:
        RetargetingConfig.set_default_urdf_dir(str(robot_dir))
    
    left_retargeting = None
    right_retargeting = None
    
    # Convert string to list for consistent handling
    if isinstance(hand_type, str):
        hand_type = [hand_type]
    
    # If hand_type is None, process both hands
    if hand_type is None:
        hand_type = ['left', 'right']
    
    # Initialize left hand retargeting if needed
    if 'left' in hand_type:
        left_config_path = get_default_config_path(robot_name, RetargetingType.dexpilot, HandType.left)
        left_retargeting = RetargetingConfig.load_from_file(left_config_path).build()
    
    # Initialize right hand retargeting if needed
    if 'right' in hand_type:
        right_config_path = get_default_config_path(robot_name, RetargetingType.dexpilot, HandType.right)
        right_retargeting = RetargetingConfig.load_from_file(right_config_path).build()
    
    # Return based on what was requested
    if hand_type == ['left']:
        return left_retargeting
    elif hand_type == ['right']:
        return right_retargeting
    else:
        return left_retargeting, right_retargeting