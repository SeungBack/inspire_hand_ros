from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    hand_type = LaunchConfiguration('hand_type', default='left')
    gui = LaunchConfiguration('gui', default='false')
    
    # Paths
    pkg_share = FindPackageShare('inspire_hand_description')
    left_urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'inspire_hand_left.urdf'])
    right_urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'inspire_hand_right.urdf'])
    
    # Robot state publisher for left hand
    left_robot_state_publisher_node = Node(
        condition=IfCondition(PythonExpression([
            "'", hand_type, "' == 'left' or '", hand_type, "' == 'both'"
        ])),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='left_hand_state_publisher',
        namespace='left_hand',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['xacro ', left_urdf_path]), value_type=str),
        }],
    )
    
    # Joint state publisher for left hand
    # left_joint_state_publisher_gui = Node(
    #     condition=IfCondition(PythonExpression([
    #         "('", hand_type, "' == 'left' or '", hand_type, "' == 'both') and '", gui, "' == 'true'"
    #     ])),
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     namespace='left_hand',
    #     output='screen',
    # )
    
    # # Robot state publisher for right hand
    # right_robot_state_publisher_node = Node(
    #     condition=IfCondition(PythonExpression([
    #         "'", hand_type, "' == 'right' or '", hand_type, "' == 'both'"
    #     ])),
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='right_hand_state_publisher',
    #     namespace='right_hand',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'robot_description': ParameterValue(Command(['xacro ', right_urdf_path]), value_type=str),
    #         # 'frame_prefix': 'right_hand/'  # Add frame prefix for right hand
    #     }],
    # )
    
    # # Joint state publisher for right hand
    # right_joint_state_publisher_gui = Node(
    #     condition=IfCondition(PythonExpression([
    #         "('", hand_type, "' == 'right' or '", hand_type, "' == 'both') and '", gui, "' == 'true'"
    #     ])),
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     namespace='right_hand',
    #     output='screen',
    # )
    
    # # If both hands are loaded, we need a joint state publisher to combine the joint states
    # joint_state_publisher = Node(
    #     condition=IfCondition(PythonExpression(["'", hand_type, "' == 'both'"])),
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'source_list': ['/left_hand/joint_states', '/right_hand/joint_states']
    #     }],
    # )
    
    # RViz with a configuration
    rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'view_hand.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    ))
    ld.add_action(DeclareLaunchArgument(
        'hand_type',
        default_value='left',
        description='Hand type: left, right, or both'
    ))
    ld.add_action(DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Enable joint state publisher GUI'
    ))
    
    # Add nodes
    ld.add_action(left_robot_state_publisher_node)
    # ld.add_action(left_joint_state_publisher_gui)
    # ld.add_action(right_robot_state_publisher_node)
    # ld.add_action(right_joint_state_publisher_gui)
    # ld.add_action(joint_state_publisher)
    ld.add_action(rviz_node)
    
    return ld