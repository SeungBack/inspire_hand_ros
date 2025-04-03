from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for the robotic hand controller."""
    
    
    pkg_share = FindPackageShare('inspire_hand_ros')
    rviz_config_path = PathJoinSubstitution([pkg_share, 'config.rviz'])
    left_urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'inspire_hand_left.urdf'])

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the robotic hand'
        ),
        
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baudrate for serial communication'
        ),
        
        DeclareLaunchArgument(
            'hand_id',
            default_value='1',
            description='ID of the hand device'
        ),
        
        DeclareLaunchArgument(
            'publish_rate',
            default_value='30.0',
            description='Rate (Hz) at which to publish joint states'
        ),
        
        DeclareLaunchArgument(
            'joint_prefix',
            default_value='hand_',
            description='Prefix for joint names'
        ),
        
        # Robotic hand controller node
        Node(
            package='inspire_hand_ros',
            executable='hand_wrapper',
            name='hand_controller',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'hand_id': LaunchConfiguration('hand_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'joint_prefix': LaunchConfiguration('joint_prefix')
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', left_urdf_path]), value_type=str),
            }],
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        )
    ])