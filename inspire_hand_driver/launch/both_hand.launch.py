from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Generate launch description for both left and right Inspire Hand drivers."""
    
    # Declare common launch arguments
    left_port_arg = DeclareLaunchArgument(
        'left_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the left Inspire Hand'
    )
    
    right_port_arg = DeclareLaunchArgument(
        'right_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the right Inspire Hand'
    )
    
    left_hand_id_arg = DeclareLaunchArgument(
        'left_hand_id',
        default_value='1',
        description='ID of the left Inspire Hand'
    )
    
    right_hand_id_arg = DeclareLaunchArgument(
        'right_hand_id',
        default_value='1',
        description='ID of the right Inspire Hand'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for serial communication'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50',
        description='Rate to publish hand status and joint states (Hz)'
    )
    
    # Configure the left hand node
    left_hand_node = Node(
        package='inspire_hand_driver',
        executable='inspire_hand_driver',
        name='inspire_hand_driver',
        parameters=[{
            'port': LaunchConfiguration('left_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'hand_id': LaunchConfiguration('left_hand_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'hand_type': 'left'
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Configure the right hand node
    right_hand_node = Node(
        package='inspire_hand_driver',
        executable='inspire_hand_driver',
        name='inspire_hand_driver',
        parameters=[{
            'port': LaunchConfiguration('right_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'hand_id': LaunchConfiguration('right_hand_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'hand_type': 'right'
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Group the left hand in a namespace
    left_hand_group = GroupAction([
        PushRosNamespace('left_hand'),
        left_hand_node
    ])
    
    # Group the right hand in a namespace
    right_hand_group = GroupAction([
        PushRosNamespace('right_hand'),
        right_hand_node
    ])
    
    return LaunchDescription([
        left_port_arg,
        right_port_arg,
        left_hand_id_arg,
        right_hand_id_arg,
        baudrate_arg,
        publish_rate_arg,
        left_hand_group,
        right_hand_group
    ])