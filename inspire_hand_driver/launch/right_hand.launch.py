from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the inspire hand driver node."""
    
    # Declare all launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the Inspire Hand'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for serial communication'
    )
    
    hand_id_arg = DeclareLaunchArgument(
        'hand_id',
        default_value='1',
        description='ID of the Inspire Hand'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100',
        description='Rate to publish hand status and joint states (Hz)'
    )
    
    
    hand_type_arg = DeclareLaunchArgument(
        'hand_type',
        default_value='right',
        description='Type of hand (right or right)'
    )
    
    # Create node
    node = Node(
        package='inspire_hand_driver',
        executable='inspire_hand_driver',
        name='inspire_hand_driver',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'hand_id': LaunchConfiguration('hand_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'hand_type': LaunchConfiguration('hand_type')
        }],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        port_arg,
        baudrate_arg,
        hand_id_arg,
        publish_rate_arg,
        hand_type_arg,
        node
    ])