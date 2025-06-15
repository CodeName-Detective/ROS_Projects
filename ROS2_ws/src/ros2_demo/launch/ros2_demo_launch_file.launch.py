from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

TOPIC = 'introducer'
SERVICE = 'add_two_ints'

def generate_launch_description():
    
    parameter_file = os.path.join(
        get_package_share_directory('ros2_demo'),
        'parameters.yaml'
    )
    
    producer_node = Node(
        package = 'ros2_demo',
        executable = 'publisher_executable',
        output = 'screen',
        emulate_tty = True,
        parameters = [
            {'topic': TOPIC}
        ]
    )
    
    consumer_node = Node(
        package = 'ros2_demo',
        executable = 'subscriber_executable',
        output = 'screen',
        emulate_tty = True,
        parameters = [
            {'topic': TOPIC}
        ]
    )
    
    service_node = Node(
        package = 'ros2_demo',
        executable = 'server_executable',
        name = 'server_node', # has precedence over the default value provided in the Script. Can also be used to change it at runtime
        output = 'screen',
        emulate_tty = True,
        parameters = [parameter_file]
    )
    
    act_service_node = Node(
        package = 'ros2_demo',
        executable = 'action_server_executable',
        output = 'screen',
        emulate_tty = True,
        parameters = [parameter_file]
    )
    
    return LaunchDescription([
        producer_node,
        consumer_node,
        service_node,
        act_service_node
    ]
    )