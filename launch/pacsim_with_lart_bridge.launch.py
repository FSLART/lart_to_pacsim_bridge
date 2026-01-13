from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pacsim',
            executable='simulator_node',
            name='pacsim',
            parameters=[
                {"discipline": "vehicle"},
            ],
            output='screen',
        ),
        Node(
            package='lart_to_pacsim_bridge',
            executable='lart_to_pacsim_bridge_node',
            name='lart_to_pacsim_bridge',
            parameters=[
                {"lart_topic:": "/lart/dynamics_cmd"},
                {"publish_wheelspeed": False},
                {"enable_torque_min": False},
            ],
            output='screen',
        ),
    ])