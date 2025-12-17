from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    writer = Node(
        package='turtle_writer',
        executable='write_words',
        name='turtle_writer',
        output='screen'
    )

    return LaunchDescription([
        turtlesim,
        writer
    ])
