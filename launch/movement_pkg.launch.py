from launch import LaunchDescription
from launch_ros.actions import Node

# customize launch file imports below

# end launch file import customization

def generate_launch_description():
    launch_description = LaunchDescription()
    launch_description.add_action(Node(
        package='movement_pkg',
        executable='movement',
        name='movement'
    ))

    # customize launch file below

    # end launch file customization
    
    return launch_description
