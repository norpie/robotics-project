from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # logger_node = Node(
    #     package='project',
    #     executable='logger',
    #     output='screen')

    navigator_node = Node(
        package='project',
        executable='navigator',
        output='screen')

    explorer_node = Node(
        package='project',
        executable='explorer',
        output='screen')


    # input_node = Node(
    #     package='project',
    #     executable='input',
    #     output='screen')

    # ld.add_action(logger_node)
    ld.add_action(navigator_node)
    ld.add_action(explorer_node)
    # ld.add_action(input_node)

    return ld
