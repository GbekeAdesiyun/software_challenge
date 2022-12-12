"""Lauch all the components in the software_training_assignment package"""
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name= 'my_container',
        namespace= '',
        package= 'rclcpp_components',
        executable= 'component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='software_training_assignment',
                plugin='software_training_assignment::Clear_turtles_client',
                name='clear_turtles_client'),
            ComposableNode(
                package= 'software_training_assignment',
                plugin= 'software_training_assignment::Circular_move_publisher'
                name='circular_move_publisher'),
            ComposableNode(
                package= 'software_training_assignment',
                plugin= 'software_training_assignment::Spawn_turtle'
                name='spawn_turtle'),
            ComposableNode(
                package= 'software_training_assignment',
                plugin= 'software_training_assignment::Reset_moving_turtle_service'
                name='reset_moving_turtle_service'),
            ComposableNode(
                package= 'software_training_assignment',
                plugin= 'software_training_assignment::Custom_publisher'
                name='custom_publisher'),
            ComposableNode(
                package= 'software_training_assignment',
                plugin= 'software_training_assignment::Moving_turtle_action_server'
                name='moving_turtle_action_server'),          
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])