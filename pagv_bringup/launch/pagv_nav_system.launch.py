#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    agv_id_arg = DeclareLaunchArgument(
        'agv_id',
        default_value='AGV001',
        description='AGV ID for VDA5050'
    )
    
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='tcp://localhost:1883',
        description='MQTT broker address'
    )
    
    # Nodes
    amr_core_node = Node(
        package='pagv_amr_core',
        executable='amr_core_node',
        name='amr_core',
        output='screen',
        parameters=[{
            'agv_id': LaunchConfiguration('agv_id'),
            'mqtt_broker': LaunchConfiguration('mqtt_broker'),
            'control_rate': 20.0
        }]
    )
    
    motion_controller_node = Node(
        package='pagv_motion_controller',
        executable='motion_controller_node',
        name='motion_controller',
        output='screen'
    )
    
    localizer_node = Node(
        package='pagv_localizer',
        executable='localizer_node',
        name='localizer',
        output='screen',
        parameters=[{
            'wheel_radius': 0.25,
            'encoder_resolution': 1000.0,
            'wheelbase': 6.5
        }]
    )
    
    return LaunchDescription([
        agv_id_arg,
        mqtt_broker_arg,
        amr_core_node,
        motion_controller_node,
        localizer_node
    ])
