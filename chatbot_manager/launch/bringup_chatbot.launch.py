import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pathlib import Path

def generate_launch_description():
    ### Set launch params ###
    params_file = LaunchConfiguration('params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('chatbot_manager'), 'params', 'bringup_chatbot_params.yaml'),
        description='Full path to the ROS2 parameters file to use for bringing up chatbot nodes')
    
    
    ### Create nodes ###
    bringup_chatbot_nodes = GroupAction([
        Node(
            package='chatbot_manager',
            executable='chatbot_manager',
            name='chatbot_manager_node'),
            
        Node(
            package='whisper_ros',
            executable='whisper_ros',
            name='whisper_ros_node',
            output='screen',
            parameters=[params_file]),
        
        Node(
            package='chat_gpt_ros',
            executable='chat_gpt_ros',
            name='chat_gpt_ros_node',
            output='screen',
            parameters=[params_file]),
        
        Node(
            package='voicevox_ros',
            executable='voicevox_ros',
            name='voicevox_ros_node',
            output='screen',
            parameters=[params_file])
            
        ])
    
    return LaunchDescription([
        declare_params_file_cmd,

        bringup_chatbot_nodes
    ])
