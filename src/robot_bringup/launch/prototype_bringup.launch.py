import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    default_topic_map = os.path.join(pkg_share, 'config', 'topic_map.yaml')
    default_locations = os.path.join(pkg_share, 'config', 'hospital_locations.yaml')

    topic_map_arg = DeclareLaunchArgument(
        'topic_map_path',
        default_value=default_topic_map,
        description='Path to canonical topic_map.yaml'
    )

    locations_arg = DeclareLaunchArgument(
        'hospital_locations_path',
        default_value=default_locations,
        description='Path to hospital_locations.yaml'
    )

    topic_map_path = LaunchConfiguration('topic_map_path')
    locations_path = LaunchConfiguration('hospital_locations_path')

    # 1. Tablet Bridge Node (REST API on port 8000 -> ROS 2 interface)
    tablet_bridge_node = Node(
        package='robot_interface',
        executable='tablet_bridge_node',
        name='tablet_bridge_node',
        output='screen',
        parameters=[{
            'topic_map_path': topic_map_path,
            'api_host': '0.0.0.0',
            'api_port': 8000,
        }]
    )

    # 4. Task Manager Node (Translates semantic destination -> Nav2 goal)
    task_manager_node = Node(
        package='robot_navigation',
        executable='task_manager_node',
        name='task_manager_node',
        output='screen',
        parameters=[{
            'topic_map_path': topic_map_path,
            'hospital_locations_path': locations_path,
        }]
    )

    # 5. Safety Supervisor Node (Monitors E-Stop & MCU telemetry)
    safety_supervisor_node = Node(
        package='robot_control',
        executable='safety_supervisor_node',
        name='safety_supervisor_node',
        output='screen',
        parameters=[{
            'topic_map_path': topic_map_path,
        }]
    )

    return LaunchDescription([
        topic_map_arg,
        locations_arg,
        LogInfo(msg="Launching Hospital Robot Phase 1 Prototype Stack..."),
        tablet_bridge_node,
        task_manager_node,
        safety_supervisor_node,
    ])

