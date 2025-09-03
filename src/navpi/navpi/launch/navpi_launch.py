import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetParametersFromFile

def generate_launch_description():
    ld = LaunchDescription()

    vdb_config = os.path.join(
            get_package_share_directory('vdb_mapping_ros2'),
            'config',
            'vdb_params.yaml'
            )
    global_planner_config = os.path.join(
            get_package_share_directory('navpi'),
            'config',
            'global_planner_params.yaml'
            )
    local_planner_config = os.path.join(
            get_package_share_directory('navpi'),
            'config',
            'local_planner_params.yaml'
            )
    rmp_local_planner_config = os.path.join(
            get_package_share_directory('navpi'),
            'config',
            'rmp_local_planner_params.yaml'
            )
    coordinator_config = os.path.join(
            get_package_share_directory('navpi'),
            'config',
            'coordinator_params.yaml'
            )
    recovery_config = os.path.join(
            get_package_share_directory('navpi'),
            'config',
            'recovery_params.yaml'
            )

    navpi = Node(
            package='navpi',
            executable='navpi_node',
            name='navpi',
            parameters = [vdb_config, global_planner_config, local_planner_config, rmp_local_planner_config,
                coordinator_config, recovery_config],
            output='screen'
            )

    ld.add_action(navpi)

    return ld
