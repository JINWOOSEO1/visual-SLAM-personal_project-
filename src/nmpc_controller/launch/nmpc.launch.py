import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share  = get_package_share_directory('nmpc_controller')
    params_file = os.path.join(pkg_share, 'config', 'nmpc_params.yaml')
    default_traj = os.path.join(pkg_share, 'scripts', 'ref_traj.npy')

    traj_file_arg = DeclareLaunchArgument(
        'traj_file',
        default_value=default_traj,
        description='Absolute path to ref_traj.npy (overrides YAML)',
    )

    nmpc_node = Node(
        package='nmpc_controller',
        executable='nmpc_node',
        name='nmpc_node',
        parameters=[
            params_file,
            # CLI argument overrides the YAML value when provided
            {'traj_file': LaunchConfiguration('traj_file')},
        ],
        # Remap to /odom until robot_localization EKF is configured.
        # When EKF is running (Phase 6), remove this remapping.
        remappings=[('/odometry/filtered', '/odom')],
        output='screen',
    )

    return LaunchDescription([
        traj_file_arg,
        nmpc_node,
    ])
