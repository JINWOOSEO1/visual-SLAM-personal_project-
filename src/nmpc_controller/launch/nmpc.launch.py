import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_real_traj_path(pkg_share):
    candidates = [
        os.path.abspath(os.path.join(
            pkg_share, '..', '..', '..', '..', 'src', 'nmpc_controller',
        )),
        os.path.abspath(os.path.join(
            os.path.dirname(os.path.realpath(__file__)), '..',
        )),
    ]

    for pkg_dir in candidates:
        scripts_dir = os.path.join(pkg_dir, 'scripts')
        if os.path.isdir(scripts_dir):
            return os.path.join(scripts_dir, 'real_traj.npy')

    return os.path.join(pkg_share, 'scripts', 'real_traj.npy')


def generate_launch_description():
    pkg_share  = get_package_share_directory('nmpc_controller')
    params_file = os.path.join(pkg_share, 'config', 'nmpc_params.yaml')
    default_traj = os.path.join(pkg_share, 'scripts', 'ref_traj.npy')
    default_real_traj = default_real_traj_path(pkg_share)

    traj_file_arg = DeclareLaunchArgument(
        'traj_file',
        default_value=default_traj,
        description='Absolute path to ref_traj.npy (overrides YAML)',
    )
    real_traj_file_arg = DeclareLaunchArgument(
        'real_traj_file',
        default_value=default_real_traj,
        description='Absolute path to save odometry trajectory as .npy',
    )

    nmpc_node = Node(
        package='nmpc_controller',
        executable='nmpc_node',
        name='nmpc_node',
        parameters=[
            params_file,
            # CLI argument overrides the YAML value when provided
            {'traj_file': LaunchConfiguration('traj_file')},
            {'real_traj_file': LaunchConfiguration('real_traj_file')},
        ],
        output='screen',
    )

    return LaunchDescription([
        traj_file_arg,
        real_traj_file_arg,
        nmpc_node,
    ])
