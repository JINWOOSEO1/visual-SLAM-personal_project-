"""
RTAB-Map Monocular Visual SLAM launch (PC 측 실행)

입력 토픽 (Pi 에서 Wi-Fi 로 수신):
    /camera/image_raw       (sensor_msgs/Image)
    /camera/camera_info     (sensor_msgs/CameraInfo)
    /odometry/filtered      (nav_msgs/Odometry, EKF 출력)
    /imu/data               (sensor_msgs/Imu, madgwick)

출력:
    /rtabmap/mapData, /rtabmap/cloud_map, TF map → odom

사용법 (PC):
    ros2 launch rc_car_bringup rtabmap.launch.py
    # GUI 없이 실행하려면:
    ros2 launch rc_car_bringup rtabmap.launch.py rtabmap_viz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    database_path = LaunchConfiguration('database_path')

    # ── RTAB-Map 파라미터 ───────────────────────────────────────────
    # 단안 카메라 + 휠 오도메트리 + IMU 융합 모드
    rtabmap_params = {
        'use_sim_time': use_sim_time,

        # 프레임 / 토픽 구독 모드
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'subscribe_depth': False,
        'subscribe_rgb': True,
        'subscribe_rgbd': False,
        'subscribe_stereo': False,
        'subscribe_scan': False,
        'subscribe_odom_info': False,

        # 카메라/odom 타임스탬프 동기화 (서로 다른 주기 → approx)
        'approx_sync': True,
        'approx_sync_max_interval': 0.05,
        'queue_size': 30,
        'qos_image': 1,            # BEST_EFFORT (Wi-Fi 손실 허용)
        'qos_camera_info': 1,
        'qos_imu': 1,
        'qos_odom': 1,

        # DB 파일
        'database_path': database_path,
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',

        # ── 평면 주행 (2D SLAM) ─────────────────────────────────
        'Reg/Force3DoF': 'true',
        'Optimizer/Slam2D': 'true',
        'Optimizer/Strategy': '1',          # 1 = g2o
        'RGBD/OptimizeFromGraphEnd': 'false',

        # ── 단안 모드 (스케일은 wheel odom 으로 해결) ────────────
        'Vis/EstimationType': '1',          # 1 = PnP (단안에 적합)
        'Vis/MinInliers': '15',
        'Vis/InlierDistance': '0.1',
        'Vis/MaxFeatures': '600',

        # ── 검출/루프클로저 주기 (RPi 부담 ↓, PC 측이지만 보수적) ──
        'Rtabmap/DetectionRate': '1.0',     # Hz
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.05',       # rad
        'RGBD/LinearUpdate': '0.05',        # m

        # ── 특징점 ────────────────────────────────────────────
        'Kp/MaxFeatures': '400',
        'Kp/DetectorStrategy': '6',         # 6 = GFTT/BRIEF (가벼움)
    }

    # 토픽 remapping
    rtabmap_remappings = [
        ('rgb/image',       '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('odom',            '/odometry/filtered'),
        ('imu',             '/imu/data'),
    ]

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
        remappings=rtabmap_remappings,
        arguments=['--delete_db_on_start'],   # 매 실행마다 새 맵 (필요시 제거)
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_depth': False,
            'subscribe_rgb': True,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'queue_size': 30,
        }],
        remappings=rtabmap_remappings,
        condition=IfCondition(rtabmap_viz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap.db',
            description='RTAB-Map database file path',
        ),
        rtabmap_node,
        rtabmap_viz_node,
    ])
