"""
Phase 4: 카메라-IMU 외부 캘리브레이션 (Static TF)

TF 프레임 구조:
    base_link
    ├── camera_link   (전방 카메라)
    └── imu_link      (MPU6050)

좌표계 규칙 (REP-103):
    x: 전방, y: 좌측, z: 상방

실측값 (단위: 미터, base_link 원점 = 뒷바퀴 축 중심):
    camera_link: 전방 0.175m, 상방 0.050m
    imu_link:    전방 0.060m, 우측 0.065m(→ y=-0.065), 상방 0.023m

IMU 물리 장착 방향:
    IMU x축 → 로봇 우측(base_link -y)
    IMU y축 → 로봇 전방(base_link +x)
    IMU z축 → 로봇 상방(base_link +z)
    ∴ yaw = -π/2 회전 적용
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # base_link → camera_link
    # 카메라가 전방(x)을 바라봄, 광학 프레임과 일치하도록 설정
    base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', '0.175',   # 전방 17.5cm
            '--y', '0.0',
            '--z', '0.050',   # 상방 5.0cm
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    # camera_link → camera (optical frame)
    # ROS 카메라 광학 프레임 규칙: z 전방, x 우측, y 하방
    # camera_link(x전방,y좌,z상) → optical(z전방,x우,y하)
    # 회전: roll=-π/2, yaw=-π/2
    # child-frame-id 는 camera_ros 노드가 이미지 헤더에 박는 frame_id ('camera') 와 일치시켜야
    # rtabmap 이 TF lookup 가능. (기존 'camera_optical_frame' → 'camera' 로 변경)
    camera_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_optical_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '-1.5707963',   # -π/2
            '--pitch', '0.0',
            '--yaw', '-1.5707963',    # -π/2
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera',
        ],
    )

    # base_link → imu_link
    # IMU x→로봇우측(-y), y→로봇전방(+x), z→로봇상방(+z)  ∴ yaw=-π/2
    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=[
            '--x', '0.060',   # 전방 6.0cm
            '--y', '-0.065',  # 우측 6.5cm
            '--z', '0.023',   # 상방 2.3cm
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '-1.5707963',   # -π/2 : IMU y축이 로봇 전방
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
    )

    return LaunchDescription([
        base_to_camera,
        camera_to_optical,
        base_to_imu,
    ])
