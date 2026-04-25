from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            emulate_tty=True,
            # teleop_twist_keyboard 는 ROS2 파라미터를 지원하지 않으므로
            # --ros-args 로 초기 속도 값을 환경변수 대신 remapping 형태로 전달.
            # 실제 적용은 노드 내부 변수라 launch 에서 직접 설정 불가 —
            # 대신 실행 후 q/z (전체), w/x (linear), e/c (angular) 키로 조정 가능.
            # 확인된 안정 값: speed 0.11 m/s / turn 2.36 rad/s
            arguments=['--ros-args', '-p', 'speed:=0.11', '-p', 'turn:=2.36'],
        ),
    ])