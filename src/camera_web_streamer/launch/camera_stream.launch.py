from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # libcamera 기반 카메라 노드 (하드웨어 ISP 사용)
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            parameters=[{
                'width': 640,
                'height': 480,
                'format': 'RGB888',
            }],
            output='screen',
        ),
        # 웹 스트림 노드 (Flask MJPEG 서버)
        Node(
            package='camera_web_streamer',
            executable='web_stream_node',
            name='web_stream_node',
            parameters=[{
                'port': 8080,
                'host': '0.0.0.0',
                'quality': 50,
                'topic': '/camera/image_raw',
            }],
            output='screen',
        ),
    ])
