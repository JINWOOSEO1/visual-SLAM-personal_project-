from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # MPU6050 드라이버 노드
        Node(
            package='mpu6050_driver',
            executable='mpu6050_node',
            name='mpu6050_node',
            parameters=[{
                'i2c_bus': 1,
                'device_address': 0x68,
                'frequency': 50.0,
                'frame_id': 'imu_link',
                'accel_range': 0,   # ±2g
                'gyro_range': 0,    # ±250 °/s
                'calibration_samples': 1000,
            }],
            output='screen',
        ),
        # Madgwick 필터: /imu/data_raw → /imu/data (orientation 포함)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'frequency': 50.0,
            }],
            remappings=[
                ('imu/data_raw', 'imu/data_raw'),
                ('imu/data', 'imu/data'),
            ],
            output='screen',
        ),
    ])
