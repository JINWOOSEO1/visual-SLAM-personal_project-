import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature

import smbus


# MPU6050 레지스터 주소
_MPU6050_ADDR = 0x68
_PWR_MGMT_1 = 0x6B
_ACCEL_CONFIG = 0x1C
_GYRO_CONFIG = 0x1B
_SMPLRT_DIV = 0x19
_CONFIG = 0x1A
_ACCEL_XOUT_H = 0x3B
_TEMP_OUT_H = 0x41
_GYRO_XOUT_H = 0x43
_WHO_AM_I = 0x75

# 스케일 팩터
_ACCEL_SCALE = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}  # ±2g, ±4g, ±8g, ±16g
_GYRO_SCALE = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}  # ±250, ±500, ±1000, ±2000 °/s

_GRAVITY = 9.80665
_DEG_TO_RAD = 0.017453292519943


class Mpu6050Node(Node):

    def __init__(self):
        super().__init__('mpu6050_node')

        # 파라미터 선언
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('device_address', _MPU6050_ADDR)
        self.declare_parameter('frequency', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('accel_range', 0)  # 0=±2g, 1=±4g, 2=±8g, 3=±16g
        self.declare_parameter('gyro_range', 0)   # 0=±250, 1=±500, 2=±1000, 3=±2000 °/s
        self.declare_parameter('calibration_samples', 200)

        self._bus_num = self.get_parameter('i2c_bus').value
        self._addr = self.get_parameter('device_address').value
        self._frame_id = self.get_parameter('frame_id').value
        self._accel_range = self.get_parameter('accel_range').value
        self._gyro_range = self.get_parameter('gyro_range').value
        freq = self.get_parameter('frequency').value
        cal_samples = self.get_parameter('calibration_samples').value

        # I2C 초기화
        self._bus = smbus.SMBus(self._bus_num)

        # WHO_AM_I 확인
        who = self._bus.read_byte_data(self._addr, _WHO_AM_I)
        if who != 0x68:
            self.get_logger().error(f'WHO_AM_I mismatch: expected 0x68, got 0x{who:02x}')
            raise RuntimeError('MPU6050 not found')
        self.get_logger().info(f'MPU6050 detected at 0x{self._addr:02x}')

        # 센서 초기화
        self._bus.write_byte_data(self._addr, _PWR_MGMT_1, 0x00)  # 슬립 해제
        time.sleep(0.1)
        self._bus.write_byte_data(self._addr, _SMPLRT_DIV, 0x00)  # 샘플레이트 = gyro rate / (1+div)
        self._bus.write_byte_data(self._addr, _CONFIG, 0x03)  # DLPF 44Hz
        self._bus.write_byte_data(self._addr, _ACCEL_CONFIG, self._accel_range << 3)
        self._bus.write_byte_data(self._addr, _GYRO_CONFIG, self._gyro_range << 3)

        self._accel_scale = _ACCEL_SCALE[self._accel_range]
        self._gyro_scale = _GYRO_SCALE[self._gyro_range]

        # 퍼블리셔
        self._imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self._temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)

        # 바이어스 캘리브레이션 (정지 상태에서)
        self.get_logger().info(f'Calibrating bias ({cal_samples} samples)... keep sensor still!')
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._accel_bias = [0.0, 0.0, 0.0]
        self._calibrate(cal_samples)
        self.get_logger().info(
            f'Calibration done: '
            f'gyro_bias=[{self._gyro_bias[0]:.4f}, {self._gyro_bias[1]:.4f}, {self._gyro_bias[2]:.4f}] rad/s, '
            f'accel_bias=[{self._accel_bias[0]:.4f}, {self._accel_bias[1]:.4f}, {self._accel_bias[2]:.4f}] m/s²'
        )

        # 타이머
        self._timer = self.create_timer(1.0 / freq, self._timer_callback)
        self.get_logger().info(f'Publishing IMU data at {freq} Hz on /imu/data_raw')

    def _read_raw_data(self):
        """가속도(6) + 온도(2) + 자이로(6) = 14바이트 한번에 읽기."""
        data = self._bus.read_i2c_block_data(self._addr, _ACCEL_XOUT_H, 14)
        vals = struct.unpack('>hhhhhhh', bytes(data))
        # vals: ax, ay, az, temp, gx, gy, gz
        return vals

    def _calibrate(self, samples):
        gx_sum = gy_sum = gz_sum = 0.0
        ax_sum = ay_sum = az_sum = 0.0
        for _ in range(samples):
            ax, ay, az, _, gx, gy, gz = self._read_raw_data()
            ax_sum += ax / self._accel_scale * _GRAVITY
            ay_sum += ay / self._accel_scale * _GRAVITY
            az_sum += az / self._accel_scale * _GRAVITY
            gx_sum += gx / self._gyro_scale * _DEG_TO_RAD
            gy_sum += gy / self._gyro_scale * _DEG_TO_RAD
            gz_sum += gz / self._gyro_scale * _DEG_TO_RAD
            time.sleep(0.005)

        self._gyro_bias[0] = gx_sum / samples
        self._gyro_bias[1] = gy_sum / samples
        self._gyro_bias[2] = gz_sum / samples
        # 가속도는 중력 성분을 유지해야 하므로 바이어스만 보정하지 않음
        # 대신 나중에 필요시 사용할 수 있도록 저장
        self._accel_bias[0] = ax_sum / samples
        self._accel_bias[1] = ay_sum / samples
        self._accel_bias[2] = az_sum / samples

    def _timer_callback(self):
        try:
            ax_raw, ay_raw, az_raw, temp_raw, gx_raw, gy_raw, gz_raw = self._read_raw_data()
        except OSError as e:
            self.get_logger().warn(f'I2C read error: {e}')
            return

        now = self.get_clock().now().to_msg()

        # 단위 변환: 가속도 → m/s², 자이로 → rad/s
        ax = ax_raw / self._accel_scale * _GRAVITY
        ay = ay_raw / self._accel_scale * _GRAVITY
        az = az_raw / self._accel_scale * _GRAVITY
        gx = (gx_raw / self._gyro_scale * _DEG_TO_RAD) - self._gyro_bias[0]
        gy = (gy_raw / self._gyro_scale * _DEG_TO_RAD) - self._gyro_bias[1]
        gz = (gz_raw / self._gyro_scale * _DEG_TO_RAD) - self._gyro_bias[2]

        # IMU 메시지
        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        # orientation은 raw 드라이버에서 제공하지 않음
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = 0.0004
        msg.angular_velocity_covariance[4] = 0.0004
        msg.angular_velocity_covariance[8] = 0.0004
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance[0] = 0.04
        msg.linear_acceleration_covariance[4] = 0.04
        msg.linear_acceleration_covariance[8] = 0.04

        self._imu_pub.publish(msg)

        # 온도 메시지
        temp_msg = Temperature()
        temp_msg.header.stamp = now
        temp_msg.header.frame_id = self._frame_id
        temp_msg.temperature = temp_raw / 340.0 + 36.53
        self._temp_pub.publish(temp_msg)

    def destroy_node(self):
        self._bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
