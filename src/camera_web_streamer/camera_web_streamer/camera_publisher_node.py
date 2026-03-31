#!/usr/bin/env python3
"""ROS2 node that captures from IMX219 via V4L2 raw Bayer and publishes as Image."""

import subprocess

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def find_unicam_device(logger):
    """Find the /dev/mediaX device that contains the unicam (IMX219) driver."""
    import glob
    for media_dev in sorted(glob.glob('/dev/media*')):
        try:
            result = subprocess.run(
                ['media-ctl', '-d', media_dev, '-p'],
                capture_output=True, text=True, timeout=5
            )
            if 'unicam' in result.stdout and 'imx219' in result.stdout:
                logger.info(f'Found unicam device: {media_dev}')
                return media_dev
        except Exception:
            continue
    return None


def run_cmd(cmd, logger, label):
    """Run a command and log both stdout and stderr on failure."""
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        output = (result.stdout + result.stderr).strip()
        if result.returncode != 0:
            logger.warn(f'{label} failed: {output}')
            return False
        logger.info(f'{label}: OK')
        return True
    except Exception as e:
        logger.error(f'{label} error: {e}')
        return False


def setup_sensor(width, height, exposure, analogue_gain, digital_gain, logger):
    """Configure IMX219 sensor format and controls before opening the camera."""
    media_dev = find_unicam_device(logger)
    if media_dev is None:
        logger.error('unicam media device not found. Camera may not work.')
    else:
        run_cmd(
            ['media-ctl', '-d', media_dev, '--set-v4l2',
             f'"imx219 10-0010":0[fmt:SRGGB10_1X10/{width}x{height}]'],
            logger, f'media-ctl ({media_dev})'
        )

    run_cmd(
        ['v4l2-ctl', '-d', '/dev/video0',
         f'--set-fmt-video=width={width},height={height},pixelformat=RG10'],
        logger, 'v4l2-ctl format'
    )

    run_cmd(
        ['v4l2-ctl', '-d', '/dev/v4l-subdev0', '--set-ctrl',
         f'exposure={exposure},analogue_gain={analogue_gain},digital_gain={digital_gain}'],
        logger, 'v4l2-ctl controls'
    )


class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('exposure', 1600)
        self.declare_parameter('analogue_gain', 120)
        self.declare_parameter('digital_gain', 512)

        device_id = self.get_parameter('device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        exposure = self.get_parameter('exposure').value
        analogue_gain = self.get_parameter('analogue_gain').value
        digital_gain = self.get_parameter('digital_gain').value

        # Auto-configure sensor format and controls
        setup_sensor(
            self.width, self.height,
            exposure, analogue_gain, digital_gain,
            self.get_logger()
        )

        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)

        # Open camera with V4L2 backend, raw Bayer mode
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        fourcc = cv2.VideoWriter_fourcc(*'RG10')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera device')
            return

        self.get_logger().info(
            f'Camera opened: device={device_id}, {self.width}x{self.height}, '
            f'{fps}fps, exposure={exposure}, '
            f'analogue_gain={analogue_gain}, digital_gain={digital_gain}'
        )

        period = 1.0 / fps
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        ret, raw = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame')
            return

        # Convert 10-bit Bayer (RG10) to 8-bit BGR
        bayer = np.frombuffer(raw.tobytes(), dtype=np.uint16).reshape(
            self.height, self.width
        )
        bayer8 = (bayer >> 2).astype(np.uint8)
        bgr = cv2.cvtColor(bayer8, cv2.COLOR_BayerRG2BGR)

        # Apply simple white balance (gray world assumption)
        avg_b = np.mean(bgr[:, :, 0])
        avg_g = np.mean(bgr[:, :, 1])
        avg_r = np.mean(bgr[:, :, 2])
        avg_all = (avg_b + avg_g + avg_r) / 3.0
        if avg_b > 0 and avg_g > 0 and avg_r > 0:
            bgr[:, :, 0] = np.clip(bgr[:, :, 0] * (avg_all / avg_b), 0, 255).astype(np.uint8)
            bgr[:, :, 1] = np.clip(bgr[:, :, 1] * (avg_all / avg_g), 0, 255).astype(np.uint8)
            bgr[:, :, 2] = np.clip(bgr[:, :, 2] * (avg_all / avg_r), 0, 255).astype(np.uint8)

        # Build ROS Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = bgr.shape[0]
        msg.width = bgr.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = bgr.shape[1] * 3
        msg.data = bgr.tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
