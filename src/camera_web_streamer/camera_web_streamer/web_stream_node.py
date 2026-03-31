#!/usr/bin/env python3
"""ROS2 node that subscribes to camera images and serves MJPEG stream via Flask."""

import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flask import Flask, Response


class WebStreamNode(Node):
    def __init__(self):
        super().__init__('web_stream_node')

        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('quality', 50)
        self.declare_parameter('topic', '/image_raw')

        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        self.quality = self.get_parameter('quality').value
        topic = self.get_parameter('topic').value

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10
        )

        self.get_logger().info(
            f'Web stream node started. Will serve on http://{self.host}:{self.port}'
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ret, jpeg = cv2.imencode(
                '.jpg', cv_image,
                [cv2.IMWRITE_JPEG_QUALITY, self.quality]
            )
            if ret:
                with self.frame_lock:
                    self.latest_frame = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def get_frame(self):
        with self.frame_lock:
            return self.latest_frame


app = Flask(__name__)
node_ref = None


@app.route('/')
def index():
    return '''
    <html>
    <head>
        <title>ROS2 Camera Stream</title>
        <style>
            body {
                display: flex;
                justify-content: center;
                align-items: center;
                min-height: 100vh;
                margin: 0;
                background: #1a1a1a;
                font-family: sans-serif;
                color: #fff;
            }
            .container { text-align: center; }
            h1 { margin-bottom: 20px; }
            img {
                max-width: 100%;
                border: 2px solid #444;
                border-radius: 8px;
            }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>ROS2 Camera Stream</h1>
            <img src="/video_feed" />
        </div>
    </body>
    </html>
    '''


@app.route('/video_feed')
def video_feed():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


def generate_frames():
    while True:
        if node_ref is not None:
            frame = node_ref.get_frame()
            if frame is not None:
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n'
                    + frame
                    + b'\r\n'
                )
            else:
                time.sleep(0.1)
        else:
            time.sleep(0.1)


def main(args=None):
    global node_ref

    rclpy.init(args=args)
    node = WebStreamNode()
    node_ref = node

    flask_thread = threading.Thread(
        target=lambda: app.run(
            host=node.host,
            port=node.port,
            debug=False,
            use_reloader=False,
            threaded=True
        ),
        daemon=True
    )
    flask_thread.start()
    node.get_logger().info(
        f'Flask server started on http://{node.host}:{node.port}'
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
