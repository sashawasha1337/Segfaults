'''
NetworkNode.py
A ROS2 node that manages networking using WebRTC and Flask-SocketIO.
SocketIO is used for signaling, while WebRTC handles peer-to-peer communication.
Streams camera feed, reports metrics that it subscribes to, and transmits data over WebRTC data channels.

Published Topics:
- cmd_vel (geometry_msgs/Twist): Publishes robot movement commands.

Subscribed Topics:
- /camera/image_raw (sensor_msgs/Image): Subscribes to camera feed.
- battery_state (sensor_msgs/BatteryState): Subscribes to battery status.
'''

import asyncio
import threading
from flask import Flask, request
from flask_cors import CORS
from flask_socketio import SocketIO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

from networking.status import StatusPusher
from networking.webrtcpeer import SinglePeerSession

app = Flask(__name__) # 
CORS(app) # Cross Origin Resource Sharing, allows browser access from a different origin
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
webrtc_loop = None

STATUS_INTERVAL = 15.0  # check status every N seconds


class NetworkNode(Node):
    def __init__(self):
        super().__init__('network_node')
        self.get_logger().info("Network node initialization started.")
        
        self.peer_session = None

        self.bridge = CvBridge() # For converting ROS Image messages to OpenCV images

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10) # Commands published for robot movement commands
        self.get_logger().info("cmd_vel publisher created.")
        self.status_pusher = StatusPusher(self, lambda: self.peer_session)
        self.status_timer = self.create_timer(STATUS_INTERVAL, self.handle_status_push)
        # Initial Status
        self.handle_status_push()

        self.battery_voltage = None
        self.battery_subscription = self.create_subscription(
            BatteryState,
            'battery_state',
            self.status_pusher.battery_callback,
            10,
        )
        self.get_logger().info("Battery subscription created.")

        self.latest_frame = None
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info("Camera subscription created.")
        self.get_logger().info("Network node initialized.")

    def camera_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error processing camera frame: {str(e)}')
        
    def execute_command(self, command):
        twist = Twist()
        if command == "forward":
            twist.linear.x = 0.2
        elif command == "back":
            twist.linear.x = -0.2
        elif command == "left":
            twist.angular.z = 0.5
        elif command == "right":
            twist.angular.z = -0.5
        self.cmd_vel_publisher.publish(twist)

    def handle_status_push(self):
        self.get_logger().debug("Pushing status update.")
        if self.peer_session and webrtc_loop:
            async def send_status():
                self.status_pusher.push_status()
            self._submit_webrtc_task(send_status(), "push status")
        self.get_logger().info("Pushed first status update.", once=True)

    def handle_connect(self):
        self.get_logger().info(f'Client connected: {request.sid}')
        try:
            self.peer_session = SinglePeerSession(self, request.sid, socketio)
        except Exception as e:
            self.get_logger().error(f"Failed to create peer session: {str(e)}")
            return
        self._submit_webrtc_task(self.peer_session.start(), "start peer session")

    def handle_disconnect(self):
        self.get_logger().info(f'Client disconnected: {request.sid}')
        if self.peer_session and request.sid == self.peer_session.sid:
            session = self.peer_session
            self._submit_webrtc_task(
                session.close(),
                "close peer session", 
                on_complete=lambda: self._clear_peer_session(session)
                )
            
    def handle_answer(self, answer_data):
        self.get_logger().info('Received WebRTC answer')
        if not self.peer_session or request.sid != self.peer_session.sid:
            self.get_logger().warning(f"No active peer session for sid {request.sid}")
            return
        self._submit_webrtc_task(self.peer_session.process_answer(answer_data), "process answer")

    def _submit_webrtc_task(self, coroutine, context, *, on_complete=None):
        task = asyncio.run_coroutine_threadsafe(coroutine, webrtc_loop)
        def _done(future):
            try:
                future.result()
            except Exception as e:
                self.get_logger().error(f"{context} error: {repr(e)}")
            finally:
                if on_complete:
                    on_complete()

        task.add_done_callback(_done)
        return task

    def _clear_peer_session(self, session):
        if self.peer_session == session:
            self.peer_session = None        
        self.get_logger().info("Peer session cleared.") 

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info("Network node shutting down.")


def main():
    rclpy.init()
    ros_node = NetworkNode()
    socketio.on_event('connect', ros_node.handle_connect)
    socketio.on_event('disconnect', ros_node.handle_disconnect)
    socketio.on_event('answer', ros_node.handle_answer)

    global webrtc_loop
    webrtc_loop = asyncio.new_event_loop()
    ros_node.get_logger().debug("WebRTC event loop started.")

    threading.Thread(target=webrtc_loop.run_forever, daemon=True).start()

    socketio_thread = threading.Thread(
        target=lambda: socketio.run(
            app,
            ssl_context=None,
            host='0.0.0.0',
            port=5000,
            debug=False,
            allow_unsafe_werkzeug=True #Disable runtime error saying Werkzeug web server is not designed for prod
        )
    )
    socketio_thread.daemon = True
    socketio_thread.start()
    try:
        ros_node.get_logger().info("Network node starting.")
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()