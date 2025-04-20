#!/usr/bin/env python3

import asyncio
import threading
import json
from flask import Flask, request
from flask_cors import CORS
from flask_socketio import SocketIO
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCRtpSender
from aiortc.contrib.media import MediaStreamTrack, VideoFrame
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy

# Flask and Socket.IO setup
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Network node that bridges WebRTC and ROS
class NetworkNode(Node):
	def __init__(self):
		super().__init__('network_node')
		# Publisher for robot movement
		self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		# Subscribe to camera feed
		self.bridge = CvBridge()
		self.latest_frame = None
		self.camera_subscription = self.create_subscription(
			Image,
			'/camera/image_raw',
			self.camera_callback,
			10)
		# Instance-level peer connections
		self.peer_connections = {}
		self.get_logger().info('Network node initialized')
	
	def camera_callback(self, msg):
		try:
			# Store the latest frame
			self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except Exception as e:
			self.get_logger().error(f'Error processing camera frame: {str(e)}')
			
	def execute_command(self, command):
		# Create Twist message based on command
		twist = Twist()
		if command == "forward":
			twist.linear.x = 0.2
		elif command == "back":
			twist.linear.x = -0.2
		elif command == "left":
			twist.angular.z = 0.5
		elif command == "right":
			twist.angular.z = -0.5
		# Publish to the cmd_vel topic
		self.cmd_vel_publisher.publish(twist)
		self.get_logger().info(f"Executed command: {command}")
	
	# Socket.IO event handlers within the node
	def handle_connect(self):
		print(f'Client connected: {request.sid}')
	
	def handle_disconnect(self):
		print(f'Client disconnected: {request.sid}')
		if request.sid in self.peer_connections:
			pc = self.peer_connections.pop(request.sid)
			asyncio.run_coroutine_threadsafe(pc.close(), asyncio.get_event_loop())
	
	def handle_offer(self, offer_data):
		print('Received WebRTC offer')
		asyncio.run_coroutine_threadsafe(self.process_offer(request.sid, offer_data), asyncio.get_event_loop())
	
	async def process_offer(self, sid, offer_data):
		offer = json.loads(offer_data)
		# Create peer connection
		pc = RTCPeerConnection()
		self.peer_connections[sid] = pc
		# Set codec preference to H.264 if supported
		capabilities = RTCRtpSender.getCapabilities("video")
		h264_codecs = [c for c in capabilities if c.mimeType == "video/H264"]
		if h264_codecs:
			transceiver = pc.addTransceiver("video")
			transceiver.setCodecPreferences(h264_codecs)
			transceiver.sender.replaceTrack(ROSVideoTrack(self))
		else:
			pc.addTrack(ROSVideoTrack(self))
		# Handle data channel for commands
		@pc.on("datachannel")
		def on_datachannel(channel):
			@channel.on("message")
			def on_message(message):
				if isinstance(message, str):
					print(f"Received command: {message}")
					self.execute_command(message)
		await pc.setRemoteDescription(RTCSessionDescription(sdp=offer["sdp"], type=offer["type"]))
		answer = await pc.createAnswer()
		await pc.setLocalDescription(answer)
		socketio.emit('answer', json.dumps({
			"sdp": pc.localDescription.sdp,
			"type": pc.localDescription.type
		}), room=sid)

# Video track that uses the latest frame from ROS
class ROSVideoTrack(VideoStreamTrack):
	def __init__(self, node):
		super().__init__()
		self.node = node
		
	async def recv(self):
		pts, time_base = await self.next_timestamp()
		# Get frame from ROS node
		if self.node.latest_frame is not None:
			frame = self.node.latest_frame
		else:
			# Create a black frame if no image is available
			import numpy as np
			frame = np.zeros((480, 640, 3), np.uint8)
		# Convert to VideoFrame
		video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
		video_frame.pts = pts
		video_frame.time_base = time_base
		
		return video_frame

# Main function
def main():
	# Initialize ROS
	rclpy.init()
	ros_node = NetworkNode()
	# Register Socket.IO event handlers using the node’s methods
	socketio.on_event('connect', ros_node.handle_connect)
	socketio.on_event('disconnect', ros_node.handle_disconnect)
	socketio.on_event('offer', ros_node.handle_offer)
	# Set up asyncio event loop
	loop = asyncio.new_event_loop()
	asyncio.set_event_loop(loop)
	threading.Thread(
		target=lambda: loop.run_forever(),
		daemon=True
	).start()
	# Start Flask/Socket.IO in a thread
	socketio_thread = threading.Thread(
		target=lambda: socketio.run(app, host='0.0.0.0', port=5000, debug=False)
	)
	socketio_thread.daemon = True
	socketio_thread.start()
	# Spin ROS node in the main thread
	try:
		rclpy.spin(ros_node)
	except KeyboardInterrupt:
		pass
	finally:
		ros_node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()