import asyncio
import threading
import json
from flask import Flask, request
from flask_cors import CORS
from flask_socketio import SocketIO
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCDataChannel
from av import VideoFrame
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import qos_profile_sensor_data

# Flask and Socket.IO setup
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
webrtc_loop = None

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
            qos_profile_sensor_data
            )
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
        # Robot initiates WebRTC connection when client connects
        fut = asyncio.run_coroutine_threadsafe(self.initiate_webrtc(request.sid), webrtc_loop)
        def _done(cb):
            try:
                cb.result()
            except Exception as e:
                print("initiate_webrtc error:", repr(e))
        fut.add_done_callback(_done)
    
    def handle_disconnect(self):
        print(f'Client disconnected: {request.sid}')
        if request.sid in self.peer_connections:
            pc = self.peer_connections.pop(request.sid)
            asyncio.run_coroutine_threadsafe(pc.close(), webrtc_loop)

    def handle_answer(self, answer_data):
        print('Received WebRTC answer')
        fut = asyncio.run_coroutine_threadsafe(self.process_answer(request.sid, answer_data), webrtc_loop)
        def _done(cb):
            try:
                cb.result()
            except Exception as e:
                print("process_answer error:", repr(e))
        fut.add_done_callback(_done)

    async def initiate_webrtc(self, sid):
        """Robot initiates the WebRTC connection"""
        print(f"Initiating WebRTC connection for {sid}")
        
        # Create peer connection as initiator
        pc = RTCPeerConnection()
        self.peer_connections[sid] = pc
        
        @pc.on("icegatheringstatechange")
        def _on_ice_gathering():
            print("iceGatheringState:", pc.iceGatheringState)
            
        @pc.on("iceconnectionstatechange")
        def _on_ice_connection():
            print("iceConnectionState:", pc.iceConnectionState)
            
        @pc.on("connectionstatechange")
        def _on_connection():
            print("connectionState:", pc.connectionState)
        
        # Add video track
        pc.addTrack(ROSVideoTrack(self))
        
        # Create data channel for commands
        data_channel = pc.createDataChannel("commands")
        
        @data_channel.on("open")
        def on_open():
            print("Data channel opened")
            
        @data_channel.on("message")
        def on_message(message):
            if isinstance(message, str):
                print(f"Received command: {message}")
                self.execute_command(message)

        # Create offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        print("=== Local offer SDP ===")
        for line in pc.localDescription.sdp.splitlines():
            if line.startswith("m="):
                print(line)
        print("=============")



        
        # Wait for ICE gathering to complete
        print("Waiting for ICE gathering to complete...")
        for _ in range(100):  # ~5 seconds at 50 ms
            await asyncio.sleep(0.05)
            if pc.iceGatheringState == "complete":
                break
                
        print("ICE gather state at send:", pc.iceGatheringState)
        
        # Send offer to client
        payload = json.dumps({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        })
        socketio.emit('offer', payload, room=sid)
        print("Sent WebRTC offer to", sid)

    async def process_answer(self, sid, answer_data):
        """Process the answer from the client"""
        if sid not in self.peer_connections:
            print(f"No peer connection found for {sid}")
            return
            
        pc = self.peer_connections[sid]
        answer = json.loads(answer_data)
        
        try:
            await pc.setRemoteDescription(RTCSessionDescription(sdp=answer["sdp"], type=answer["type"]))
            print("WebRTC connection established!")
        except Exception as e:
            print(f"Error setting remote description: {e}")
            if sid in self.peer_connections:
                del self.peer_connections[sid]

# Video track that uses the latest frame from ROS
class ROSVideoTrack(VideoStreamTrack):
    def __init__(self, node):
        super().__init__()
        self.node = node
        
    async def recv(self):
        await asyncio.sleep(1/15)  # ~15 fps
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
    # Register Socket.IO event handlers using the node's methods
    socketio.on_event('connect', ros_node.handle_connect)
    socketio.on_event('disconnect', ros_node.handle_disconnect)
    socketio.on_event('answer', ros_node.handle_answer)  # Changed from 'offer' to 'answer'

    # Set up dedicated asyncio event loop (GLOBAL) for WebRTC
    global webrtc_loop
    webrtc_loop = asyncio.new_event_loop()

    def _run_loop():
        asyncio.set_event_loop(webrtc_loop)
        webrtc_loop.run_forever()
    threading.Thread(target=_run_loop, daemon=True).start()

    # Start Flask/Socket.IO in a thread
    socketio_thread = threading.Thread(
        target=lambda: socketio.run(app, ssl_context=None, host='0.0.0.0', port=5000, debug=False)
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
