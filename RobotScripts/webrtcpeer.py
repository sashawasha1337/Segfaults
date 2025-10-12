'''
Helper class that manages WebRTC connection lifecycle, including offer/answer exchange,
data channel communication, and video track streaming.
'''

import asyncio
import json
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame

class SinglePeerSession:
    def __init__(self, node, sid, socketio):
        self.node = node
        self.sid = sid
        self.socketio = socketio
        self.peer_connection = None
        self.data_channel = None
        self.logger = node.get_logger()


    async def start(self):
        self.logger.info(f"Initiating WebRTC connection for {self.sid}")

        if self.peer_connection:
            self.logger.info("Closing existing peer connection")
            try:
                await self.peer_connection.close()
            except Exception as e:
                self.logger.error(f"Error closing peer connection: {repr(e)}")
            finally:
                self.peer_connection = None
                self.data_channel = None

        pc = RTCPeerConnection()
        self.peer_connection = pc

        @pc.on("icecandidate")
        def on_ice_candidate(event):
            self.logger.info(f"ICE candidate for {self.sid}: {event.candidate}")

        @pc.on("iceconnectionstatechange")
        def on_ice_connection_state_change():
            self.logger.info(f"ICE connection state changed for {self.sid}: {pc.iceConnectionState}")

        @pc.on("connectionstatechange")
        def on_connection_state_change():
            self.logger.info(f"Connection state changed for {self.sid}: {pc.connectionState}")

        @pc.on("icegatheringstatechange")
        def on_ice_gathering_state_change():
            self.logger.info(f"ICE gathering state changed for {self.sid}: {pc.iceGatheringState}")

        self.data_channel = pc.createDataChannel("robot")
        self.logger.info(f"Data channel created for {self.sid}")

        @self.data_channel.on("open")
        def on_open():
            self.logger.info(f"Data channel opened for {self.sid}")

        @self.data_channel.on("close")
        def on_close():
            self.logger.info(f"Data channel closed unexpectedly for {self.sid}")

        @self.data_channel.on("error")
        def on_error(error):
            self.logger.error(f"Data channel error for {self.sid}: {error}")

        @self.data_channel.on("message")
        def on_message(message):
            self.logger.info(f"Data channel message from {self.sid}: {message}")
            try:
                if isinstance(message, bytes):
                    message = message.decode("utf-8")
                obj = json.loads(message)
                if obj.get("type") == "command":
                    cmd = obj.get("command")
                    self.logger.info(f"Received command: {cmd}")
                    self.node.execute_command(cmd)
                    # Send acknowledgment back
                    try:
                        ack = json.dumps({"type": "command_ack", "command": cmd})
                        if self.data_channel.readyState == "open":
                            self.data_channel.send(ack)
                        else: 
                            self.logger.warning(f"Cannot send ack - data channel state: {self.data_channel.readyState}")
                    except Exception as e:
                        self.logger.error(f"Error sending command acknowledgment: {e}")
                else:
                    self.logger.info(f"Unknown inbound message: {obj}")
            except Exception as e:
                self.logger.error(f"Error handling inbound message: {e}")
        
        
        pc.addTrack(ROSVideoTrack(self.node))

        offer = await self.peer_connection.createOffer()
        await self.peer_connection.setLocalDescription(offer)

        self.logger.info(f"Offer type: {offer.type}, SDP length: {len(offer.sdp)}")

        payload = json.dumps({
            "sdp": self.peer_connection.localDescription.sdp,
            "type": self.peer_connection.localDescription.type,
        })
        self.socketio.emit('offer', payload, room=self.sid)
        self.logger.info(f"Sent WebRTC offer to {self.sid}")

    async def process_answer(self, answer_data): 
        if not self.peer_connection:
            self.logger.warning(f"No active peer for sid {self.sid}")
            return
        
        try:
            answer = json.loads(answer_data)
            await self.peer_connection.setRemoteDescription(
                RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
            )
            self.get_logger().info(f"WebRTC connection established successfully for {self.sid}")
        except json.JSONDecodeError as e:
            self.logger.error(f"Invalid JSON in answer data: {e}")
        except Exception as e:
            self.logger.error(f"Error setting remote description: {e}")
            await self.peer_connection.close()
            self.peer_connection = None
            self.data_channel = None

    async def close(self): 
        if self.peer_connection:
            await self.peer_connection.close()
            self.peer_connection = None
            self.data_channel = None
            self.logger.info("Peer connection closed")

# Helper class to manage video streaming from ROS to WebRTC
class ROSVideoTrack(VideoStreamTrack):
    def __init__(self, node):
        super().__init__()
        self.node = node

    async def recv(self):
        try:
            await asyncio.sleep(1 / 15)
            pts, time_base = await self.next_timestamp()
            frame = self.node.latest_frame
            if frame is None:
                import numpy as np
                frame = np.zeros((480, 640, 3), np.uint8)
                self.node.get_logger().warning("No camera frame available, sending black frame", throttle_duration_sec=5)
            video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
            video_frame.pts = pts
            video_frame.time_base = time_base
            return video_frame
        except Exception as e:
            self.node.get_logger().error(f"Error in video frame retrieval: {repr(e)}")
            raise