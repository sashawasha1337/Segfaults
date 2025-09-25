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
        self.peer_connection.addTrack(ROSVideoTrack(self.node))
    
        data_channel = self.peer_connection.createDataChannel("commands")
        self.data_channel = data_channel
        
        @data_channel.on("open")
        def on_open():
            print("Data channel opened")
            
        @data_channel.on("message")
        def on_message(message):
            if isinstance(message, str):
                print(f"Received command: {message}")
                try:
                    self.node.execute_command(message)
                except Exception as e:
                    self.logger.error(f"Error executing command: {e}")

        offer = await self.peer_connection.createOffer()
        await self.peer_connection.setLocalDescription(offer)

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
        
        answer = json.loads(answer_data)
        
        try:
            await self.peer_connection.setRemoteDescription(
                RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
            )
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
        await asyncio.sleep(1 / 15)
        pts, time_base = await self.next_timestamp()
        frame = self.node.latest_frame
        if frame is None:
            import numpy as np
            frame = np.zeros((480, 640, 3), np.uint8)
        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame
