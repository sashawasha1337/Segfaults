"""
Helper class that manages WebRTC connection lifecycle, including offer/answer exchange,
data channel communication, and video track streaming.
"""

import asyncio
import json

from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
    RTCConfiguration,
    RTCIceServer,
)

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

        # Clean up any existing PC
        if self.peer_connection:
            self.logger.info("Closing existing peer connection")
            try:
                await self.peer_connection.close()
            except Exception as e:
                self.logger.error(f"Error closing peer connection: {repr(e)}")
            finally:
                self.peer_connection = None
                self.data_channel = None

        # Configure STUN to improve connectivity
        config = RTCConfiguration(
            iceServers=[RTCIceServer(urls=["stun:stun.l.google.com:19302"])]
        )
        pc = RTCPeerConnection(configuration=config)
        self.peer_connection = pc

        # --- Data channel created on server side
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
                            self.logger.warning(
                                f"Cannot send ack - data channel state: {self.data_channel.readyState}"
                            )
                    except Exception as e:
                        self.logger.error(
                            f"Error sending command acknowledgment: {e}"
                        )
                else:
                    self.logger.info(f"Unknown inbound message: {obj}")
            except Exception as e:
                self.logger.error(f"Error handling inbound message: {e}")


        video_track = ROSVideoTrack(self.node)
        pc.addTrack(video_track)

        # --- Create & set local description (offer)
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        # Wait for ICE gathering to complete (non-trickle flow)
        try:
            while pc.iceGatheringState != "complete":
                await asyncio.sleep(0.05)
        except Exception as e:
            self.logger.error(f"Error while waiting ICE gathering: {repr(e)}")

        payload = json.dumps(
            {
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type,
            }
        )
        self.socketio.emit("offer", payload, room=self.sid)
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
            self.logger.info(
                f"WebRTC connection established successfully for {self.sid}"
            )

        except json.JSONDecodeError as e:
            self.logger.error(f"Invalid JSON in answer data: {e}")
        except Exception as e:
            self.logger.error(f"Error setting remote description: {e}")
            try:
                await self.peer_connection.close()
            finally:
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
        self.logger = node.get_logger()

    async def recv(self):
        await asyncio.sleep(1/25)
        # wait until a real frame exists
        while self.node.latest_frame is None:
            await asyncio.sleep(0.05)

        pts, time_base = await self.next_timestamp()
        frame = self.node.latest_frame.copy()
        vf = VideoFrame.from_ndarray(frame, format="bgr24")
        vf.pts, vf.time_base = pts, time_base

        return vf

