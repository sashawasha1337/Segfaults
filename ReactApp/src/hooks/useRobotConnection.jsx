// hooks/useRobotConnection.js
import { useState, useEffect, useRef } from 'react';
import Peer from 'simple-peer/simplepeer.min.js';
import { io } from 'socket.io-client';

export function useRobotConnection(robotIP, videoRef, onBatteryUpdate, onWifiUpdate) {
  const [connectionStatus, setConnectionStatus] = useState('disconnected');
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState(null);
  const peerRef = useRef(null);
  const socketRef = useRef(null);

  // Initialize connection
  useEffect(() => {
    if (!robotIP) {
      setConnectionStatus('No robot IP available');
      return;
    }

    setConnectionStatus('connecting');
    
    // Connect to signaling server
    console.log(`Connecting to: http://${robotIP}`);
    const socket = io(`http://${robotIP}`, {
      reconnectionAttempts: 3,
      timeout: 5000,
      transports: ['websocket', 'polling'],
    });
    socketRef.current = socket;

    socket.on('connect', () => {
      setConnectionStatus('signaling');
      console.log('Connected to robot signaling server');
    });
    
    // Handle incoming offer from robot
    socket.on('offer', (offerData) => {
      console.log('Received WebRTC offer from robot');
      setConnectionStatus('processing offer');
      
      try {
        // Create peer connection as answerer (not initiator)
        const peer = new Peer({ 
          initiator: false,  // Client is now the answerer
          trickle: false
        });
        peerRef.current = peer;

        // Handle all data from the robot through simple-peer's data event
        peer.on('data', (data) => {
          console.log("Message from robot:", data.toString());
          try {
            const msg = JSON.parse(data.toString());
            if (msg.type === "status") {
              onBatteryUpdate?.(msg.batteryVoltage);
              onWifiUpdate?.(msg.wifiStrength);
            } else if (msg.type === "command") {
              console.log("Command acknowledged:", msg);
            }
          } catch (err) {
            console.error("Failed to parse message:", err, data.toString());
          }
        });
        
        // Handle WebRTC signaling - send answer back to robot
        peer.on('signal', data => {
          console.log('Sending answer to robot');
          socket.emit('answer', JSON.stringify(data));
        });
        
        peer.on('stream', stream => {
          setConnectionStatus('streaming');
          if (videoRef && videoRef.current) {
            videoRef.current.srcObject = stream;
            videoRef.current.play().catch(err => {
              console.error('Error playing video stream:', err);
            });
          }
          setIsConnected(true);
        });
        
        peer.on('connect', () => {
          console.log("WebRTC peer connection established");
          setConnectionStatus('connected');
          setIsConnected(true);
          setError(null);

        
        });

        
        peer.on('close', () => {
          console.warn("WebRTC peer connection closed");
          setConnectionStatus('disconnected');
          setIsConnected(false);
        });

        peer.on('error', err => {
          console.error('Peer error:', err);
          setError('Connection error');
          setConnectionStatus('connection failed');
          setIsConnected(false);
        });
        
        // Signal the offer to start the connection
        peer.signal(JSON.parse(offerData));
        
      } catch (err) {
        console.error('Error creating Peer:', err);
        setError('Failed to create WebRTC peer connection');
        setConnectionStatus('connection failed');
        return;
      }
    });
    
    socket.on('disconnect', () => {
      setConnectionStatus('disconnected');
      setIsConnected(false);
    });
    
    socket.on('connect_error', (err) => {
      console.error('Socket connection error:', err);
      setError('Failed to connect to robot');
      setConnectionStatus('connection failed');
    });
    
    // Cleanup function
    return () => {
      if (peerRef.current) {
        peerRef.current.destroy();
      }
      if (socketRef.current) {
        socketRef.current.disconnect();
      }
    };
  }, [robotIP, videoRef]);



  // Function to send commands
const sendCommand = async (command) => {
  if (!peerRef.current || !isConnected) {
    console.warn("Cannot send command: not connected");
    return false;
  }
  
  try {
    const payload = JSON.stringify({ 
      type: "command", 
      command: command 
    });
    peerRef.current.send(payload);
    console.log("Sent command:", payload);
    return true;
  } catch (err) {
    console.error("Error sending command:", err);
    setError("Failed to send command");
    return false;
  }
};


  return {
    isConnected,
    connectionStatus,
    error,
    sendCommand
  };
}