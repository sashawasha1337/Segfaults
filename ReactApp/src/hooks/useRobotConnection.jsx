// hooks/useRobotConnection.js
import { useState, useEffect, useRef } from 'react';
import Peer from 'simple-peer/simplepeer.min.js';
import { io } from 'socket.io-client';

export function useRobotConnection(robotIP, videoRef) {
  const [connectionStatus, setConnectionStatus] = useState('disconnected');
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState(null);
  const peerRef = useRef(null);
  const socketRef = useRef(null);
  const dataChannelRef = useRef(null);

  // Initialize connection
  useEffect(() => {
    if (!robotIP) {
      setConnectionStatus('No robot IP available');
      return;
    }

    setConnectionStatus('connecting');
    
    // Connect to signaling server
    console.log(`Connecting to: http://${robotIP}:5000`);
    const socket = io(`http://${robotIP}:5000`, {
      reconnectionAttempts: 3, // Retry connection 3 times
      timeout: 5000, // Timeout after 5 seconds
    });
    socketRef.current = socket;

    socket.on('connect', () => {
      setConnectionStatus('signaling');
      
      // Create peer connection
      try {
        const peer = new Peer({ 
          initiator: true,
          trickle: false
        });
        peerRef.current = peer;
      } catch (err) {
        console.error('Error creating Peer:', err);
        setError('Failed to create WebRTC peer connection');
        setConnectionStatus('connection failed');
        return;
    }
      
      // Handle WebRTC signaling
      peer.on('signal', data => {
        socket.emit('offer', JSON.stringify(data));
      });
      
      peer.on('stream', stream => {
        setConnectionStatus('streaming');
        if (videoRef && videoRef.current) {
          videoRef.current.srcObject = stream;
        }
        setIsConnected(true);
      });
      
      peer.on('data', data => {
        console.log('Received data:', data.toString());
      });
      
      peer.on('connect', () => {
        setConnectionStatus('connected');
        setIsConnected(true);
        setError(null);

        const dataChannel = peer.createDataChannel('commands');
        dataChannelRef.current = dataChannel;
      });
      
      // ✅ ADD THIS: handle WebRTC disconnect
      peer.on('close', () => {
        console.warn("WebRTC peer connection closed");
        setConnectionStatus('disconnected');
        setIsConnected(false);
      });


      peer.on('error', err => {
        console.error('Peer error:', err);
        setError('Connection error');
        setIsConnected(false);
      });
      
      // Handle answer from robot
      socket.on('answer', data => {
        peer.signal(JSON.parse(data));
      });
    });
    
    socket.on('disconnect', () => {
      setConnectionStatus('disconnected');
      setIsConnected(false);
    });
    
    socket.on('connect_error', (err) => {
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
    if (!robotIP) {
      setError('No robot connection available');
      return;
    }
    
    try {
      // Try to send via WebRTC data channel if available
      if (dataChannelRef.current && isConnected) {
        dataChannelRef.current.send(command);
        return true;
      }
      return false;
    } catch (error) {
      setError('Failed to send command');
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
