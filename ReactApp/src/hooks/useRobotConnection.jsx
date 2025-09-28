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

        // Listen for additional data channels
        peer._pc.ondatachannel = (event) => {
          const channel = event.channel;
          console.log(`Data channel received: ${channel.label}`);

          if (channel.label === "status") {
            channel.onopen = () => {
              console.log("Status data channel is open (browser)");
            };
          
            channel.onmessage = (ev) => {
              console.log("STATUS EVENT TRIGGERED");
              console.log("Raw status message from robot:", ev.data, typeof ev.data);
            
              try {
                let text;
                if (typeof ev.data === "string") {
                  text = ev.data;
                } else if (ev.data instanceof ArrayBuffer) {
                  text = new TextDecoder().decode(ev.data);
                } else if (ev.data && typeof ev.data.text === "function") {
                  ev.data.text().then((t) => {
                    console.log("Blob status text:", t);
                    parseStatus(t);
                  });
                  return;
                } else {
                  console.warn("Unknown status payload type:", ev.data);
                  return;
                }
              
                parseStatus(text);
              } catch (err) {
                console.error("Error handling status message:", err);
              }
            };
          
            const parseStatus = (text) => {
              console.log("Parsing status:", text);
              try {
                const msg = JSON.parse(text);
                console.log("Parsed status object:", msg);
                if (msg?.type === "status") {
                  onBatteryUpdate?.(msg.batteryVoltage);
                  onWifiUpdate?.(msg.wifiStrength);
                }
              } catch (e) {
                console.error("Failed to parse JSON:", e, text);
              }
            };
          }


          if (channel.label === "commands") {
            peerRef.current.commandChannel = channel; // Store reference for sending commands
            channel.onopen = () => {
              console.log("Commands data channel is open");
            }
            channel.onclose = () => {
              console.log("Commands data channel is closed");
            }
            channel.onerror = (err) => {
              console.error("Commands data channel error:", err);
            }
          }
        };
        
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
        
          // Debug peer connection state
          console.log('Peer._pc:', peer._pc);
          if(peer && peer._pc.ondatachannel) {
            console.log('Ondatachannel handler exists');
          }
        
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
    if (!robotIP) {
      setError('No robot connection available');
      return false;
    }
    
    try {
      // Try to send via WebRTC data channel if available
      if (peerRef.current?.commandChannel && peerRef.current?.commandChannel?.readyState === "open") {
        console.log("Sending command via WebRTC:", command);
        peerRef.current.commandChannel.send(command);
        return true;
      } else {
        console.warn('WebRTC not connected, cannot send command');
        return false;
      }
    } catch (error) {
      console.error('Error sending command:', error);
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