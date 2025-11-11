import React, { useState, useRef, useEffect, use } from "react";
import { Box,  Button, Card, CardMedia, Grid, Typography } from "@mui/material";
import { ArrowBack, ArrowDownward, ArrowForward, ArrowUpward } from "@mui/icons-material";
import BackButton from "../components/BackButton";
import SettingsButton from '../components/SettingsButton';
import LogoutButton from "../components/LogoutButton";
import { db } from "../firebaseConfig"; // Import Firestore
import { doc, getDoc } from "firebase/firestore"; // Import Firestore methods
import { useParams } from "react-router-dom"; // Import useParams to get URL parameters
import { useRobotConnection }  from "../hooks/useRobotConnection"; // Import custom hook for WebRTC connection

const ControlPage = () => {


  const { robotID } = useParams(); // Get the robot ID from the URL parameters
  const [robotIP, setRobotIP] = useState(""); // State to store the robot's IP address
  const  videoRef = useRef(null); // Ref to access the video element
  const [connection, setConnection] = useState(null); // State to manage the connection

  const [batteryVoltage, setBatteryVoltage] = useState(null); // State to store battery voltage
  const [wifiStrength, setWifiStrength] = useState(null); // State to store WiFi strength
 
  // Function to fetch the robot's IP address from Firestore using its ID
  const fetchRobotIP = async () => {
    try {
      const robotDocRef = doc(db, "robots", robotID); // Reference the document directly
      const robotDoc = await getDoc(robotDocRef); // Fetch the document
      if (robotDoc.exists()) {
        const robotData = robotDoc.data();
        setRobotIP(robotData.ipAddress); // Set the robot's IP address in state
      } else {
        console.warn("No robot found with the given ID.");
      }
    } catch (error) {
      console.error("Error fetching robot IP address:", error);
    }
  };


  useEffect(() => {
      fetchRobotIP();
  }, []);

  // Call the hook at the top level. The hook can internally handle when robotIP changes.
  const { isConnected, connectionStatus, error, sendCommand } = useRobotConnection(
    robotIP, 
    videoRef,
    setBatteryVoltage,
    setWifiStrength
  );

  const adjustRobotDirection = async (command) => {
      const sent = await sendCommand(command); // Send the command to the robot
      if (!sent) {
          console.error("Failed to send movement command:", command);
          console.error("Error details:", error);
          console.error("Connection Details:", { isConnected, connectionStatus });

          
      }
  };




  // JSX code for the page
  return (
    <div style ={{
        padding: "20px",
        display: "flex",
        flexDirection: "column",
        justifyContent: "center",
        alignItems: "center"
      }}
    >
            <Box
              sx={{
                position: "absolute",
                top: 30,
                left: 30,
                zIndex: 1000,
              }}
            >
              <LogoutButton />
            </Box>
      <SettingsButton path={`/RobotSettingPage/${robotID}`} />
      <BackButton path="/HomePage" />

      <div style={{position:"fixed", top: "0", left: "50%",transform: "translateX(-50%)"}}>
        <Typography variant="h3" style={{textAlign: "center"}}>Robot Control</Typography>
      </div>
      
      <Card 
        style={{
          width: "600px", 
          height: "400px",
          margin: "20px",
          border: "1px solid black",
          borderRadius: "10px",
          boxShadow: "0 4px 8px rgba(0, 0, 0, 0.2)",
          overflow: "hidden",
        }}>
        { robotIP ? (
          <video
            ref ={videoRef}
            muted
            autoPlay
            playsInline
            style={{
              width: "100%",
              height: "100%",
              objectFit: "cover"
            }}
          />
          ) : (
            <Typography variant="h5" style={{textAlign: "center"}}>No Live Feed Detected...</Typography>
          )}
      </Card>

      <Box 
        style={{
          display: "flex",
          justifyContent: "space-between",
          borderRadius: "10px",
          marginBottom: "20px",
          padding: "10px",
          width: "300px",
        }}
      >
        <Typography variant="body1">Connection Status: {isConnected ? "Connected" : "Disconnected"}</Typography>
        <Typography variant="body1">Battery Voltage: {batteryVoltage ?? "N/A"}</Typography>
        <Typography variant="body1">Wifi Strength: {wifiStrength != null ? `${wifiStrength} dB` : "N/A"}</Typography>
      </Box>

      <Grid container direction="column" justifyContent="space-between" alignItems="center" >
        <Grid item >
          <Button onClick={() => adjustRobotDirection("forward")}
            variant="contained"
            style={{
              display: "flex",
              justifyContent:"center",
              alignItems: "center"
            }}
          >
            <ArrowUpward style={{ fontSize: 50 }} />
          </Button>
        </Grid>

        <Grid container spacing={12} justifyContent="center" alignItems="center">
          <Grid item>
            <Button onClick={() => adjustRobotDirection("left")}
              variant="contained"
              style={{
                display: "flex",

              }}
            >
              <ArrowBack style={{ fontSize: 50 }} />
            </Button>
          </Grid>

          <Grid item>
            <Button onClick={() => adjustRobotDirection("right")}
              variant="contained"
              style={{
                display: "flex",

              }}
            >
              <ArrowForward style={{ fontSize: 50 }} />
            </Button>
          </Grid>
        </Grid>

        <Grid item>
          <Button onClick={() => adjustRobotDirection("back")}
            variant="contained"
            style={{
              display: "flex",
              justifyContent:"center",
              alignItems: "center"
            }}
          >
            <ArrowDownward style={{ fontSize: 50 }} />
          </Button>
        </Grid>  
      </Grid>
    </div>    
  );
};

export default ControlPage;