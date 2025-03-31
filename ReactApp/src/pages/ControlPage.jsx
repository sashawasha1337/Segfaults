
import React, { useState, useRef, useEffect } from "react";
import { Button, Card, CardMedia, Grid, Typography } from "@mui/material";
import { ArrowBack, ArrowDownward, ArrowForward, ArrowUpward } from "@mui/icons-material";
import BackButton from "../components/BackButton";
import SettingsButton from '../components/SettingsButton';
import { db } from "../firebaseConfig"; // Import Firestore
import { collection, query, getDocs, where } from "firebase/firestore";
import { useParams } from "react-router-dom"; // Import useParams to get URL parameters
import { RobotConnection }  from "../hooks/RobotConnection"; // Import custom hook for WebRTC connection

const ControlPage = () => {


  const { robotID } = useParams(); // Get the robot ID from the URL parameters
  const [robotIP, setRobotIP] = useState(""); // State to store the robot's IP address
  const  videoRef = useRef(null); // Ref to access the video element
 
  // Function to fetch the robot's IP address from Firestore using its ID
  const fetchRobotIP = async () => {
      try {
          const robotsRef = collection(db, "robots");
          const q = query(robotsRef, where("robotID", "==", robotID));
          const querySnapshot = await getDocs(q);
          if (!querySnapshot.empty) {
              const robotData = querySnapshot.docs[0].data();
              setRobotIP(robotData.ipAddress); // Set the robot's IP address in state
          } else {
              console.warn("No robot found with the given ID.");
          }
      }
      catch (error) {
          console.error("Error fetching robot IP address:", error);
      }
  };


  useEffect(() => {
      fetchRobotIP();
  }, [fetchRobotIP]);


  const { 
      isConnected,
      connectionStatus,
      error: connectionError,
      sendCommand
  } = RobotConnection(robotIP, videoRef); // Custom hook to manage WebRTC connection

  const adjustRobotDirection = async (command) => {
      const sent = await sendCommand(command); // Send the command to the robot
      if (!sent) {
          console.error("Failed to send command:", command);
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
      <SettingsButton path="/RobotSettingPage" />
      <BackButton path="/HomePage" />

      <div style={{position:"fixed", top: "0", left: "50%",transform: "translateX(-50%)"}}>
        <Typography variant="h3" style={{textAlign: "center"}}>Robot Control</Typography>
      </div>
      
      <Card style={{ width: "100%", margin: "20px", border: "1px solid black" }}>
        <video
          ref ={videoRef}
          autoPlay
          playsInline
          style={{
            width: "100%",
            height: "auto",
            borderRadius: "10px",
            border: "1px solid black"
          }}
        />
        
      </Card>

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

        <Grid container spacing={2} justifyContent="space-between" alignItems="center">
          <Grid item xs={6}>
            <Button onClick={() => adjustRobotDirection("left")}
              variant="contained"
              style={{display: "flex"}}
            >
              <ArrowBack style={{ fontSize: 50 }} />
            </Button>
          </Grid>

          <Grid item xs={6}>
            <Button onClick={() => adjustRobotDirection("right")}
              variant="contained"
              style={{display: "flex"}}
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