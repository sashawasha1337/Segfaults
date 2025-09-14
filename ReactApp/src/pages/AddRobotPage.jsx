import React,{ useState } from "react";
import { useNavigate } from "react-router-dom";

import { Box, Button, Grid2, Switch, TextField } from "@mui/material";
import BackButton from "../components/BackButton";

import {collection, addDoc, getDocs} from "firebase/firestore";
import { db } from "../firebaseConfig";
import {useAuth} from "../ContextForAuth.jsx";


function AddRobotPage() {
  const navigate = useNavigate();
  const { currentUser } = useAuth();      
  const [robotName, setRobotName] = useState("");
  const [robotIp, setRobotIp] = useState("");

  const handleAddRobot = async () => {
    try{
      const robotsRef = collection(db, "robots");
      const snapshot = await getDocs(robotsRef);
      
      let maxId = 0;
      snapshot.forEach((doc) => {
        const robot = doc.data();
        const robotId = parseInt(robot.robotID);
        if(!isNaN(robotId) && robotId > maxId){
          maxId = robotId;
        }
      });
      await addDoc(robotsRef, {
        ipAddress: robotIp,
        location: "dock",
        name: robotName,
        robotID: (maxId + 1).toString(),
        status: "idle",
        users: "exampleUser",
      });
      navigate("/HomePage");
    }catch(error){
      console.error("Error adding robot: ", error);
    }
  }

  return (
    <>
      <BackButton path="/HomePage" />

      <h1 style ={{color: "black"}}>
        Add a Robot
      </h1>
      <Box //box for text fields
        component="form"
        sx={{ 
          "& .MuiTextField-root": { m: 4, width: "25ch" },
          display: "flex", 
          flexDirection: "column", 
          alignItems: "center",
          position: "relative",
        }}
        noValidate
        autoComplete="off"
      >

        <TextField
          required
          label="Name"
          placeholder="Name"
          variant="filled"
          value={robotName}
          onChange={(e) => setRobotName(e.target.value)}
        />

        <TextField
          required
          label="IP Address"
          placeholder="IP Address"
          variant="filled"
          value={robotIp}
          onChange={(e) => setRobotIp(e.target.value)}
        />

        
        <Box //box and grid for switches
          sx={{ mt: 2 }}> 
          <Grid2 container spacing={2} alignItems="center">
            <Grid2 item>
              <Switch sx={{transform: "scale(1.5)"}} />
            </Grid2>
            <Grid2 item>
              <span style={{ fontWeight: "bold", color: "black" }}>Return to dock when signal lost</span>
            </Grid2>
          </Grid2>

          <Grid2 container spacing={2} alignItems="center" sx={{ mt: 1 }}>
            <Grid2 item>
              <Switch sx={{transform: "scale(1.5)"}} />
            </Grid2>
            <Grid2 item>
              <span style={{ fontWeight: "bold", color: "black" }}>Robot equipped with lidar</span>
            </Grid2>
          </Grid2>
        </Box>


        <Button variant="contained"
        onClick = {handleAddRobot}
        sx={{
          mt: 6, 
          width: "195px", 
          height: "60px", 
          fontSize: "1.2rem", 
          backgroundColor: "purple",
        }}
          >
          Add Robot
        </Button>
      </Box>
    </>
  );
}

export default AddRobotPage;
