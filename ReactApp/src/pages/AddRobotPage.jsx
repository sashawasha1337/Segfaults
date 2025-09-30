import React,{ useState } from "react";
import { useNavigate } from "react-router-dom";

import { Box, Button, Grid2, Switch, TextField } from "@mui/material";
import BackButton from "../components/BackButton";

import {collection, addDoc, getDocs, doc, updateDoc, arrayUnion, setDoc, serverTimestamp } from "firebase/firestore";
import { db } from "../firebaseConfig";
import {useAuth} from "../ContextForAuth.jsx";
import { Merge } from "@mui/icons-material";


function AddRobotPage() {
  const navigate = useNavigate();
  const { currentUser } = useAuth();      
  const [robotName, setRobotName] = useState("");
  const [robotIp, setRobotIp] = useState("");
  const [email, setEmail] = useState("");
  const [admin, setAdmin] = useState("");
  const [emailError, setEmailError] = useState("");
  const [emailList, setEmailList] = useState([]);
  const norm = (s) => (s || "").trim().toLowerCase();

  const emailRegex = /^[a-zA-Z0-9._-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,6}$/;


  // Tests the entered email
  function handleAddEmail(){
    if (!emailRegex.test(email)) {
      setEmailError("Please enter a valid email address");
      return;
    }
    if (emailList.some(e => e.toLowerCase() === email.toLowerCase())) {
      setEmailError("Already Added");
      setEmail("");
      return;
    }
    setEmailError("");
    setEmailList([...emailList, email.toLowerCase()]);
    setEmail("");
  }

  // Ability to remove a email
  function removeEmail(value){
    setEmailList(emailList.filter(e => e.toLowerCase() !== value.toLowerCase()));
  }


  // Handles adding robot to users profiles and robots collection
  const handleAddRobot = async () => {
  try {
    if (!currentUser) throw new Error("Not signed in");

    // normalize + ensure creator is in users
    const emails = Array.from(new Set([norm(currentUser.email), ...emailList.map(norm)]));

    const robotsRef = collection(db, "robots");
    const docRef = await addDoc(robotsRef, {
      ipAddress: robotIp,
      location: "dock",
      name: robotName,
      status: "idle",
      users: emails,
      admin: norm(currentUser.email),
      createdAt: serverTimestamp(),
    });

    // add robot docId to each profile
    for (const e of emails) {
      await setDoc(
        doc(db, "profiles", e),
        { robots: arrayUnion(docRef.id) },
        { merge: true }
      );
    }

    navigate("/HomePage");
  } catch (error) {
    console.error("Error adding robot: ", error);
  }
};

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

        <div>
          <TextField
            error={!!emailError}
            label="Share to"
            placeholder="Email"
            variant="standard"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            helperText={emailError}
          />
          <Button variant="contained"
          onClick = {handleAddEmail}
          sx={{
            mt: 6, 
            backgroundColor: "blue",
          }}
            >
            Add Email
          </Button>
        </div>
        <div>
          {emailList.map((emailPart => (
            <div key={emailPart}>
              {emailPart}
              <Button onClick={ () => removeEmail(emailPart)}>Remove</Button>
            </div>
          )))}
        </div>


        
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
