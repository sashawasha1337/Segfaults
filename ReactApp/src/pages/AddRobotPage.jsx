import React,{ useState } from "react";
import { useNavigate } from "react-router-dom";

import { Box, Button, Grid2, Switch, TextField } from "@mui/material";
import BackButton from "../components/BackButton";

import {collection, addDoc, getDocs, doc, updateDoc, arrayUnion, setDoc, serverTimestamp } from "firebase/firestore";
import { db } from "../firebaseConfig";
import {useAuth} from "../ContextForAuth.jsx";
import { Snackbar, Alert } from "@mui/material";
import ContentCopyIcon from "@mui/icons-material/ContentCopy";
import CheckIcon from "@mui/icons-material/Check";
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
  const [err, setErr] = React.useState("");
  const norm = (s) => (s || "").trim().toLowerCase();

  const emailRegex = /^[a-zA-Z0-9._-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,6}$/;

  const [successOpen, setSuccessOpen] = useState(false);
  const [successMsg, setSuccessMsg] = useState("");
  const [newRobotId, setNewRobotId] = useState("");
  const [copied, setCopied] = useState(false);

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

  // Handles copying robot ID to clipboard
  const handleCopyId = async () => {
  try {
    if (!newRobotId) return;
    await navigator.clipboard.writeText(newRobotId);
    setCopied(true);
    setTimeout(() => setCopied(false), 1500);
  } catch (e) {
    console.error("Clipboard copy failed:", e);
  }
};
  // Handles adding robot to users profiles and robots collection
  const handleAddRobot = async () => {
  if (!robotName.trim() || !robotIp.trim()) {
    setErr("Please fill out both the Name and IP Address fields before adding a robot.");
    return;
  }    
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
    setNewRobotId(docRef.id);
    setSuccessMsg("Robot added successfully!");
    setSuccessOpen(true);
    // clear form fields
    setRobotName("");
    setRobotIp("");
    setEmail("");
    setEmailList([]);
    setEmailError("");

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
          error={!robotName.trim() && err !== ""}
          helperText={!robotName.trim() && err !== "" ? "Name is required" : ""}
        />

        <TextField
          required
          label="IP Address"
          placeholder="IP Address"
          variant="filled"
          value={robotIp}
          onChange={(e) => setRobotIp(e.target.value)}
          error={!robotIp.trim() && err !== ""}
          helperText={!robotIp.trim() && err !== "" ? "IP Address is required" : ""}
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
        disabled={!robotName.trim() || !robotIp.trim()}
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
      <Snackbar
        open={successOpen}
        onClose={() => setSuccessOpen(false)}
        anchorOrigin={{ vertical: "bottom", horizontal: "center" }}
      >
      <Alert
        severity="success"
        variant="filled"
        onClose={() => setSuccessOpen(false)}
        sx={{ width: "100%", alignItems: "flex-start" }}
        action={
          <Box sx={{ display: "flex", gap: 1 }}>
            <Button
             color="inherit"
             size="small"
              startIcon={copied ? <CheckIcon /> : <ContentCopyIcon />}
              onClick={handleCopyId}
             disabled={!newRobotId}
            >
             {copied ? "Copied" : "Copy ID"}
            </Button>
            <Button
              color="inherit"
              size="small"
              onClick={() => {
                setSuccessOpen(false);
                navigate("/HomePage");
             }}
            >
             Go to Home
            </Button>
          </Box>
    }   
      >
        <Box sx={{ display: "flex", flexDirection: "column", gap: 1 }}>
          <span>{successMsg || "Robot added successfully"}</span>
          <Box sx={{ display: "flex", alignItems: "center", gap: 1 }}>
           <span style={{ opacity: 0.9 }}>Robot ID:</span>
           <Box
             sx={{
                fontFamily: "monospace",
               px: 1.25,
                py: 0.25,
                borderRadius: 1,
                bgcolor: "rgba(255,255,255,0.15)",
                border: "1px solid rgba(255,255,255,0.3)",
                maxWidth: 320,
                overflow: "hidden",
                textOverflow: "ellipsis",
                whiteSpace: "nowrap",
              }}
              title={newRobotId}
            >
             {newRobotId || "—"}
            </Box>
          </Box>
        </Box>
      </Alert>
    </Snackbar>
        </>
      );
    }

export default AddRobotPage;
