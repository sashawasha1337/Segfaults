import * as React from "react";
import { useNavigate } from "react-router-dom";

import { Box, Button, Grid2, Switch, TextField } from "@mui/material";
import BackButton from "../components/BackButton";


function AddRobotPage() {
  const navigate = useNavigate();

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
        />

        <TextField
          required
          label="IP Address"
          placeholder="IP Address"
          variant="filled"
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
        onClick={() => navigate("/HomePage")}
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
