import * as React from "react";
import Box from "@mui/material/Box";
import TextField from "@mui/material/TextField";
import ArrowBackIcon from "@mui/icons-material/ArrowBack"; 
import IconButton from "@mui/material/IconButton";
import { useNavigate } from "react-router-dom";
import Switch from '@mui/material/Switch';
import { FormControlLabel } from "@mui/material";
import Button from '@mui/material/Button';
import Grid2 from "@mui/material/Grid";

function AddRobots() {
  const navigate = useNavigate();

  return (
    <>
      <IconButton
        onClick={() => navigate("/")}
        sx={{
          position: "absolute",
          top: 50,
          right: 75,
          color: "black",
          padding: "40",
        }}
      >
        <ArrowBackIcon fontSize="large"/>
      </IconButton>

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


        <Button variant="contained" sx={{
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

export default AddRobots;
