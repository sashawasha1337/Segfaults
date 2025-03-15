import * as React from "react";
import Box from "@mui/material/Box";
import TextField from "@mui/material/TextField";
import ArrowBackIcon from "@mui/icons-material/ArrowBack"; 
import IconButton from "@mui/material/IconButton";
import { useNavigate } from "react-router-dom";
import Switch from '@mui/material/Switch';
import { FormControlLabel } from "@mui/material";

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

      <Box
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
        <FormControlLabel
          control={<Switch />}
          label="Return to dock when signal lost"
          sx={{
            display: "flex",  
            justifyContent: "center",
            alignItems: "center",
            "& .MuiFormControlLabel-label": {
              color: "black",
              fontWeight: "bold",
            }
          }}
        />
        <FormControlLabel
          control={<Switch />}
          label="Robot eqipped with lidar"
          sx={{
            display: "flex",  
            justifyContent: "center",
            alignItems: "center",
            "& .MuiFormControlLabel-label": {
              color: "black",
              fontWeight: "bold",
            }
          }}
        />
      </Box>
    </>
  );
}

export default AddRobots;
