import React from "react";
import { Box, Typography, Button, IconButton } from "@mui/material";
import ArrowBackIcon from "@mui/icons-material/ArrowBack";
import { useNavigate } from "react-router-dom";

function RobotSettings() {
  const navigate = useNavigate();

  return (
    <Box
      sx={{
        width: "100%",
        minHeight: "100vh",
        bgcolor: "#FFFFFF",
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        position: "relative",
        paddingTop: 4,
      }}
    >
      <IconButton
        onClick={() => navigate("/")}
        sx={{
          position: "absolute",
          top: 0,
          right: -500,
          color: "black",
          padding: "40",
        }}
      >
        <ArrowBackIcon fontSize="large" />
      </IconButton>

      <Button
        // Delete Robot Button
        variant="contained"
        sx={{
          backgroundColor: "#EC221F",
          border: "1px solid #C00F0C",
          borderRadius: "8px",
          width: 194,
          height: 69,
          mb: 2,
          "&:hover": {
            backgroundColor: "#C00F0C",
          },
        }}
      >
        Delete Robot
      </Button>

      <Button
        // Shutdown Robot Button
        variant="contained"
        sx={{
          backgroundColor: "#EC221F",
          border: "1px solid #C00F0C",
          borderRadius: "8px",
          width: 194,
          height: 69,
          mb: 2,
          "&:hover": {
            backgroundColor: "#C00F0C",
          },
        }}
      >
        Shutdown
      </Button>

      <Button
        // Update Software Button
        variant="contained"
        sx={{
          backgroundColor: "#5A5A5A",
          border: "1px solid #C00F0C",
          borderRadius: "8px",
          width: 194,
          height: 69,
          mb: 2,
          "&:hover": {
            backgroundColor: "#4A4A4A",
          },
        }}
      >
        Update Software
      </Button>

      <Button
        // Change Network Name Button
        variant="contained"
        sx={{
          backgroundColor: "#5A5A5A",
          border: "1px solid #C00F0C",
          borderRadius: "8px",
          width: 194,
          height: 69,
          mb: 2,
          "&:hover": {
            backgroundColor: "#4A4A4A",
          },
        }}
      >
        Change Network Name
      </Button>

      <Box
        sx={{
          mt: 25,
          mb: 4,
          textAlign: "center",
        }}
      >
        <Typography
          sx={{
            color: "#000000",
            fontFamily: "Inter",
            fontSize: 16,
            lineHeight: "100%",
          }}
        >
          Operating System: Ubuntu 24.04
        </Typography>
        <Typography
          sx={{
            color: "#000000",
            fontFamily: "Inter",
            fontSize: 16,
            lineHeight: "100%",
          }}
        >
          ROS Version: 19.02 Jazzy Jalisco
        </Typography>
        <Typography
          sx={{
            color: "#000000",
            fontFamily: "Inter",
            fontSize: 16,
            lineHeight: "100%",
          }}
        >
          Date Added To Network: 02/27/2025
        </Typography>
      </Box>
    </Box>
  );
}

export default RobotSettings;
