import React from "react";
import { Typography, Container, Box } from "@mui/material";
import { useNavigate } from "react-router-dom";
import DisplayCard from "../components/DisplayCard";
import BackButton from "../components/BackButton";


function Trash_View_Page() {

  const navigate = useNavigate();

  const metadataLabels = {
    Confidence: "Confidence",
    Category: "Category",
    Location: "Location",
    Coordinates: "Coordinates",
    Timestamp: "Timestamp",
    Robot_ID: "Robot ID",
  }

  const sampleMetadata = {
    Confidence: "92%",
    Category: "Litter",
    Location: "Library",
    Coordinates: "38.56094° N, 77.31693° W",
    Timestamp: "2025-03-14 10:32:00",
    Robot_ID: "RBT-001",
  };

  return (
    <Box
      sx={{
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        width: "100%", // Ensures full width
        height: "100%", // Centers vertically too
        textAlign: "center",
      }}
    >
      <BackButton backURL="/ActivityLog"/>

      <Container maxWidth="sm">
        <Typography variant="h4" align="center" sx={{ mb: 2 }}>
          Robot Detection Display
        </Typography>
        <DisplayCard
          title="Detected Object"
          image="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcTuGFmJvGmySA0ZIEY5j6Jg0zGOwqOaEBFhaw&s"
          metadata={sampleMetadata}
          labels={metadataLabels}
        />
      </Container>
    </Box>
  );
}

export default Trash_View_Page;
