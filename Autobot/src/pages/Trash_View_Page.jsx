import React from "react";
import { Card, CardMedia, CardContent, Typography, Container, Box } from "@mui/material";
import ArrowBackIcon from "@mui/icons-material/ArrowBack"; 
import IconButton from "@mui/material/IconButton";
import { useNavigate } from "react-router-dom";


const DetectionCard = ({ image, metadata }) => {
  return (
    <Card sx={{ maxWidth: 500, boxShadow: 3 }}>
      <CardMedia
        component="img"
        height="300"
        image={image || "https://https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcTuGFmJvGmySA0ZIEY5j6Jg0zGOwqOaEBFhaw&s.placeholder.com/500"} // Default placeholder
        alt="Detected Object"
      />
      <CardContent>
        <Typography variant="h6" component="div">
          Detected Object
        </Typography>
        <Box sx={{ mt: 1 }}>
          {metadata ? (
            Object.entries(metadata).map(([key, value]) => (
              <Typography key={key} variant="body2" color="text.secondary">
                <strong>{key}:</strong> {value}
              </Typography>
            ))
          ) : (
            <Typography variant="body2" color="text.secondary">
              No metadata available.
            </Typography>
          )}
        </Box>
      </CardContent>
    </Card>
  );
};

function Trash_View_Page() {
  const navigate = useNavigate();

  const sampleMetadata = {
    Confidence: "92%",
    Category: "Litter",
    Location: "Library",
    Coordinates: "38.56094° N, 77.31693° W",
    Timestamp: "2025-03-14 10:32:00",
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
      <IconButton
        onClick={() => navigate("/")} // Ensure navigate is used correctly
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
      <Container maxWidth="sm">
        <Typography variant="h4" align="center" sx={{ mb: 2 }}>
          Robot Detection Display
        </Typography>
        <DetectionCard
          image="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcTuGFmJvGmySA0ZIEY5j6Jg0zGOwqOaEBFhaw&s"
          metadata={sampleMetadata}
        />
      </Container>
    </Box>
  );
}

export default Trash_View_Page;
