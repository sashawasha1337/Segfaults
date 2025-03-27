import React, { useState, useEffect } from "react";
import { Typography, Container, Box } from "@mui/material";
import { useNavigate } from "react-router-dom";
import DisplayCard from "../components/DisplayCard";
import BackButton from "../components/BackButton";
import { db } from "../firebaseConfig"; // Import Firestore
import { collection, query, orderBy, limit, getDocs } from "firebase/firestore";


function Trash_View_Page() {

  const [imageUrl, setImageUrl] = useState(null);
  const [loading, setLoading] = useState(true);
  const [metadata, setMetadata] = useState({
  Confidence: "",
  Category: "",
  Location: "",
  Coordinates: "",
  Timestamp: "",
  Robot_ID: "",
});

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
  // fetch image from database
  useEffect(() => {
    const fetchImage = async () => {
      setLoading(true);
      try {
        const imagesRef = collection(db, "robot_images");
        const q = query(imagesRef, orderBy("timestamp", "desc"), limit(1));
        const querySnapshot = await getDocs(q);

        if (!querySnapshot.empty) {
          setImageUrl(querySnapshot.docs[0].data().url);
        } else {
          console.warn("No images found in Firestore.");
        }
      } catch (error) {
        console.error("Error fetching image:", error);
      }
      setLoading(false);
    };

    fetchImage();
  }, []);

  //fetch metadata from database
  useEffect(() => {
    const fetchMetadata = async () => {
      setLoading(true);
      try{
        const metadataRef = collection(db, "GPS_data");
        const q = query(metadataRef, orderBy("Timestamp", "desc"), limit(1));
        const querySnapshot = await getDocs(q);

        if(!querySnapshot.empty){
          const data = querySnapshot.docs[0].data();
          //maybe we can store everything as strings?
          setMetadata({
            Confidence: data.Confidence,
            Category: data.Category,
            Location: data.Location,
            Coordinates: data.Coordinates 
            ? `${data.Coordinates.latitude}, ${data.Coordinates.longitude}` 
            : "No coordinates found", //convert firestore geopoint to redable string
            Timestamp: data.Timestamp.toDate().toLocaleString(), //convert firestore timestamp to readable string
            Robot_ID: data.Robot_ID

          });
        }else{
          console.warn("No metadata found in Firestore.");
        }
      }catch(error){
        console.error("Error fetching metadata:", error); 
      }
      setLoading(false);
    }
    fetchMetadata();
  }, []);

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
          image={imageUrl || "https://picsum.photos/300"} // Show placeholder if no image found
          metadata={metadata}
          labels={metadataLabels}
        />
      </Container>
    </Box>
  );
}

export default Trash_View_Page;
