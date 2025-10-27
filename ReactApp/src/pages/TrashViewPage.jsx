import React, { useState, useEffect } from "react";
import { useNavigate, useLocation } from "react-router-dom";
import { Typography, Container, Box } from "@mui/material";
import DisplayCard from "../components/DisplayCard";
import BackButton from "../components/BackButton";
import { db } from "../firebaseConfig";
import { doc, getDoc } from "firebase/firestore";

function TrashViewPage() {
  const navigate = useNavigate();
  const { state } = useLocation(); // contains eventId, imageUrl, robotId, etc.

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

  const metadataLabels = {
    Confidence: "Confidence",
    Category: "Category",
    Location: "Location",
    Coordinates: "Coordinates",
    Timestamp: "Timestamp",
    Robot_ID: "Robot ID",
  };

  useEffect(() => {
    let alive = true;
    const run = async () => {
      setLoading(true);
      try {
        let img = state?.imageUrl ?? null;
        let rob = state?.robotId ?? "Unknown";
        let cat = state?.category ?? "Unknown";
        let loc = state?.location ?? "Unknown";
        let timeTxt = "";
        let confidence = state?.confidence ?? null;

        if (state?.eventId) {
          const ref = doc(db, "events", state.eventId);
          const snap = await getDoc(ref);
          if (snap.exists()) {
            const data = snap.data();
            img = data.image_url ?? null;
            rob = data.robotId ?? rob;
            cat = data.category ?? cat;
            loc = data.location ?? loc;
            confidence = data.confidence ?? confidence;
            const t =
              data.time?.toDate?.() instanceof Date
                ? data.time.toDate()
                : typeof data.time === "string"
                ? new Date(data.time)
                : null;
            if (t instanceof Date && !isNaN(t)) {
              timeTxt = t.toLocaleString();
            }
          }
        } else if (state?.timeMS) {
          const t = new Date(state.timeMS);
          if (!isNaN(t)) timeTxt = t.toLocaleString();
        }

        if (alive) {
          setImageUrl(img);
          setMetadata({
            Confidence: confidence,
            Category: cat,
            Location: loc,
            Coordinates: "",
            Timestamp: timeTxt,
            Robot_ID: rob,
          });
        }
      } catch (e) {
        console.error("Error loading event:", e);
      } finally {
        alive && setLoading(false);
      }
    };
    run();
    return () => {
      alive = false;
    };
  }, [db, state]);

  return (
    <Box
      sx={{
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        width: "100%", // Ensures full width
        height: "100%",// Centers vertically too 
        textAlign: "center",
      }}
    >
      <BackButton path="/ActivityLogPage"/>

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

export default TrashViewPage;