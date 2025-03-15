import * as React from "react";
import Box from "@mui/material/Box";
import TextField from "@mui/material/TextField";
import ArrowBackIcon from "@mui/icons-material/ArrowBack"; 
import IconButton from "@mui/material/IconButton";
import { useNavigate } from "react-router-dom";
import Switch from '@mui/material/Switch';
import { FormControlLabel } from "@mui/material";
import EventTable from "../components/EventTable";

function ActivityLog() {
  const navigate = useNavigate();

  const events =[
    { type: "Event 1", location: "Location 1", time: "2023-10-01 10:00", eventId: 1 },
    { type: "Event 2", location: "Location 2", time: "2023-10-02 11:00", eventId: 2 },
    { type: "Event 3", location: "Location 3", time: "2023-10-03 12:00", eventId: 3 },
    { type: "Event 4", location: "Location 4", time: "2023-10-04 13:00", eventId: 4 },
    { type: "Event 5", location: "Location 5", time: "2023-10-05 14:00", eventId: 5 },
  ]

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

    <Box sx={{marginTop: '100px', padding: '10px'}}>
        <h1>Activity Log</h1>
        <EventTable events={events} />
    </Box>
    </>
  );
}

export default ActivityLog;
