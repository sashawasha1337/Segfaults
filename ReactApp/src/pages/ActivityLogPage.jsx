import * as React from "react";
import { useNavigate } from "react-router-dom";

import { Box } from "@mui/material";
import BackButton from "../components/BackButton";
import EventTable from "../components/EventTable";


function ActivityLogPage() {
  const navigate = useNavigate();

  const events =[
    { eventId: 1, robotId: "RBT-01", category: "Litter", location: "Location 1", time: "2023-10-01 10:00", eventId: 1 },
    { eventId: 2, robotId: "RBT-02", category: "Litter", location: "Location 2", time: "2023-10-02 11:00", eventId: 2 },
    { eventId: 3, robotId: "RBT-02", category: "Litter", location: "Location 3", time: "2023-10-03 12:00", eventId: 3 },
    { eventId: 4, robotId: "RBT-01", category: "Litter", location: "Location 4", time: "2023-10-04 13:00", eventId: 4 },
    { eventId: 5, robotId: "RBT-01", category: "Litter", location: "Location 5", time: "2023-10-05 14:00", eventId: 5 },
  ]

  return (
    <>
      <BackButton path="/HomePage" />

      <Box sx={{marginTop: '100px', padding: '10px'}}>
        <h1>Activity Log</h1>
        <EventTable events={events} />
      </Box>
    </>
  );
}

export default ActivityLogPage;
