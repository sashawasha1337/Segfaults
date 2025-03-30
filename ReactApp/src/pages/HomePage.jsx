import React from 'react';
import { useNavigate } from "react-router-dom";
import "../styles/HomePage.css";

import { Box, Button } from "@mui/material";
import {RobotCard} from "../components/RobotCard";
import AddButton from '../components/AddButton';
import SettingsButton from '../components/SettingsButton';


function HomePage() {
  const navigate = useNavigate();

  return (
    <div className = "homepage">
      <SettingsButton path="/" /> {/* path to user settings page needs to be added */}
      <AddButton path="/AddRobotPage" />
      
      <Button variant="contained"
        onClick={() => navigate("/ActivityLogPage")}
        sx={{
          position: "absolute",
          top: 100,
          right: 35,
          textTransform: "none",
          borderRadius: "10px",
          width: "125px",
          height: "50px",
          fontSize: "1.0rem",
          backgroundColor: "black",
        }}
      >
        Activity Log
      </Button>

      <RobotCard
        imgSrc="https://images.squarespace-cdn.com/content/v1/5a3c1a29f9a61e2987882112/bee5c58a-5b2c-4302-bb18-433dd7bd5f2c/ROSmaster.jpeg"
        imgAlt="Robot"
        title="Robot 1"
        description="Segfaults UGV"
        buttonText="FPV/Control"
        link="/ControlPage"
      />

      <RobotCard
        imgSrc="https://images.squarespace-cdn.com/content/v1/5a3c1a29f9a61e2987882112/bee5c58a-5b2c-4302-bb18-433dd7bd5f2c/ROSmaster.jpeg"
        imgAlt="Robot"
        title="Robot 2"
        description="Segfaults UGV"
        buttonText="FPV/Control"
        link="/ControlPage"
      />
    </div>
  );
};

export default HomePage;
