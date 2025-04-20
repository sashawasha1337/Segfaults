import React, {useEffect, useState} from 'react';
import { useNavigate } from "react-router-dom";
import "../styles/HomePage.css";

import { Box, Button } from "@mui/material";
import {RobotCard} from "../components/RobotCard";
import AddButton from '../components/AddButton';
import SettingsButton from '../components/SettingsButton';
import {db} from "../firebaseConfig";
import {collection, getDocs} from "firebase/firestore";


function HomePage() {
  const navigate = useNavigate();

  const [robots, setRobots] = useState([]);

  useEffect(() => {
    const fetchRobots = async() => {
      try{
        const robotsRef = collection(db, "robots");
        const robotsSnapshot = await getDocs(robotsRef);
        const robotData = robotsSnapshot.docs.map(doc => ({
          id: doc.id,
          ...doc.data(),
        }));
        setRobots(robotData);
      }catch (error) {
        console.error("Error fetching robots:", error);
      }
    };
    fetchRobots();
  }, []);


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


      {robots.map((robot, index) => (
        <RobotCard
          key={robot.id}
          imgSrc="https://images.squarespace-cdn.com/content/v1/5a3c1a29f9a61e2987882112/bee5c58a-5b2c-4302-bb18-433dd7bd5f2c/ROSmaster.jpeg"
          imgAlt={`Robot ${robot.id}`}
          title={robot.name || `Robot ${index + 1}`}
          description={`IP: ${robot.ipAddress}`}
          buttonText="FPV/Control"
          link={`/ControlPage/${robot.id}`}
        />  
        ))}
    </div>
  );
};

export default HomePage;
