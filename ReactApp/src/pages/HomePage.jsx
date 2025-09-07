import React, {useEffect, useState} from 'react';
import { useNavigate } from "react-router-dom";
import "../styles/HomePage.css";

import { Box, Button } from "@mui/material";
import {RobotCard} from "../components/RobotCard";
import AddButton from '../components/AddButton';
import SettingsButton from '../components/SettingsButton';
import {db} from "../firebaseConfig";
import {query,where, collection, getDocs} from "firebase/firestore";
import { useAuth } from "../ContextForAuth.jsx"; 


function HomePage() {
  const navigate = useNavigate();

  const [robots, setRobots] = useState([]);
  const { currentUser } = useAuth(); 
  useEffect(() => {
    const fetchRobots = async() => {
      try{
        if(!currentUser){
          console.error("No current user found.");
          return;
        }
        const robotsRef = collection(db, "robots");
        console.log("Current user ID:", currentUser.uid); // Log the current user ID
        const robotsSnapshot = await getDocs(query(robotsRef,where("users","array-contains",currentUser.uid)));
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
  }, [currentUser]);


  return (
    <div className = "homepage">
      <SettingsButton path="/UserSettingsPage" /> 
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

      {robots.length === 0 ? (
      <p style={{ marginTop: "200px", textAlign: "center" }}>
        No robots found. Click the + button to add one!
        </p>
      ) : (
      robots.map((robot, index) => (
        <RobotCard
          key={robot.id}
          imgSrc="https://images.squarespace-cdn.com/content/v1/5a3c1a29f9a61e2987882112/bee5c58a-5b2c-4302-bb18-433dd7bd5f2c/ROSmaster.jpeg"
          imgAlt={`Robot ${robot.id}`}
          title={robot.name || `Robot ${index + 1}`}
          description={`IP: ${robot.ipAddress}`}
          buttonText="FPV/Control"
          link={`/ControlPage/${robot.id}`}
        />
      ))
    )}
  </div>
);
};

export default HomePage;
