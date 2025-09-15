import React, {useEffect, useState} from 'react';
import { useNavigate } from "react-router-dom";
import "../styles/HomePage.css";

import { Box, Button } from "@mui/material";
import {RobotCard} from "../components/RobotCard";
import AddButton from '../components/AddButton';
import SettingsButton from '../components/SettingsButton';
import {db} from "../firebaseConfig";
import {query,where, collection, getDocs, doc, getDoc} from "firebase/firestore";
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

        // getting the user profiles
        const profilesRef = doc(db, "profiles", currentUser.email);
        const profileSnap = await getDoc(profilesRef);

        console.log("Current user ID:", currentUser.uid); // Log the current user ID

        const robotsData = [];
        if(profileSnap.exists()){

          //getting the list of robots that the user has access to
          const profileData = profileSnap.data();
          const robotIds = profileData.robots || [];

          //getting the robots information from the uuid 
          for (const ids of robotIds){
            const robotsRef = doc(db, "robots", ids);
            const robotSnap = await getDoc(robotsRef);
            if(robotSnap.exists()){
              robotsData.push({id: robotSnap.id, ...robotSnap.data()});
            }
          }


        }
        //setting the data of the robots for displaying
        setRobots(robotsData);
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
