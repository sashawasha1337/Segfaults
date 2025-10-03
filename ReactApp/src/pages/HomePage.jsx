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
import { updateDoc, writeBatch, arrayRemove } from "firebase/firestore";


function HomePage() {
  const navigate = useNavigate();
  const norm = (s) => (s || "").toString().trim().toLowerCase(); // normalize emails to lower case 

  const handleDeleteRobot = async (robotId) => {
    try{
      if(!currentUser){
        alert("You must be logged in to delete a robot.");
        return;
      }
      const meEmail = norm(currentUser.email);
      const robotRef = doc(db, "robots", robotId);
      const robotSnap = await getDoc(robotRef);
      if(!robotSnap.exists()){
        //robot already deleted or does not exist
        await updateDoc(doc(db, "profiles", currentUser.email), {
          robots: arrayRemove(robotId),
        });
        setRobots((prev) => prev.filter((r) => r.id !== robotId));
        return;
      }
      const robot = robotSnap.data();
      const isAdmin = norm(robot.admin) === meEmail;
      if(!isAdmin){
        //not admin, just remove from profile
        const myProfiuleRef = doc(db, "profiles", currentUser.email);
        await Promise.all([
          updateDoc(myProfiuleRef, { robots: arrayRemove(robotId) }),
          updateDoc(robotRef, { users: arrayRemove(currentUser.email) }),
        ]);
        setRobots((prev) => prev.filter((r) => r.id !== robotId));
        return;
      }
      //admin, delete robot and remove from all profiles
      const ok = window.confirm("Delete this robot for all users? This cannot be undone.");
      if(!ok) return;

      //finding all profiles with this robot
      const qprofiles = query(
        collection(db, "profiles"),
        where("robots", "array-contains", robotId)
      );
      const profSnap = await getDocs(qprofiles);
      const batch = writeBatch(db);

      //remove robotid from each profile
      profSnap.forEach((p) => {
        batch.update(p.ref, { robots: arrayRemove(robotId) });
      });
      //delete the robot
      batch.delete(robotRef);
      await batch.commit();
      setRobots((prev) => prev.filter((r) => r.id !== robotId));
    }catch (error) {
      console.error("Error deleting robot:", error);
      alert("Failed to delete robot. See console for details.");
    }
  };
  


  const [robots, setRobots] = useState([]);
  const { currentUser } = useAuth(); 
  useEffect(() => {
    if(!currentUser){
          console.error("No current user found.");
          return;
        }
    const fetchRobots = async() => {
      try{

        // getting the user profiles
        const profilesRef = doc(db, "profiles", norm(currentUser.email));
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
        <><RobotCard
          key={robot.id}
          imgSrc="https://images.squarespace-cdn.com/content/v1/5a3c1a29f9a61e2987882112/bee5c58a-5b2c-4302-bb18-433dd7bd5f2c/ROSmaster.jpeg"
          imgAlt={`Robot ${robot.id}`}
          title={robot.name || `Robot ${index + 1}`}
          description={`IP: ${robot.ipAddress}`}
          buttonText="FPV/Control"
          //link={`/ControlPage/${robot.id}`} /><Button
          link={`/RobotDashboardPage/${robot.id}`} /><Button
            variant="outlined"
            size="small"
            sx={{ mt: 1, textTransform: "none" }}
            onClick={() => handleDeleteRobot(robot.id)}
          >
            Delete Robot
          </Button></>
      ))
    )}
  </div>
);
};

export default HomePage;
