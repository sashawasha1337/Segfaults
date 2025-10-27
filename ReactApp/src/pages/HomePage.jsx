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
  const [allRobotIds, setAllRobotIds] = useState([]);
  const [page, setPage] = useState(1);
  const pageSize = 6;

  const loadPage = async (robotIds, pageNum) => {
    const startIndex = (pageNum - 1) * pageSize;
    const endIndex = startIndex + pageSize;
    const pageIds = robotIds.slice(startIndex, endIndex);

    const robotsData = [];
    for (const id of pageIds) {
      const robotRef = doc(db, "robots", id);
      const robotSnap = await getDoc(robotRef);
      if (robotSnap.exists()) {
        robotsData.push({ id: robotSnap.id, ...robotSnap.data() });
      }
    }

    setRobots(robotsData);
    setPage(pageNum);
  };

  useEffect(() => {
    if (!currentUser) {
      console.error("No current user found.");
      return;
    }

    const fetchProfileAndRobots = async () => {
      try {
        const profilesRef = doc(db, "profiles", norm(currentUser.email));
        const profileSnap = await getDoc(profilesRef);

        if (!profileSnap.exists()) {
          console.error("Profile not found for user:", currentUser.email);
          return;
        }

        const profileData = profileSnap.data();
        const robotIds = profileData.robots || [];
        setAllRobotIds(robotIds);

        // Automatically load the first page
        loadPage(robotIds, 1);
      } catch (error) {
        console.error("Error fetching robots:", error);
      }
    };

    fetchProfileAndRobots();
  }, [currentUser]);


  return (
    <>
      <Box className="homepage"
        sx={{
          width: "100%", 
          minHeight: "100vh",
          display: "flex",
          flexDirection: "row",
        }}
      >
            {/* --- Top buttons --- */}
      <Box>
        <SettingsButton path="/UserSettingsPage" />
        <AddButton path="/AddRobotPage" />

        <Button
          variant="contained"
          onClick={() => navigate("/ActivityLogPage")}
          sx={{
            position: "absolute",
            top: 100,
            right: 35,
            textTransform: "none",
            borderRadius: "10px",
            width: "125px",
            height: "50px",
            fontSize: "1rem",
            backgroundColor: "black",
          }}
        >
          Activity Log
        </Button>
      </Box>
          {/* --- Robot list --- */}
        <div> {/* div need around box or else previous and next buttons are stretched on page with 1 robot */}
          <Box
            sx={{
              display: "flex",
              flexWrap: "wrap",
              gap: 3,           // spacing between cards
              justifyContent: "center",
              paddingTop: 4,
              paddingX: 2,
            }}
          >
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
                  admin={robot.admin}
                  buttonText="FPV/Control"
                  link={`/RobotDashboardPage/${robot.id}`}
                  onDelete={() => handleDeleteRobot(robot.id)}
                />
              ))
            )}
          </Box>
          {allRobotIds.length > pageSize && (
            <Box
              sx={{
                display: "flex",
                justifyContent: "center",
                marginTop: 6,
                marginBottom: 6,
                width: "100%",
              }}
            >
              <Button
                variant="outlined"
                disabled={page === 1}
                onClick={() => loadPage(allRobotIds, page - 1)}
                sx={{
                  marginRight: 2 ,
                  minWidth: 120,
                }}
              >
                Previous
              </Button>
              <Button
                variant="outlined"
                disabled={page * pageSize >= allRobotIds.length}
                onClick={() => loadPage(allRobotIds, page + 1)}
                sx={{
                  minWidth: 120,
                }}
              >
                Next
              </Button>
            </Box>
          )}
        </div>
      </Box>
      
    </>
  );
}

export default HomePage;
