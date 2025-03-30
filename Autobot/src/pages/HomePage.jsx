import React from 'react';
import "../styles/homepage.css";
import {RobotCard} from "../components/RobotCard";
import AddButton from '../components/AddButton';
import SettingsButton from '../components/SettingsButton';

function HomePage() {
  return (
    <div className = "homepage">
     <SettingsButton path="/" />
     <AddButton path="/AddRobots" />
     <RobotCard
     imgSrc="https://images.squarespace-cdn.com/content/v1/5a3c1a29f9a61e2987882112/bee5c58a-5b2c-4302-bb18-433dd7bd5f2c/ROSmaster.jpeg"
     imgAlt="Robot"
     title="Robot 1"
     description="Segfaults UGV"
     buttonText="Activity Log"
     link="/activitylog"
     />
     <RobotCard
     imgSrc="https://images.squarespace-cdn.com/content/v1/5a3c1a29f9a61e2987882112/bee5c58a-5b2c-4302-bb18-433dd7bd5f2c/ROSmaster.jpeg"
     imgAlt="Robot"
     title="Robot 2"
     description="Segfaults UGV"
     buttonText="Activity Log"
     link="/activitylog"
     />
     
    </div>
    
    
  );
}

export default HomePage;