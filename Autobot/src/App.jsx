import { useState } from 'react'
import reactLogo from './assets/react.svg'
import viteLogo from '/vite.svg'
import { useNavigate } from "react-router-dom";//probably need this
import { Routes, Route } from 'react-router-dom';        
import AddRobots from "./pages/AddRobots";  
import Trash_View_Page from './pages/Trash_View_Page';  
import ActivityLog from "./pages/ActivityLog";
import LoginPage from "./pages/LoginPage";
import NewAccount from "./pages/NewAccount/NewAccount.jsx";
import HomePage from './pages/HomePage';
import RobotSettings from './pages/RobotSettings';
import './App.css'

function App() {
  const [count, setCount] = useState(0)

  return (
    <>
    <Routes>
      <Route path="/" element={<LoginPage />} />
      <Route path="/AddRobots" element={<AddRobots />} />
      <Route path="/Trash_View_Page" element={<Trash_View_Page />} />
      <Route path="/ActivityLog" element={<ActivityLog />} />
      <Route path="/HomePage" element={<HomePage />} />
      <Route path="/LoginPage" element={<LoginPage />} />
      <Route path="/NewAccount" element={<NewAccount />} />
      <Route path="/RobotSettings" element={<RobotSettings />} />
    </Routes>
  </>
);
}

export default App
