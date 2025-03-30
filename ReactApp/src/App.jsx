import { Routes, Route } from 'react-router-dom';
import ActivityLogPage from "./pages/ActivityLogPage.jsx"; 
import AddRobotPage from "./pages/AddRobotPage.jsx";
import ControlPage from './pages/ControlPage';
import ForgotUsernamePage from './pages/ForgotUsernamePage.jsx';
import HomePage from './pages/HomePage';
import LoginPage from "./pages/LoginPage";
import ResetPasswordPage from './pages/ResetPasswordPage.jsx';
import RobotSettingPage from './pages/RobotSettingPage.jsx';
import SignUpPage from "./pages/SignUpPage.jsx";
import TrashViewPage from './pages/TrashViewPage.jsx';
import './App.css'

function App() {
  return (
    <Routes>
      <Route path="/" element={<LoginPage />} />
      <Route path="/ActivityLogPage" element={<ActivityLogPage />} />
      <Route path="/AddRobotPage" element={<AddRobotPage />} />
      <Route path="/ControlPage" element={<ControlPage />} />
      <Route path="/ForgotUsernamePage" element={<ForgotUsernamePage />} />
      <Route path="/HomePage" element={<HomePage />} />
      <Route path="/ResetPasswordPage" element={<ResetPasswordPage />} />
      <Route path="/RobotSettingPage" element={<RobotSettingPage />} />
      <Route path="/SignUpPage" element={<SignUpPage />} />
      <Route path="/TrashViewPage" element={<TrashViewPage />} />
    </Routes>
  );
}

export default App
