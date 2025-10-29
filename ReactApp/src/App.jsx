import { Routes, Route } from 'react-router-dom';
import ActivityLogPage from "./pages/ActivityLogPage.jsx"; 
import AddRobotPage from "./pages/AddRobotPage.jsx";
import ControlPage from './pages/ControlPage';
import ForgotUsernamePage from './pages/ForgotUsernamePage.jsx';
import HomePage from './pages/HomePage';
import LoginPage from "./pages/LoginPage";
import ResetPasswordPage from './pages/ResetPasswordPage.jsx';
import RobotSettingPage from './pages/RobotSettingPage.jsx';
import MapViewPage from './pages/MapViewPage';
import SignUpPage from "./pages/SignUpPage.jsx";
import TrashViewPage from './pages/TrashViewPage.jsx';
import UserSettingsPage from './pages/UserSettingsPage.jsx';
import RobotDashboardPage from './pages/RobotDashboardPage.jsx';
import ProtectedPage from './components/ProtectedPage.jsx';
import DocumentationLayout from './pages/Documentation/DocumentationLayout';
import GettingStarted from './pages/Documentation/GettingStarted';
import Configuration from './pages/Documentation/Configuration';
import Troubleshooting from './pages/Documentation/Troubleshooting';
import RobotID from './pages/Documentation/RobotID';
import './App.css'


function App() {
  return (
    <Routes>
      <Route path="/" element={<LoginPage />} />
      <Route path="/docs" element={<DocumentationLayout />}>
          <Route path="getting-started" element={<GettingStarted />} />
          <Route path="configuration" element={<Configuration />} />
          <Route path="troubleshooting" element={<Troubleshooting />} />
          <Route path="robot-id" element={<RobotID />} />
      </Route>
      <Route path="/LoginPage" element={<LoginPage />} />
      <Route element={<ProtectedPage />} >
        <Route path="/ActivityLogPage" element={<ActivityLogPage />} />
        <Route path="/AddRobotPage" element={<AddRobotPage />} />
        <Route path="/ControlPage/:robotID" element={<ControlPage />} />
        <Route path="/ForgotUsernamePage" element={<ForgotUsernamePage />} />
        <Route path="/HomePage" element={<HomePage />} />
        <Route path="/ResetPasswordPage" element={<ResetPasswordPage />} />
        <Route path="/RobotSettingPage/:robotID" element={<RobotSettingPage />} />
        <Route path="/MapViewPage" element={<MapViewPage />} />
        <Route path="/SignUpPage" element={<SignUpPage />} />
        <Route path="/TrashViewPage" element={<TrashViewPage />} />
        <Route path="/UserSettingsPage" element={<UserSettingsPage />} />
        <Route path="/RobotDashboardPage/:robotID" element={<RobotDashboardPage />} /> 
      </Route>
    </Routes>
  );
}

export default App
