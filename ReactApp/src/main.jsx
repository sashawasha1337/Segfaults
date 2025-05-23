import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import { BrowserRouter as Router } from "react-router-dom";
import App from './App.jsx'
import './index.css'
import { AuthProvider } from './ContextForAuth.jsx';
import { useAuth } from "./ContextForAuth.jsx";
//refer to contextforauth.jsx for the authprovider and useauth function
createRoot(document.getElementById('root')).render(
  <StrictMode>
    <AuthProvider>
      <Router>
        <App />
      </Router>
    </AuthProvider>
  </StrictMode>
)