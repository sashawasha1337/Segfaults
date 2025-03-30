import React from "react";
import IconButton from "@mui/material/IconButton";
import SettingsOutlinedIcon from '@mui/icons-material/SettingsOutlined';
import { useNavigate } from "react-router-dom";

{/* 
  Template for a sprocket icon button. This will typically be used to send
  users to a settings page. SettingsButton takes a string argument that
  represents the path of the page the button will take the user to.
*/}
const SettingsButton = ({path="/"}) => { //defaults to "/" page if no url specified
  const navigate = useNavigate();
    
  return (
    <IconButton
      onClick={() => navigate(path)}
      sx={{
        position: "absolute",
        top: 30,
        left: 30,
        color: "black"
      }}
    >
      <SettingsOutlinedIcon fontSize="large"/>
    </IconButton>
  );
}

export default SettingsButton;