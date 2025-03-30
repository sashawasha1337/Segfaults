import React from "react";
import { useNavigate } from "react-router-dom";

import IconButton from "@mui/material/IconButton";
import ArrowBackIcon from "@mui/icons-material/ArrowBack";


const BackButton = ({path="/"}) => { // defaults to "/" page if no url specified
  const navigate = useNavigate();
    
  return (
    <IconButton
      onClick={() => navigate(path)} // passes path to navigate function
      sx={{
        position: "absolute",
        top: 50,
        right: 75,
        color: "black",
        padding: "40"
      }}
    >
      <ArrowBackIcon fontSize="large"/>
    </IconButton>
  );
};

export default BackButton;