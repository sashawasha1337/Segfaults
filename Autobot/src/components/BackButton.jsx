import React from "react";
import IconButton from "@mui/material/IconButton";
import ArrowBackIcon from "@mui/icons-material/ArrowBack";
import { useNavigate } from "react-router-dom";


const BackButton = ({backURL="/"}) => {//defaults to "/" page if no url specified
    const navigate = useNavigate();
    
    return (
        <IconButton
        onClick={() => navigate(backURL)}//passes backURL to navigate function
        sx={{
            position: "absolute",
            top: 50,
            right: 75,
            color: "black",
            padding: "40",
        }}
        >
        <ArrowBackIcon fontSize="large"/>
        </IconButton>
    );
    }

export default BackButton;