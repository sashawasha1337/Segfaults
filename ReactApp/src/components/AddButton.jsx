import React from "react";
import { useNavigate } from "react-router-dom";

import IconButton from "@mui/material/IconButton";
import AddCircleOutlineOutlinedIcon from '@mui/icons-material/AddCircleOutlineOutlined';

{/* 
  AddButton is a template for a plus surrounded by a circle icon button. This will
  typically be used to send users to an add page. AddButton passes path as a prop
  that represents the path of the page the button will take the user to.
*/}
const AddButton = ({path="/"}) => { //defaults to "/" page if no url specified
  const navigate = useNavigate();

  return (
    <IconButton
      onClick={() => navigate(path)}
      sx={{
        position: "absolute",
        top: 30,
        right: 30,
        color: "black"
      }}
    >
      <AddCircleOutlineOutlinedIcon fontSize="large"/>
    </IconButton>
  );
};

export default AddButton;