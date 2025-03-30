import React from "react";
import { useNavigate, useParams } from "react-router-dom";

import { Alert, Box, Button, TextField,Dialog, DialogActions,DialogTitle,DialogContent } from "@mui/material";

const UserSettingsPage = () => {
    const navigate = useNavigate();
    const [oldPassword, setOldPassword] = React.useState("");   
    const [newPassword, setNewPassword] = React.useState("");
    const [repeatPassword, setRepeatPassword] = React.useState("");
    const {username}=useParams();

    const[openDialog, setOpenDialog] = React.useState(false);



   
    React.useEffect(() => {
        if(!username)
            navigate("/"); 
        //we can add secondary verification here to check if the username exists in the database
    }, [username, navigate]);

   

    const handleClickLogout = () => {
        // logic to handle user logout should go here
        setOpenDialog(true); 
    };
    const handleConfirmedLogout = () => {

        // logic to handle removing user tracking for logout should go here
        setOpenDialog(false);
        navigate("/"); 
    }
    const handleCancelLogout = () => {
        setOpenDialog(false);
    }
const handleUpdatePassword= () => {
    if (newPassword !=repeatPassword){
        //add logic to check if old password is correct here
        alert("Passwords do not match!");
        return;
    }
    // logic to handle successful password update should go here
    console.log("Password updated successfully!");
};

return(
 
    <Box // component container
        component="form"
        sx={{ 
            "& .MuiTextField-root": { mt: 2, width: "40ch" },
            display: "flex",
            flexDirection: "column",
            alignItems: "center",
            position: "relative"
        }}
        noValidate
        autoComplete="off"
    >
    <h1>{username} Settings</h1>

    {/* Password update text fields */}
    <TextField
        label="Old Password"
        variant="filled"
        type="password"
        required
        onChange={(e) => setOldPassword(e.target.value)}
    />
    <TextField
        label="New Password"
        variant="filled"
        type="password"
        required
        onChange={(e) => setNewPassword(e.target.value)}
    />
      
    <TextField
        label="Repeat New Password"
        variant="filled"
        type="password"
        required
        onChange={(e) => setRepeatPassword(e.target.value)}
    />

    <Button onClick={handleUpdatePassword} variant="contained" sx={{ mt: 2 }}>
        Update Password
    </Button>

    <Button onClick={handleClickLogout} variant="contained" sx={{ mt: 2, backgroundColor:"red" }}>
        Logout
    </Button>
    <Dialog open={openDialog} onClose={handleCancelLogout}>
            <DialogTitle>Are you sure?</DialogTitle>
            <DialogContent>
                <p>You will be returned to the login screen</p>
            </DialogContent>
            <DialogActions>
                <Button onClick={handleConfirmedLogout} color="primary" variant="contained">
                    Logout
                </Button>
                <Button onClick={handleCancelLogout} color="secondary" variant="outlined">
                    Cancel
                </Button>
            </DialogActions>
        

    </Dialog>

    </Box>
  );
  





};
export default UserSettingsPage;