import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import { signOut } from "firebase/auth";
import { auth } from "../firebaseConfig.js";
import { Button, Dialog, DialogActions, DialogTitle, DialogContent } from "@mui/material";

export default function LogoutButton() {
    const navigate = useNavigate();
    const [openDialog, setOpenDialog] = useState(false);

    const handleClickLogout = () => {
        // open confirmation dialog
        setOpenDialog(true);
    };

    const handleConfirmedLogout = () => {
        // sign out the user and navigate to login
        signOut(auth)
            .then(() => {
                navigate("/");
            })
            .catch((error) => {
                alert("logout Fail");
            });
        setOpenDialog(false);
    };

    const handleCancelLogout = () => {
        setOpenDialog(false);
    };

    return (
        <>
            <Button onClick={handleClickLogout} variant="contained" sx={{ mt: 2, backgroundColor: "red" }}>
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
        </>
    );
}