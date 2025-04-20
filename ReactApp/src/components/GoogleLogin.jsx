import React, { useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { auth } from "../firebaseConfig.js";
import { GoogleAuthProvider, signInWithRedirect, getRedirectResult, signInWithPopup } from "firebase/auth";
import { Button } from "@mui/material";
import GoogleIcon from "@mui/icons-material/Google";
import { provider } from "../firebaseConfig.js";

export function GoogleLogin() {
  const navigate = useNavigate();
  const [errorCode, setErrorCode] = useState('');
  const [errorMessage, setErrorMessage] = useState('');


  async function handleGoogleLogin() {
    signInWithPopup(auth, provider)
    .then((result) => {
        // This gives you a Google Access Token. You can use it to access the Google API.
        const credential = GoogleAuthProvider.credentialFromResult(result);
        const token = credential.accessToken;
        // The signed-in user info.
        const user = result.user;
        // IdP data available using getAdditionalUserInfo(result)
        navigate("/HomePage");
        // ...
    }).catch((error) => {
        // Handle Errors here.
        const errorCode = error.code;
        const errorMessage = error.message;
        // The email of the user's account used.
        const email = error.customData.email;
        // The AuthCredential type that was used.
        const credential = GoogleAuthProvider.credentialFromError(error);
        // ...
    });
  }

  return (
    <Button
      variant="contained"
      startIcon={<GoogleIcon />}
      onClick={handleGoogleLogin}
      sx={{
        mt: 3,
        textTransform: "none",
        width: "305px",
        height: "50px",
        fontSize: "1.0rem"
      }}
    >
      Sign in with Google
    </Button>
  );
}