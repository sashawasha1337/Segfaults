import React, { useState, useEffect }from "react";
import { useNavigate } from "react-router-dom";
import { auth } from "../firebaseConfig.js";
import { signInWithEmailAndPassword, sendPasswordResetEmail } from "firebase/auth";
import { Alert, Box, Button, TextField } from "@mui/material";
import { Typography } from "@mui/material";
import PasswordTextField from "../components/PasswordTextField";
import { GoogleLogin } from "../components/GoogleLogin.jsx";
import Logo from "../assets/Logo.png";

{/*
  The Login page allows users to enter their username and password to gain access to the web application.
  In addition, this page links to pages that allows the user to create a new account, sign in with a Google
  account, recover a username through a registered email, and reset an account's password through a
  registered email.
*/}
function LoginPage() {
  const navigate = useNavigate();

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');

  const [invalidCredentials, setInvalidCredentials] = React.useState(false);
  
  const [emailError, setEmailError] = useState('');
  const [passwordError, setPasswordError] = useState('');
  const [errorCode, setErrorCode] = useState('');
  const [errorMessage, setErrorMessage] = useState('');

  const emailRegex = /^[a-zA-Z0-9._-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,6}$/;


  const handleForgotPassword = () => {
    sendPasswordResetEmail(auth, email)
    .then(() => {
      // Password reset email sent!
      // ..
    })
    .catch((error) => {
      setErrorCode(error.code);
      setErrorMessage(error.message);
      // ..
    });
    alert("Reset Password Link Sent");
  }

  async function handleClickLogin() {
    setEmailError('');
    setPasswordError('');
    setInvalidCredentials(false);
    if (!emailRegex.test(email)) {
      setEmailError("Please enter a valid email address");
      return;
    }
  
    try {
      const userCredential = await signInWithEmailAndPassword(auth, email, password);
      const user = userCredential.user;
      navigate("/HomePage");
      //console.log(auth.currentUser.email);
    } catch (error) {
      //console.log(error);
      //setErrorCode(error.code);
      //setErrorMessage(error.message);
  
      setInvalidCredentials(true); // Shows generic error
    }
  }
  

  return (
    <Box // whole screen container
      sx={{
        position: "fixed",
        inset: 0,
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        backgroundColor: "#f4ecd8",
        height: "100dvh",
        width: "100vw"
      }}
    >
      <Box // login card
        component="form"
        sx={{ 
          display: "flex",
          flexDirection: "column",
          alignItems: "center",
          backgroundColor: "rgba(255, 255, 255, 0.8)",
          borderRadius: 2,
          p: { xs: 2, sm: 4 },
          boxShadow: 3,
          maxHeight: "100%",
          overflowY: 'auto',
          width: "fit", // responsive width between 320px and 420px
          minHeight: "min(90dvh, 600px)", // never taller than viewport
          justifyContent: "center",
        }}
        noValidate
        autoComplete="off"
      >
        <Typography 
          variant="h2"
          sx={{fontFamily:"Garamond, serif", fontWeight: "bold", mb: 1 , color: "#3b2f2f"}}
        >
          Robohub
        </Typography>
        <img 
          src={Logo} 
          alt="Logo" 
          style={{ width: "75%", marginBottom: "8%" }} 
        />
        {/* Username text field */}
        <TextField
          label="Email"
          variant="outlined"
          onChange={e => setEmail(e.target.value)}
        />
        <Alert icon={false} severity="error" // hides the error alert icon
          sx={{
            mt: 0.25,
            mr: 7,
            backgroundColor: "transparent",
            color: "red",
            padding: 0,
            display: emailError ? "block" : "none" // display component when invalid credentials are entered
          }}
        >
          {emailError}
        </Alert>
  
        <PasswordTextField mt={1} width="40ch" value={password}
          onChange={e => setPassword(e.target.value)}
        />
         
        {/*
          Invalid credentials error message
          This appears below the password textfield when invalid credentials are entered
          and the log in button is clicked.
        */}
        <Alert icon={false} severity="error" // hides the error alert icon
          sx={{
            mt: 0.25,
            mr: 7,
            backgroundColor: "transparent",
            color: "red",
            padding: 0,
            display: invalidCredentials ? "block" : "none" // display component when invalid credentials are entered
          }}
        >
          Invalid email and password combination.
        </Alert>
  
        {/* Forgot username button */}
        <Button variant="text" disableRipple
          onClick={handleForgotPassword}
          sx={{
            mt: invalidCredentials ? 0 : 1.5, // adjusts margin to accommodate the invalid credentials error message
            mr: 27,
            "&:hover": {
              backgroundColor: "transparent"
            },
            textTransform: "none", // allows button text to display lowercase letters
            padding: 0,
            minWidth: "auto",
            fontSize: "0.9rem"
          }}
        >
          <span>Forgot Password?</span>
        </Button>
  
        {/* Log in buttonn */}
        <Button variant="contained"
          onClick={handleClickLogin} // handles credential verification and navigation to homepage
          sx={{
            mt: 0,
            textTransform: "none",
            borderRadius: "50px", // rounds the corners of the button
            width: "125px",
            height: "50px",
            fontSize: "1.0rem",
            backgroundColor: "purple"
          }}
        >
          Log in
        </Button>
  
        {/* Sign up button */}
        <Button variant="contained"
          onClick={() => navigate("/SignUpPage")}
          sx={{
            mt: 1,
            textTransform: "none",
            borderRadius: "50px",
            width: "125px",
            height: "50px",
            fontSize: "1.0rem",
            backgroundColor: "purple"
          }}
        >
          Sign up
        </Button>
  
        {/* Sign in with Google button */}
        <GoogleLogin></GoogleLogin>
  
        {/* Reset Password button */}
        <Button variant="text" disableRipple
          onClick={() => navigate("/ResetPasswordPage")}
          sx={{
            mt: 2,
            "&:hover": {
              backgroundColor: "transparent"
            },
            textTransform: "none",
            padding: 0,
            minWidth: "auto",
            fontSize: "0.8rem"
          }}
        >
        </Button>
      </Box>
    </Box>
  );
};

export default LoginPage;