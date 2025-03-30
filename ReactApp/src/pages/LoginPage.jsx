import React from "react";
import { useNavigate } from "react-router-dom";

import { Alert, Box, Button, TextField } from "@mui/material";
import GoogleIcon from "@mui/icons-material/Google";
import PasswordTextField from "../components/PasswordTextField";

{/*
  The Login page allows users to enter their username and password to gain access to the web application.
  In addition, this page links to pages that allows the user to create a new account, sign in with a Google
  account, recover a username through a registered email, and reset an account's password through a
  registered email.
*/}
function LoginPage() {
  const navigate = useNavigate();

  const [invalidCredentials, setInvalidCredentials] = React.useState(false);

  let count = 0; // delete this after credential verification logic is implemented

  const handleClickLogin = () => {
    // logic to handle credential verification should go here
    switch (count++) { // code is currently set up to test the error message for invalid credential input
      case 0:
        // case where credentials are invalid, invalidCredentials should be set to true
        setInvalidCredentials(true);
        break;
      case 1:
        // case where credentials are valid, invalidCredentials should be set to false and then navigate to homepage
        setInvalidCredentials(false);
        navigate("/HomePage");
        break;
    }
  };

  return (
    <Box // component container
      component="form"
      sx={{ 
        "& .MuiTextField-root": { mt: 15, width: "40ch" },
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        position: "relative"
      }}
      noValidate
      autoComplete="off"
    >
    
      {/* Username text field */}
      <TextField
        label="Username"
        variant="filled"
      />

      <PasswordTextField mt={3} width="40ch" />
       
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
        Invalid username and password combination.
      </Alert>

      {/* Forgot username button */}
      <Button variant="text" disableRipple
        onClick={() => navigate("/ForgotUsernamePage")}
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
        <span>Forgot Username?</span>
      </Button>

      {/* Log in buttonn */}
      <Button variant="contained"
        onClick={() => {handleClickLogin()}} // handles credential verification and navigation to homepage
        sx={{
          mt: 7,
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
          mt: 5,
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
      <Button variant="contained" startIcon={<GoogleIcon />}
        onClick={() => navigate("/")} // path to Google sign in page needs to be added
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
        <span>Reset Password</span>
      </Button>
    </Box>
  );
};

export default LoginPage;