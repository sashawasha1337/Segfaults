import React from "react";
import { Box, Button, Container, TextField, Typography } from "@mui/material";
import BackButton from "../components/BackButton";
import { useNavigate } from "react-router-dom";

{/*
  The forgot username page allows users to get sent an email that contains
  the username associated with the email in the case that the user's
  username was forgotten.
*/}
function ForgotUsernamePage() {
  const navigate = useNavigate();

  const [hasError, setHasError] = React.useState(false);

  {/*
    logic to check if email is in database should go here
    code currently just toggles error on and off on click
  */}
  const handleHasError = () => setHasError((error) => !error);

  return (
    <Box // container for entire window
      sx={{
        display: "flex",
        justifyContent: "center",
        flexDirection: "column",
        alignItems: "center",
        width: "100%",
        height: "100%"
      }}
    >

      <BackButton path="/"/>

      {/* Text header */}
      <Typography variant="h5" sx={{ fontWeight: "bold"}}>
          Enter your email to find your username
      </Typography>

      <Container // container for inner components
        component="form"
        sx={{
          mt: 5,
          display: "flex",
          flexDirection: "column",
          alignItems: "center",
          position: "relative",
          border: "2px solid #157ed4",
          borderRadius: "10px",
          padding: "30px"
        }}
        noValidate
        autoComplete="off"
      >

        {/* Email text field */}
        <TextField
          label="Email"
          variant="filled"
          error={hasError} // toggles textfield error styling
          helperText={hasError ? "No account found with the provided email" : ""}
          sx={{ width: "300px" }}
        />

        {/* Reset password button */}
        <Button variant="contained"
          onClick={() => {handleHasError()}} // checks validity of provided email
          sx={{
            mt: 5,
            textTransform: "none",
            borderRadius: "50px",
            width: "175px",
            height: "50px",
            fontSize: "1.0rem",
            backgroundColor: "purple",
          }}
        >
          Find username
        </Button>

      </Container>
    </Box>
  );
};

export default ForgotUsernamePage;