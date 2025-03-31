import React, { useState, useEffect } from 'react';
import { useNavigate } from "react-router-dom";
import '../styles/SignUpPage.css';
import { validatePassword,createUserWithEmailAndPassword,  getRedirectResult, GoogleAuthProvider } from "firebase/auth";
import { auth } from "../firebaseConfig.js";

import { Button, FilledInput, FormControl, FormHelperText, InputLabel } from "@mui/material";
import LockIcon from '@mui/icons-material/Lock';
import BackButton from "../components/BackButton";
import PasswordTextField from '../components/PasswordTextField';



function SignUpPage() {


  const navigate = useNavigate();

  const [email, setEmail] = useState('');
  const [emailError, setEmailError] = useState('');
  const [passwordError, setPasswordError] = useState('');
  const [password, setPassword] = useState('');
  const [repassword, setRepassword] = useState('');
  const [signinFail, setSigninFail] = useState(false);
  const [signin, setSignin] = useState(false);
  

  useEffect(() => {
    if (password && repassword && password !== repassword) {
      setPasswordError("Passwords do not match");
    } else {
      setPasswordError('');
    }
  }, [password, repassword]);


  async function handleSignUp() {
    const emailRegex = /^[a-zA-Z0-9._-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,6}$/;
    setEmailError('');
    setPasswordError('');

    if (password !== repassword) {
      setPasswordError("Passwords do not match");
    }
    
    if (!emailRegex.test(email)) {
      setEmailError("Please enter a valid email address");
    }

    if (password !== repassword || !emailRegex.test(email)){ return; }

    const status = await validatePassword( auth, password );
    if (!status.isValid) {
      setPasswordError("Lowercase character required\n" +
                        "uppercase character required\n"+
                        "Numeric character required\n"+
                        "Non-alphanumeric character required\n"+
                        "The following characters satisfy the non-alphanumeric character requirement:\n"+
                        " ^ $ * . [ ] { } ( ) ? &quot ! @ # % & / \ , > < ' : ; | _ ~\n"+
                        "Minimum password length (ranges from 6 to 30 characters; defaults to 6)");
      return;
    }
    console.log("Signing in with:", email, password);
    createUserWithEmailAndPassword( auth, email, password)
    .then((userCredential) => {
      // user signed in
      const user = userCredential.user;
      navigate("/HomePage");
    }).catch((error) => {
      // Handle Errors here.
      const errorCode = error.code;
      const errorMessage = error.message;
      // ...
    });
  
  };
      
  function handlePassword() {
    console.log("Handling password reset");
  };

  return (
    <>
      <BackButton path='/' />

      <div className="background">
        <div className="container">

          <div className="input">
            <FormControl sx={{ m: 1, width: '25ch' }} variant="filled">
              <InputLabel htmlFor="filled-adornment-username"><LockIcon />Email</InputLabel>
              <FilledInput
                onChange={e => setEmail(e.target.value)}
                id="filled-adornment-username"
                type='text'
                label="Username"
              />
              <FormHelperText id="component-helper-text">
                {/* Display error message if any */}
                {emailError && (
                <FormHelperText error>{emailError}</FormHelperText>
                )}
              </FormHelperText>
            </FormControl>
          </div>

          <div className="input">
            <FormControl sx={{ m: 1, width: '25ch' }} variant="filled">
              <InputLabel htmlFor="filled-adornment-username">Password</InputLabel>
              <FilledInput
                id="confirm-password"
                type="password"
                onChange={e => setPassword(e.target.value)}
              />
            </FormControl>
          </div>

          <div className="input">
            <FormControl sx={{ m: 1, width: '25ch' }} variant="filled">
              <InputLabel htmlFor="filled-adornment-username">Password</InputLabel>
              <FilledInput
                id="confirm-password"
                type="password"
                onChange={e => setRepassword(e.target.value)}
              />
              {/* Display error message if any */}
              {passwordError && (
                <FormHelperText error>{passwordError}</FormHelperText>
              )}
            </FormControl>
          </div>
          
          <div className="LogButton">
            <div className="submit-container">
              <div className="Submit">
                <Button variant="contained"
                  className="Submit-Button"
                  onClick={handleSignUp}
                >
                  Sign Up
                </Button>
              </div>
            </div>
          </div>
            

        </div>
      </div>
    </>
  );
};

export default SignUpPage;