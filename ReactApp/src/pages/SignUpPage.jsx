import React, { useState } from 'react';
import { useNavigate } from "react-router-dom";
import '../styles/SignUpPage.css';

import { Button, FilledInput, FormControl, InputLabel } from "@mui/material";
import LockIcon from '@mui/icons-material/Lock';
import BackButton from "../components/BackButton";
import PasswordTextField from '../components/PasswordTextField';


function SignUpPage() {
  const navigate = useNavigate();

  const [email, setEmail] = useState('');
  const [code, setCode] = useState('');
  const [password, setPassword] = useState('');
  const [signinFail, setSigninFail] = useState(false);

  function handleSignin() {
    console.log("Signing in with:", email, password);
    navigate("/Homepage")
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
            </FormControl>
          </div>

          <div className="input">
            <PasswordTextField m={1} width='25ch' />
          </div>

          <div className="input">
            <PasswordTextField m={1} width='25ch' label='Confirm Password' />
          </div>

          <div className="input" >
            <FormControl sx={{ m: 1, width: '25ch' }} variant="filled">
                <InputLabel htmlFor="filled-adornment-code">Activation Code</InputLabel>
                <FilledInput
                  onChange={e => setCode(e.target.value)}
                  id="filled-adornment-code"
                  type='text'
                  label="Activation Code"
                />
            </FormControl>
          </div>
          
          <div className="LogButton">
            <div className="submit-container">
              <div className="Submit">
                <Button variant="contained"
                  className="Submit-Button"
                  onClick={handleSignin}
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