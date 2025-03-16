import React, { useState } from 'react';
import './NewAccount.css';
import BackButton from "../../components/BackButton";


import EmailIcon from '@mui/icons-material/Email';
import LockIcon from '@mui/icons-material/Lock';

import Box from '@mui/material/Box';
import IconButton from '@mui/material/IconButton';
import Input from '@mui/material/Input';
import FilledInput from '@mui/material/FilledInput';
import OutlinedInput from '@mui/material/OutlinedInput';
import InputLabel from '@mui/material/InputLabel';
import InputAdornment from '@mui/material/InputAdornment';
import FormHelperText from '@mui/material/FormHelperText';
import FormControl from '@mui/material/FormControl';
import TextField from '@mui/material/TextField';
import Visibility from '@mui/icons-material/Visibility';
import VisibilityOff from '@mui/icons-material/VisibilityOff';
import Button from '@mui/material/Button';



function NewAccount() {

    const [email, setEmail] = useState('');
    const [code, setCode] = useState('');
    const [password, setPassword] = useState('');
    const [signinFail, setSigninFail] = useState(false);
    const [showPassword, setShowPassword] = useState(false);

    function handleSignin() {
        console.log("Signing in with:", email, password);
      };
      
    function handlePassword() {
        console.log("Handling password reset");
    };

    const handleClickShowPassword = () => setShowPassword((show) => !show);
    
    const handleMouseDownPassword = (event) => {
        event.preventDefault();
      };
    
      const handleMouseUpPassword = (event) => {
        event.preventDefault();
      };


    return(
        <>
            <BackButton backURL = '/'
            />

            <div className="background">

                <div className="container">
                   
                    <div className="input" >
                            <FormControl sx={{ m: 1, width: '25ch' }} variant="outlined">
                                <InputLabel htmlFor="outlined-adornment-username"><LockIcon />Email</InputLabel>
                                <OutlinedInput
                                    onChange={e => setEmail(e.target.value)}
                                    id="outlined-adornment-username"
                                    type='text'
                                    endAdornment={
                                    <InputAdornment position="end">
                                        <IconButton
                                        onMouseDown={handleMouseDownPassword}
                                        onMouseUp={handleMouseUpPassword}
                                        edge="end"
                                        >
                                        </IconButton>
                                    </InputAdornment>
                                    }
                                    label="Username"
                                />
                            </FormControl>
                    </div>
                    <div>
                        <div className="input">
                            <FormControl sx={{ m: 1, width: '25ch' }} variant="outlined">
                                <InputLabel htmlFor="outlined-adornment-password">Password</InputLabel>
                                <OutlinedInput
                                    onChange={e => setPassword(e.target.value)}
                                    id="outlined-adornment-password"
                                    type={showPassword ? 'text' : 'password'}
                                    endAdornment={
                                    <InputAdornment position="end">
                                        <IconButton
                                        aria-label={
                                            showPassword ? 'hide the password' : 'display the password'
                                        }
                                        onClick={handleClickShowPassword}
                                        onMouseDown={handleMouseDownPassword}
                                        onMouseUp={handleMouseUpPassword}
                                        edge="end"
                                        >
                                        {showPassword ? <VisibilityOff /> : <Visibility />}
                                        </IconButton>
                                    </InputAdornment>
                                    }
                                    label="Password"
                                />
                            </FormControl>
                        </div>
                    </div>
                    <div className="input" >
                            <FormControl sx={{ m: 1, width: '25ch' }} variant="outlined">
                                <InputLabel htmlFor="outlined-adornment-code">Activation Code</InputLabel>
                                <OutlinedInput
                                    onChange={e => setCode(e.target.value)}
                                    id="outlined-adornment-code"
                                    type='text'
                                    endAdornment={
                                    <InputAdornment position="end">
                                        <IconButton
                                        onMouseDown={handleMouseDownPassword}
                                        onMouseUp={handleMouseUpPassword}
                                        edge="end"
                                        >
                                        </IconButton>
                                    </InputAdornment>
                                    }
                                    label="Activation Code"
                                />
                            </FormControl>
                    </div>
                    <div className="LogButton">
                        <div className="submit-container">
                            <div className="Submit"><Button variant="contained" className="Submit-Button" onClick={handleSignin} >Sign Up</Button></div>
                        </div>
                    </div>
                
                </div>
            </div>
        </>
    );

}

export default NewAccount;