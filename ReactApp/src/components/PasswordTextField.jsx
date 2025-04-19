import React from "react";
import { useNavigate } from "react-router-dom";

import { FilledInput, FormControl, IconButton, InputAdornment, InputLabel } from "@mui/material";
import { Visibility, VisibilityOff } from "@mui/icons-material";

{/* 
  PasswordTextField is a template for a text field built for entering passwords. When entering
  characters into the textfield, the characters will be obfuscated. Clicking the eye icon will
  toggle the password obfuscation. There are four optional props m, mt, width, and label which
  allows various attributes of PasswordTextField to be altered.
*/}
const PasswordTextField = ({ m=0, mt=0, width="30ch", label="Password", ...props}) => {
    const navigate = useNavigate();

    const [showPassword, setShowPassword] = React.useState(false);
    
    const handleClickShowPassword = () => setShowPassword((show) => !show);

    const handleMouseDownPassword = (event) => {
    event.preventDefault();
    };

    const handleMouseUpPassword = (event) => {
    event.preventDefault();
    };

    return (
      <FormControl sx={{ m: m, mt: mt, width: width }} variant="filled">
        <InputLabel htmlFor="password-text-field">{label}</InputLabel>
        <FilledInput
          id="password-text-field"
          type={showPassword ? 'text' : 'password'} // toggles how characters are displayed
          endAdornment={
            <InputAdornment position="end">
              <IconButton
                aria-label={
                showPassword ? 'hide password' : 'display password'
                }
                onClick={handleClickShowPassword} // toggles showPassword on click
                onMouseDown={handleMouseDownPassword}
                onMouseUp={handleMouseUpPassword}
                edge="end"
              >
                {showPassword ? <VisibilityOff /> : <Visibility />} {/* toggles show/hide password icon */}
              </IconButton>
            </InputAdornment>
          }
          label="Password"
          {...props}
        />
      </FormControl>
    );
  };
  
  export default PasswordTextField;