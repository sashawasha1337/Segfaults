import React from 'react';
import { Typography, Box } from '@mui/material';
import CodeBlock from '../../components/CodeBlock';

export default function RobotID() {
  return (
    <Box sx={{ ml: '240px', p: 1}}>
        <Typography variant="h4" gutterBottom>Robot ID</Typography>
        <Typography variant="body1" paragraph>
            Here’s how to set up your robot ID to link to the database:
        </Typography>
        <Typography variant="body1" paragraph sx={{ p: 3, textAlign: 'left', }}>
            Requirements are:
            <Typography style={{ p:0, marginLeft: '1.5rem' }}>
                - Ubuntu system (with ROS2 installed)
            </Typography>
        </Typography>
        <Box sx={{ textAlign: 'left'}}>
            <ol>

                <li>Start your Ubuntu system, and navigate to the terminal or use ctrl+shift+t</li>
                <li> 
                    In the terminal run the follow command with your robot ID<CodeBlock code={'echo \'export ROBOT_UID="Your_RobotID"\' >> ~/.bashrc'} />.
                </li>
                <li> Then close the terminal.</li>
            </ol>
        </Box>
    </Box>
  );
}
