import React from 'react';
import { Typography, Box } from '@mui/material';
import CodeBlock from '../../components/CodeBlock';

export default function GettingStarted() {
  return (
    <Box sx={{ ml: '240px', p: 3}}>
      <Typography variant="h4" gutterBottom>Getting Started</Typography>
      <Typography variant="body1" paragraph>
        Welcome to the documentation! Here’s how to get your robot running:
      </Typography>
      <ol>
        <li>list hardware and software dependencies 
        </li>
      </ol>
    </Box>
  );
}
