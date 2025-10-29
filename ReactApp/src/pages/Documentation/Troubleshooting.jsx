import React from 'react';
import { Typography, Box } from '@mui/material';

export default function Troubleshooting() {
  return (
    <Box sx={{ ml: '240px', p: 3}}>
      <Typography variant="h4" gutterBottom>Troubleshooting</Typography>
      <Typography variant="body1" paragraph>
        Common issues and how to fix them.
      </Typography>
    </Box>
  );
}
