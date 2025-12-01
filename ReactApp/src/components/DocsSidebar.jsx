import React from 'react';
import { List, ListItemButton, ListItemText, Box, Divider } from '@mui/material';
import { useNavigate, useLocation } from 'react-router-dom';

// Show only the Robot ID docs entry per request
const navItems = [
  { label: 'Robot ID', path: '/docs/robot-id' },
];

export default function DocsSidebar() {
  const navigate = useNavigate();
  const location = useLocation();

  return (
    <Box sx={{ 
            position: 'fixed',
            left: 0,
            top: 0,
            height: '100vh',            // full viewport height
            width: 240,                 // fixed width
            display: 'flex',
            flexDirection: 'column',
            borderRight: '1px solid #444', 
            bgcolor: 'background.paper',
        }}>
      <List>
        {navItems.map((item) => (
          <ListItemButton
            key={item.path}
            selected={location.pathname === item.path}
            onClick={() => navigate(item.path)}
          >
            <ListItemText primary={item.label} />
          </ListItemButton>
        ))}
      </List>
      <Divider />
    </Box>
  );
}
