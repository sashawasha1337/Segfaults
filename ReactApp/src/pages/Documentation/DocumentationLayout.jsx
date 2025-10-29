import React from 'react';
import { Box } from '@mui/material';
import DocsSidebar from '../../components/DocsSidebar';
import { Outlet } from 'react-router-dom';

export default function DocumentationLayout() {
  return (
    <Box sx={{ display: 'flex', height: '100vh', bgcolor: 'background.default', color: 'text.primary' }}>
      <DocsSidebar />
      <Box sx={{ flexGrow: 1, overflowY: 'auto' }}>
        <Outlet />
      </Box>
    </Box>
  );
}
