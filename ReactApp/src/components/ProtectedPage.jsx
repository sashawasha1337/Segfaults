import {Navigate, Outlet} from 'react-router-dom';
import { getAuth, onAuthStateChanged} from "firebase/auth";
import { useState, useEffect } from "react";
import { CircularProgress, Box } from '@mui/material';

function ProtectedPage() {
    // Check if the user is authenticated
    const auth = getAuth();
    const [user, setUser] = useState();
    const [loading, setLoading] = useState(true); // Show a loading spinner while checking auth to prevent inadvertent brief access


    useEffect(() => {
        // Listen for authentication changes
        const unsubscribe = onAuthStateChanged(auth, (user) => {
            setUser(user);
            setLoading(false);
        });

        // Unsubscribe to the listener when the component unmounts
        return () => unsubscribe();

    }, [auth]);

    if (loading) {
        return (
            <Box sx={{ display: "flex", justifyContent: "center", alignItems: "center", height: "100vh" }}>
                <CircularProgress />
            </Box>
        );
    }

    // If the user is not authenticated, redirect to the login page
    if (!user) {
        console.log("User not logged in. Redirecting to login page.");
        return <Navigate to="/LoginPage" />;
    }

    // If the user is authenticated, render the protected page
    return (
        <Outlet />
    );
}

export default ProtectedPage;