import {getAuth, onAuthStateChanged} from 'firebase/auth';
import React, {createContext, useContext, useEffect, useState} from 'react';

const AuthContext = createContext();
//this will wrap the entire app in the auth provider
//all components should definitely be able to see the authorization going on
export const AuthProvider = ({children}) => {   
    const [currentUser, setCurrentUser] = useState(null);
    const [loading, setLoading] = useState(true);
    const auth = getAuth();
    
    useEffect(() => {
        const unsubscribe = onAuthStateChanged(auth, (user) => {
        setCurrentUser(user);
        setLoading(false);
        });
        return () => unsubscribe();
    }, [auth]);
    
    return (
        <AuthContext.Provider value={{currentUser}}>
        {!loading && children}
        </AuthContext.Provider>
    );
}
// this is a 2nd export which is not the whole wrapper
//instead just a way to access the authenticated user
export const useAuth = () => useContext(AuthContext);