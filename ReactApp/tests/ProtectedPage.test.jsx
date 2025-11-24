import {render, screen} from '@testing-library/react';

import {vi} from 'vitest';
//fake firebase calls

const fakeAuth ={}
vi.mock("firebase/auth", () => ({
    getAuth: vi.fn(() => fakeAuth),
    onAuthStateChanged: vi.fn(),
}));


import ProtectedPage from '../src/components/ProtectedPage.jsx';
import { MemoryRouter, Routes, Route } from 'react-router-dom';




import { getAuth, onAuthStateChanged } from "firebase/auth";


//rendering this is done differently than in the app
//react router mounts EVERY possible route whenever navigation happens 
//can cause infinite loops
function renderProtected(initialPath= "/HomePage"){
    return render(
        <MemoryRouter initialEntries={[initialPath]}>
            <Routes>
                <Route path="/LoginPage" element={<div>LoginPage</div>} />
                <Route element={<ProtectedPage />} >
                    <Route path="/HomePage" element={<div>HomePage</div>} />
                </Route>

                
            </Routes>
        </MemoryRouter>
    );
}
describe("ProtectedPage", () => {

    beforeEach(() => {
        onAuthStateChanged.mockImplementation((auth,cb)=>{
            cb(null);
            return () => {};
        });
    });

    test("redirects to LoginPage if not authenticated", async () => {
            renderProtected(); 
            expect(await screen.findByText("LoginPage")).toBeInTheDocument();  
    });
    test("renders protected content if authenticated", async () => {
        onAuthStateChanged.mockImplementation((auth, callback) => {
            callback({ uid: "user123" });
            return () => {};
        });

        renderProtected();
        expect(await screen.findByText("HomePage")).toBeInTheDocument();
    });
});