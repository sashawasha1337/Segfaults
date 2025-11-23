import {render, screen} from '@testing-library/react';
import ProtectedPage from '../components/ProtectedPage.jsx';
import { MemoryRouter, Routes, Route } from 'react-router-dom';
import {vi} from 'vitest';

//fake firebase calls
vi.mock("firebase/auth", () => ({
    getAuth: vi.fn(() => ({})),
    onAuthStateChanged: vi.fn(),
}));


function renderProtected(initialPath= "/HomePage"){
    return render(
        <MemoryRouter initialEntries={[initialPath]}>
            <Routes>
                <Route element={<ProtectedPage />} >
                    <Route path="/HomePage" element={<div>HomePage</div>} />
                </Route>

                <Route path="/LoginPage" element={<div>LoginPage</div>} />
            </Routes>
        </MemoryRouter>
    );
}
describe("ProtectedPage", () => {

    beforeEach(() => {
        onAuthStateChanged.mockReset();
    });

    test("redirects to LoginPage if not authenticated", async () => {
        onAuthStateChanged.mockImplementation((auth, callback) => {
            callback(null); 
            return () => {};
        });

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