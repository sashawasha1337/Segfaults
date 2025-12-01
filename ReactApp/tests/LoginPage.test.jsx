
import{describe, test ,expect, vi, beforeEach} from 'vitest';
import { render, screen, fireEvent, waitFor} from '@testing-library/react';
import LoginPage from '../src/pages/LoginPage.jsx';


const mockNavigate = vi.fn();

vi.mock('react-router-dom', () => ({
    useNavigate: vi.fn(() => mockNavigate),
}));

const mockSignIn = vi.fn();
const mockReset = vi.fn();

vi.mock('firebase/auth', () => ({
    signInWithEmailAndPassword: vi.fn((auth, email, password) => mockSignIn(email, password)),
    sendPasswordResetEmail: vi.fn((auth, email) => mockReset(email)),
}));
vi.mock("../src/firebaseConfig.js", () => ({
    auth: {},
}));

describe('LoginPage', () => {
    beforeEach(() => {
        vi.clearAllMocks();
    });

    test('renders UI elements', () => {
        render(<LoginPage />);
        expect(screen.getByText(/Robohub/i)).toBeTruthy();
        expect(screen.getByLabelText(/Email/i)).toBeTruthy();
        expect(screen.getByText(/Log in/i)).toBeTruthy();
        expect(screen.getByText(/Sign up/i)).toBeTruthy();
        expect(screen.getByText(/Forgot Password\?/i)).toBeTruthy();
    });

    test('shows email validation error for invalid email', async () => {
        render(<LoginPage />);
        const emailInput = screen.getByLabelText(/Email/i);
        const loginButton = screen.getByText(/Log in/i);

        fireEvent.change(emailInput, { target: { value: 'invalid email' } });
        fireEvent.click(loginButton);

        expect(await screen.findByText(/Please enter a valid email address/i)).toBeTruthy();
        expect(mockSignIn).not.toHaveBeenCalled();
    });

    test('shows invalid credentials message when sign-in fails', async () => {
        mockSignIn.mockRejectedValueOnce(new Error('auth/wrong-password'));
        render(<LoginPage />);
        const emailInput = screen.getByLabelText(/Email/i);
        const passwordInput = screen.getByLabelText(/^Password$/i);
        const loginButton = screen.getByText(/Log in/i);
        fireEvent.change(emailInput, { target: { value: 'test@example.com' } });
        fireEvent.change(passwordInput, { target: { value: 'badpass' } });
        fireEvent.click(loginButton);
        expect(await screen.findByText(/Invalid email and password combination\./i)).toBeTruthy();
        expect(mockSignIn).toHaveBeenCalledWith('test@example.com', 'badpass');
        expect(mockNavigate).not.toHaveBeenCalled();
    });

    test('navigates to HomePage on successful login', async () => {
        mockSignIn.mockResolvedValueOnce({ user: { email: 'test@example.com' } });
        render(<LoginPage />);
        const emailInput = screen.getByLabelText(/Email/i);
        const passwordInput = screen.getByLabelText(/^Password$/i);
        const loginButton = screen.getByText(/Log in/i);
        fireEvent.change(emailInput, { target: { value: 'test@example.com' } });
        fireEvent.change(passwordInput, { target: { value: 'correctpass' } });
        fireEvent.click(loginButton);
        expect(mockSignIn).toHaveBeenCalledWith('test@example.com', 'correctpass');
        await waitFor(() =>
            expect(mockNavigate).toHaveBeenCalledWith('/HomePage')
        );
    });

    test('calls password reset and shows alert when clicking "Forgot Password?"', async () => {
        mockReset.mockResolvedValueOnce();
      
        const alertSpy = vi.spyOn(window, 'alert').mockImplementation(() => {});
        render(<LoginPage />);
        const emailInput = screen.getByLabelText(/Email/i);
        const forgotButton = screen.getByText(/Forgot Password\?/i);

        fireEvent.change(emailInput, { target: { value: 'resetme@example.com' } });
        fireEvent.click(forgotButton);

        expect(mockReset).toHaveBeenCalledWith('resetme@example.com');
        expect(alertSpy).toHaveBeenCalledWith('Reset Password Link Sent');

        alertSpy.mockRestore();
    });
});
