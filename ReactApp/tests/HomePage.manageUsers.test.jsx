import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import { vi } from 'vitest';

const mockNavigate = vi.fn();

vi.mock('react-router-dom', () => ({
  useNavigate: () => mockNavigate,
}));

vi.mock('../src/ContextForAuth.jsx', () => ({
  useAuth: () => ({ currentUser: { email: 'admin@example.com' } }),
}));

vi.mock('firebase/firestore', async (importOriginal) => {
  const actual = await importOriginal();

  return {
    ...actual,
    getFirestore: () => ({}),
    doc: vi.fn((db, col, id) => ({ _path: `${col}/${id}` })),
    getDoc: vi.fn(async (ref) => {
      if (ref._path.startsWith('profiles/')) {
        return {
          exists: () => true,
          data: () => ({ robots: ['robot123'] }),
        };
      }
      if (ref._path.startsWith('robots/')) {
        return {
          exists: () => true,
          id: 'robot123',
          data: () => ({ id: 'robot123', name: 'Test Robot', admin: 'admin@example.com', users: ['user1@example.com'] }),
        };
      }
      return { exists: () => false };
    }),
    updateDoc: vi.fn(),
    arrayRemove: vi.fn(),
    collection: vi.fn(),
    query: vi.fn(),
    where: vi.fn(),
    getDocs: vi.fn(),
    writeBatch: vi.fn(() => ({ update: vi.fn(), delete: vi.fn(), commit: vi.fn() })),
  };
});

// Mock RobotCard component
vi.mock('../src/components/RobotCard', () => ({
  RobotCard: ({ onManageUsers }) => (
    <div>
      <button onClick={onManageUsers}>Manage Users</button>
    </div>
  ),
}));

import HomePage from '../src/pages/HomePage.jsx';

// Tests
describe('HomePage Manage Users Button', () => {
  beforeEach(() => {
    mockNavigate.mockReset();
  });

  it('calls navigate with correct URL when Manage Users is clicked', async () => {
    render(<HomePage />);

    // Wait for Manage Users button to appear
    const manageBtn = await screen.findByText('Manage Users');

    // Click it
    fireEvent.click(manageBtn);

    // Assert navigation
    expect(mockNavigate).toHaveBeenCalledWith('/ManageUsersPage/robot123');
  });
});
