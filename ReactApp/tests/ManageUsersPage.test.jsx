import { vi } from "vitest";
import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import React from "react";
import { MemoryRouter, Routes, Route } from "react-router-dom";

const usersState = { users: ["user1@example.com"] };

vi.mock("firebase/firestore", async (importOriginal) => {
  const actual = await importOriginal();
  return {
    ...actual,
    doc: vi.fn((db, col, id) => ({ _path: `${col}/${id}` })),
    getDoc: vi.fn(async (ref) => ({
      exists: () => true,
      id: "robot123",
      data: () => ({ id: "robot123", name: "Test Robot", users: [...usersState.users] }),
    })),
    updateDoc: vi.fn(async (ref, { users }) => {
      if (usersState.users.includes(users)) {
        // remove
        usersState.users = usersState.users.filter((u) => u !== users);
      } else {
        // add
        usersState.users.push(users);
      }
    }),
    arrayUnion: (val) => val,
    arrayRemove: (val) => val,
  };
});

import ManageUsersPage from "../src/pages/ManageUsersPage.jsx";
import { updateDoc } from "firebase/firestore"; // to assert calls

// Tests
describe("ManageUsersPage", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("renders robot users, allows adding and removing users", async () => {
    render(
      <MemoryRouter initialEntries={["/ManageUsersPage/robot123"]}>
        <Routes>
          <Route path="/ManageUsersPage/:robotID" element={<ManageUsersPage />} />
        </Routes>
      </MemoryRouter>
    );


    await screen.findByText("Manage Users - Test Robot");

    expect(screen.getByText("user1@example.com")).toBeInTheDocument();

    // Add new user
    const input = screen.getByPlaceholderText("Enter user email");
    fireEvent.change(input, { target: { value: "newuser@example.com" } });
    fireEvent.click(screen.getByText("Add"));

    await waitFor(() => {
      expect(updateDoc).toHaveBeenCalledWith(
        { _path: "robots/robot123" },
        { users: "newuser@example.com" }
      );
      // New user appears in UI
      expect(screen.getByText("newuser@example.com")).toBeInTheDocument();
    });

    // Remove a user
    const removeBtn = screen.getByText("newuser@example.com").closest("div").querySelector("button");
    fireEvent.click(removeBtn);

    await waitFor(() => {
      expect(updateDoc).toHaveBeenCalledWith(
        { _path: "robots/robot123" },
        { users: "newuser@example.com" }
      );
      // Removed user no longer in UI
      expect(screen.queryByText("newuser@example.com")).not.toBeInTheDocument();
    });
  });
});
