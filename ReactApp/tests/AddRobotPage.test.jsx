// tests/AddRobotPage.test.jsx
import React from "react";
import {
  describe,
  it,
  expect,
  vi,
  beforeEach,
  afterEach,
} from "vitest";
import {
  render,
  screen,
  fireEvent,
  waitFor,
  cleanup,
} from "@testing-library/react";
import "@testing-library/jest-dom/vitest";

const {
  mockAddDoc,
  mockSetDoc,
  mockCollection,
  mockDoc,
  navigateMock,
} = vi.hoisted(() => ({
  mockAddDoc: vi.fn(),
  mockSetDoc: vi.fn(),
  mockCollection: vi.fn((db, path) => ({ db, path })),
  mockDoc: vi.fn((db, coll, id) => ({ db, coll, id })),
  navigateMock: vi.fn(),
}));

vi.mock("react-router-dom", () => ({
  useNavigate: () => navigateMock,
}));

vi.mock("firebase/firestore", () => ({
  __esModule: true,
  collection: mockCollection,
  addDoc: mockAddDoc,
  doc: mockDoc,
  setDoc: mockSetDoc,
  arrayUnion: (...args) => ({ __arrayUnion: args }),
  serverTimestamp: () => "__serverTimestamp__",
}));

vi.mock("../src/firebaseConfig", () => ({
  __esModule: true,
  db: {},
}));

vi.mock("../src/ContextForAuth.jsx", () => ({
  __esModule: true,
  useAuth: () => ({
    currentUser: { email: "owner@example.com" },
  }),
}));

vi.mock("../src/components/LogoutButton", () => ({
  __esModule: true,
  default: () => <button data-testid="logout-button">Logout</button>,
}));

vi.mock("../src/components/BackButton", () => ({
  __esModule: true,
  default: ({ path }) => (
    <button data-testid="back-button">Back</button>
  ),
}));

import AddRobotPage from "../src/pages/AddRobotPage.jsx";

beforeEach(() => {
  vi.clearAllMocks();

  if (!global.navigator) {
    global.navigator = {};
  }
  if (!navigator.clipboard) {
    navigator.clipboard = { writeText: vi.fn().mockResolvedValue() };
  } else {
    navigator.clipboard.writeText = vi.fn().mockResolvedValue();
  }
});

afterEach(() => {
  cleanup();
});

describe("AddRobotPage", () => {
  it("disables Add Robot when fields are empty and enables when filled", () => {
    render(<AddRobotPage />);

    const addButton = screen.getByRole("button", { name: /Add Robot/i });
    expect(addButton).toBeDisabled();

    const nameInput = screen.getByLabelText(/Name/i);
    const ipInput = screen.getByLabelText(/IP Address/i);

    fireEvent.change(nameInput, { target: { value: "TestBot" } });
    fireEvent.change(ipInput, { target: { value: "192.168.0.10" } });

    expect(addButton).not.toBeDisabled();
  });

  it("adds a robot, updates profiles, shows snackbar and supports copy ID", async () => {
    mockAddDoc.mockResolvedValue({ id: "robot123" });
    mockSetDoc.mockResolvedValue(undefined);

    render(<AddRobotPage />);

    const nameInput = screen.getByLabelText(/Name/i);
    const ipInput = screen.getByLabelText(/IP Address/i);
    const addButton = screen.getByRole("button", { name: /Add Robot/i });

    fireEvent.change(nameInput, { target: { value: "My Robot" } });
    fireEvent.change(ipInput, { target: { value: "10.0.0.42" } });

    fireEvent.click(addButton);

    await waitFor(() => {
      expect(mockAddDoc).toHaveBeenCalled();
    });

    expect(mockCollection).toHaveBeenCalledWith(expect.anything(), "robots");
    const addArgs = mockAddDoc.mock.calls[0][1];

    expect(addArgs.name).toBe("My Robot");
    expect(addArgs.ipAddress).toBe("10.0.0.42");
    expect(addArgs.status).toBe("idle");
    expect(addArgs.location).toBe("dock");
    expect(addArgs.admin).toBe("owner@example.com");
    expect(Array.isArray(addArgs.users)).toBe(true);
    expect(addArgs.users).toContain("owner@example.com");
    expect(addArgs.createdAt).toBe("__serverTimestamp__");

    await waitFor(() => {
      expect(mockSetDoc).toHaveBeenCalledTimes(1);
    });

    const [docArg, profileData, options] = mockSetDoc.mock.calls[0];
    expect(docArg).toMatchObject({
      coll: "profiles",
      id: "owner@example.com",
    });
    expect(options).toEqual({ merge: true });
    expect(profileData.robots.__arrayUnion[0]).toBe("robot123");

    expect(
      await screen.findByText(/Robot added successfully/i) 
    ).toBeInTheDocument();
    expect(await screen.findByText("robot123")).toBeInTheDocument();

    const copyButton = screen.getByRole("button", { name: /Copy ID/i });
    fireEvent.click(copyButton);

    await waitFor(() => {
      expect(navigator.clipboard.writeText).toHaveBeenCalledWith("robot123");
    });

    expect(await screen.findByText(/Copied/i)).toBeInTheDocument();
  });
});