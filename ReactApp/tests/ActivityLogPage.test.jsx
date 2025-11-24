// tests/ActivityLogPage.test.jsx
import React from "react";
import { describe, it, expect, vi, beforeEach, afterEach } from "vitest";
import {
  render,
  screen,
  waitFor,
  fireEvent,
  cleanup,
} from "@testing-library/react";
import "@testing-library/jest-dom/vitest";

const { mockGetDoc, mockGetDocs, eventTableRenderSpy } = vi.hoisted(() => ({
  mockGetDoc: vi.fn(),
  mockGetDocs: vi.fn(),
  eventTableRenderSpy: vi.fn(),
}));

vi.mock("react-router-dom", () => ({
  useNavigate: () => vi.fn(),
}));

vi.mock("firebase/app", () => ({
  initializeApp: vi.fn(() => ({})),
}));

vi.mock("firebase/auth", () => ({
  getAuth: vi.fn(() => ({})),
  GoogleAuthProvider: vi.fn(),
  signInWithPopup: vi.fn(),
}));

vi.mock("firebase/firestore", () => ({
  __esModule: true,
  getFirestore: vi.fn(() => ({})),
  collection: vi.fn((db, path) => ({ db, path })),
  query: vi.fn((...args) => args),
  doc: vi.fn((db, coll, id) => ({ db, coll, id })),
  where: vi.fn((field, op, value) => ({ field, op, value })),
  orderBy: vi.fn((field, dir) => ({ field, dir })),
  limit: vi.fn((n) => n),
  getDoc: mockGetDoc,
  getDocs: mockGetDocs,
}));

vi.mock("../src/ContextForAuth.jsx", () => ({
  __esModule: true,
  useAuth: () => ({ currentUser: { email: "user@example.com" } }),
}));

vi.mock("../src/components/LogoutButton.jsx", () => ({
  __esModule: true,
  default: () => <button data-testid="logout-button">Logout</button>,
}));

vi.mock("../src/components/BackButton.jsx", () => ({
  __esModule: true,
  default: () => <button data-testid="back-button">Back</button>,
}));

vi.mock("../src/components/EventTable.jsx", () => ({
  __esModule: true,
  default: (props) => {
    eventTableRenderSpy(props);
    const { events } = props;
    return (
      <table data-testid="event-table">
        <tbody>
          {events.map((e) => (
            <tr key={e.eventId}>
              <td>{e.robotId}</td>
              <td>{e.category}</td>
              <td>{e.location}</td>
            </tr>
          ))}
        </tbody>
      </table>
    );
  },
}));

import ActivityLogPage from "../src/pages/ActivityLogPage.jsx";

beforeEach(() => {
  vi.clearAllMocks();
});

afterEach(() => {
  cleanup();
});

function makeEventDoc(id, override = {}) {
  const n = Number(id.replace("e", ""));
  return {
    id,
    data: () => ({
      robotId: "robot-1",
      category: "Alert",
      location: "Lab",
      time: { toDate: () => new Date(2025, 0, 1, 10, 0, n) },
      image_url: "http://example.com/img.jpg",
      ...override,
    }),
  };
}

describe("ActivityLogPage", () => {
  it("fetches and renders events for the current user", async () => {
    mockGetDoc.mockResolvedValue({
      exists: () => true,
      data: () => ({ robots: ["robot-1"] }),
    });

    mockGetDocs.mockResolvedValue({
      docs: [makeEventDoc("e0", { category: "Intrusion", location: "Hallway" })],
    });

    render(<ActivityLogPage />);

    expect(
      await screen.findByRole("heading", { name: /Activity Log/i })
    ).toBeInTheDocument();

    await waitFor(() => {
      const call = eventTableRenderSpy.mock.calls.at(-1)[0];
      expect(call.events).toHaveLength(1);
      expect(call.events[0].robotId).toBe("robot-1");
    });

    expect(screen.getByText("robot-1")).toBeInTheDocument();
    expect(screen.getByText("Intrusion")).toBeInTheDocument();
    expect(screen.getByText("Hallway")).toBeInTheDocument();
    expect(screen.getByText(/Last refresh:/i)).toBeInTheDocument();
  });

  it("shows an error when user has no robots", async () => {
    mockGetDoc.mockResolvedValue({
      exists: () => true,
      data: () => ({ robots: [] }),
    });

    mockGetDocs.mockResolvedValue({ docs: [] });

    render(<ActivityLogPage />);

    await waitFor(() => {
      expect(
        screen.getByText(/You don't have any robots associated/i)
      ).toBeInTheDocument();
    });

    expect(screen.queryByTestId("event-table")).not.toBeInTheDocument();
  });

  it("shows 'No events yet' when robots exist but events do not", async () => {
    mockGetDoc.mockResolvedValue({
      exists: () => true,
      data: () => ({ robots: ["robot-1"] }),
    });

    mockGetDocs.mockResolvedValue({ docs: [] });

    render(<ActivityLogPage />);

    await waitFor(() => {
      expect(
        screen.getByText(/No events yet\. Try refreshing/i)
      ).toBeInTheDocument();
    });

    expect(screen.getByRole("button", { name: /Refresh/i })).toBeInTheDocument();
  });

  it("supports pagination using Next and Previous buttons", async () => {
    mockGetDoc.mockResolvedValue({
      exists: () => true,
      data: () => ({ robots: ["robot-1"] }),
    });

    const manyDocs = Array.from({ length: 20 }, (_, i) =>
      makeEventDoc(`e${i}`, {
        category: `Cat-${i}`,
        location: `Loc-${i}`,
      })
    );

    mockGetDocs.mockResolvedValue({ docs: manyDocs });

    render(<ActivityLogPage />);

    await waitFor(() => {
      expect(eventTableRenderSpy).toHaveBeenCalled();
    });

    let call = eventTableRenderSpy.mock.calls.at(-1)[0];
    expect(call.events).toHaveLength(15);
    expect(call.events[0].eventId).toBe("e19");

    const nextButton = await screen.findByRole("button", { name: /Next/i });
    const prevButton = await screen.findByRole("button", { name: /Previous/i });

    expect(prevButton).toBeDisabled();

    fireEvent.click(nextButton);

    await waitFor(() => {
      const c = eventTableRenderSpy.mock.calls.at(-1)[0];
      expect(c.events[0].eventId).toBe("e4");
      expect(c.events.length).toBe(5);
    });

    expect(prevButton).not.toBeDisabled();

    fireEvent.click(prevButton);

    await waitFor(() => {
      const c = eventTableRenderSpy.mock.calls.at(-1)[0];
      expect(c.events[0].eventId).toBe("e19");
      expect(c.events.length).toBe(15);
    });

    await waitFor(() => {
        expect(prevButton).toBeDisabled();
    });
  });
});
