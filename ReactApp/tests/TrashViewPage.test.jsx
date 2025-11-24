// tests/TrashViewPage.test.jsx
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
  waitFor,
  cleanup,
} from "@testing-library/react";
import "@testing-library/jest-dom/vitest";

const { mockGetDoc, mockDoc, locationState, displayCardSpy } = vi.hoisted(() => ({
  mockGetDoc: vi.fn(),
  mockDoc: vi.fn((db, coll, id) => ({ db, coll, id })),
  locationState: {},
  displayCardSpy: vi.fn(),
}));

vi.mock("react-router-dom", () => ({
  useNavigate: () => vi.fn(),
  useLocation: () => ({ state: locationState }),
}));

vi.mock("firebase/firestore", () => ({
  __esModule: true,
  doc: mockDoc,
  getDoc: mockGetDoc,
}));

vi.mock("../src/firebaseConfig", () => ({
  __esModule: true,
  db: {},
}));

vi.mock("../src/components/BackButton", () => ({
  __esModule: true,
  default: ({ path }) => (
    <button data-testid="back-button">Back</button>
  ),
}));

vi.mock("../src/components/LogoutButton", () => ({
  __esModule: true,
  default: () => (
    <button data-testid="logout-button">Logout</button>
  ),
}));

vi.mock("../src/components/DisplayCard", () => ({
  __esModule: true,
  default: (props) => {
    displayCardSpy(props);
    const { title, image, metadata } = props;
    return (
      <div data-testid="display-card">
        <h2>{title}</h2>
        <img src={image} alt="detected" />
        <div>Category: {metadata.Category}</div>
        <div>Location: {metadata.Location}</div>
        <div>Robot: {metadata.Robot_ID}</div>
        <div>Confidence: {String(metadata.Confidence)}</div>
        <div>Timestamp: {metadata.Timestamp}</div>
      </div>
    );
  },
}));

import TrashViewPage from "../src/pages/TrashViewPage.jsx";

beforeEach(() => {
  vi.clearAllMocks();
  Object.keys(locationState).forEach((k) => delete locationState[k]);
});

afterEach(() => {
  cleanup();
});

describe("TrashViewPage", () => {
  it("renders using location state when there is no eventId", async () => {
    const tMs = 1700000000000;
    Object.assign(locationState, {
      imageUrl: "http://example.com/from-state.jpg",
      robotId: "robot-1",
      category: "StateCat",
      location: "StateLoc",
      confidence: 0.95,
      timeMS: tMs,
    });

    mockGetDoc.mockResolvedValue(null);

    render(<TrashViewPage />);

    expect(
      screen.getByRole("heading", { name: /Robot Detection Display/i })
    ).toBeInTheDocument();

    await waitFor(() => {
      expect(displayCardSpy).toHaveBeenCalled();
    });

    const lastCall = displayCardSpy.mock.calls.at(-1)[0];

    expect(lastCall.image).toBe("http://example.com/from-state.jpg");
    expect(lastCall.metadata.Category).toBe("StateCat");
    expect(lastCall.metadata.Location).toBe("StateLoc");
    expect(lastCall.metadata.Robot_ID).toBe("robot-1");
    expect(lastCall.metadata.Confidence).toBe(0.95);
    expect(lastCall.metadata.Timestamp).not.toBe("");
    expect(mockGetDoc).not.toHaveBeenCalled();
  });

  it("fetches event by eventId and uses firestore data", async () => {
    Object.assign(locationState, {
      eventId: "ev123",
      imageUrl: null,
      robotId: "robot-from-state",
      category: "OldCat",
      location: "OldLoc",
      confidence: null,
      timeMS: null,
    });

    mockGetDoc.mockResolvedValue({
      exists: () => true,
      data: () => ({
        image_url: "http://example.com/from-db.jpg",
        robotId: "robot-db",
        category: "DbCat",
        location: "DbLoc",
        confidence: 0.7,
        time: {
          toDate: () => new Date("2025-01-01T10:00:00Z"),
        },
      }),
    });

    render(<TrashViewPage />);

    await waitFor(() => {
      expect(displayCardSpy).toHaveBeenCalled();
    });

    expect(mockDoc).toHaveBeenCalledWith(expect.anything(), "events", "ev123");
    const props = displayCardSpy.mock.calls.at(-1)[0];

    expect(props.image).toBe("http://example.com/from-db.jpg");
    expect(props.metadata.Category).toBe("DbCat");
    expect(props.metadata.Location).toBe("DbLoc");
    expect(props.metadata.Robot_ID).toBe("robot-db");
    expect(props.metadata.Confidence).toBe(0.7);
    expect(props.metadata.Timestamp).toContain("2025");
  });

  it("uses placeholder image when no image is available", async () => {
    Object.assign(locationState, {
      eventId: "noimg",
      imageUrl: null,
    });

    mockGetDoc.mockResolvedValue({
      exists: () => true,
      data: () => ({
        robotId: "robot-noimg",
        category: "NoImgCat",
        location: "NoImgLoc",
      }),
    });

    render(<TrashViewPage />);

    await waitFor(() => {
      expect(displayCardSpy).toHaveBeenCalled();
    });

    const props = displayCardSpy.mock.calls.at(-1)[0];

    expect(props.image).toBe("https://picsum.photos/300");
    expect(props.metadata.Robot_ID).toBe("robot-noimg"); 
    expect(props.metadata.Category).toBe("NoImgCat");
    expect(props.metadata.Location).toBe("NoImgLoc");
  });
});