// tests/RobotDashboard.map.test.jsx
import React from "react";
import { render, screen, fireEvent, waitFor } from "@testing-library/react";
import { describe, it, expect, vi, beforeEach } from "vitest";

// Mock BackButton component so it doesn't call useNavigate and yell at me
vi.mock("../src/components/BackButton.jsx", () => ({
  default: () => <div data-testid="mock-back-btn" />,
}));

vi.mock("../src/firebaseConfig", () => ({
  db: {},
}));

vi.mock("firebase/firestore", () => {
  const onSnapshot = (q, cb) => {
    const fakeDoc = { data: () => ({ latitude: 38.5608, longitude: -121.424 }) };
    const snapshot = { empty: false, docs: [fakeDoc] };
    cb(snapshot);
    return () => {};
  };

  const setDoc = vi.fn();

  return {
    __esModule: true,
    collection: vi.fn(),
    query: vi.fn(),
    orderBy: vi.fn(),
    limit: vi.fn(),
    where: vi.fn(),
    onSnapshot,
    doc: vi.fn((db, coll, id) => ({ coll, id })),
    getDoc: vi.fn(), 
    setDoc,
    serverTimestamp: () => new Date(),
  };
});

vi.mock("@react-google-maps/api", () => ({
  GoogleMap: ({ children, onClick }) => (
    <div
      data-testid="google-map"
      onClick={() =>
        onClick &&
        onClick({
          latLng: {
            lat: () => 38.56123,
            lng: () => -121.42345,
          },
        })
      }
    >
      {children}
    </div>
  ),
  Marker: ({ label }) => <div data-testid="marker">{label}</div>,
  Circle: () => <div data-testid="circle" />,
  DirectionsRenderer: () => <div data-testid="directions" />,
  useLoadScript: () => ({ isLoaded: true }),
}));

import { setDoc } from "firebase/firestore";

const fakeRoutePoints = [
  { lat: 38.5608, lng: -121.4240 },
  { lat: 38.5610, lng: -121.4238 },
];

if (!global.window) {
  // jsdom usually defines this already, but just in case
  // @ts-ignore
  global.window = {};
}

// Fake google.maps.DirectionsService so the real getRoutePoints works
window.google = {
  maps: {
    TravelMode: { WALKING: "WALKING" },
    DirectionsService: function () {
      this.route = (options, callback) => {
        const makeLatLng = (lat, lng) => ({
          lat: () => lat,
          lng: () => lng,
        });

        const result = {
          routes: [
            {
              overview_path: [
                makeLatLng(fakeRoutePoints[0].lat, fakeRoutePoints[0].lng),
                makeLatLng(fakeRoutePoints[1].lat, fakeRoutePoints[1].lng),
              ],
            },
          ],
        };

        callback(result, "OK");
      };
    },
  },
};

import { MapTab } from "../src/pages/RobotDashboardPage";

describe("MapTab", () => {
  beforeEach(() => {
    setDoc.mockClear();
  });

  it("opens dialog on map click and writes waypoint + route to Firestore on confirm", async () => {
    render(<MapTab robotID="robot-123" />);


    const map = screen.getByTestId("google-map");
    fireEvent.click(map);


    const dialogTitle = await screen.findByText("Set Waypoint?");
    expect(dialogTitle).toBeInTheDocument();


    const confirmButton = screen.getByRole("button", { name: /confirm/i });
    fireEvent.click(confirmButton);


    await waitFor(() => {
      expect(setDoc).toHaveBeenCalledTimes(1);
    });

    const [, dataArg] = setDoc.mock.calls[0];

    expect(dataArg).toMatchObject({
      latitude: expect.any(Number),
      longitude: expect.any(Number),
      path: fakeRoutePoints,
    });
  });
});