//routing tests for App component
//ReactApp/tests/app_route_test.jsx
//not testing individual pages, just that routing to them works, more of an example of how to test routing with Vitest
import { vi } from "vitest";
const fakeAuth ={}
vi.mock("firebase/auth", () => ({
    getAuth: vi.fn(() => fakeAuth),

    onAuthStateChanged: vi.fn((auth, callback) => {
      callback({ uid: "user123" });
      return () => {};
  }),
}));

import { render, screen } from "@testing-library/react";
import { MemoryRouter } from "react-router-dom";
import App from "../src/App";



vi.mock("../src/pages/ActivityLogPage.jsx", () => ({ default: () => <div>ActivityLogPage</div> }));
vi.mock("../src/pages/AddRobotPage.jsx",     () => ({ default: () => <div>AddRobotPage</div> }));
vi.mock("../src/pages/ControlPage",          () => ({ default: () => <div>ControlPage</div> }));
vi.mock("../src/pages/ForgotUsernamePage.jsx", () => ({ default: () => <div>ForgotUsernamePage</div> }));
vi.mock("../src/pages/HomePage",             () => ({ default: () => <div>HomePage</div> }));
vi.mock("../src/pages/LoginPage",            () => ({ default: () => <div>LoginPage</div> }));
vi.mock("../src/pages/ResetPasswordPage.jsx", () => ({ default: () => <div>ResetPasswordPage</div> }));
vi.mock("../src/pages/RobotSettingPage.jsx", () => ({ default: () => <div>RobotSettingPage</div> }));
vi.mock("../src/pages/MapViewPage",          () => ({ default: () => <div>MapViewPage</div> }));
vi.mock("../src/pages/SignUpPage.jsx",       () => ({ default: () => <div>SignUpPage</div> }));
vi.mock("../src/pages/TrashViewPage.jsx",    () => ({ default: () => <div>TrashViewPage</div> }));
vi.mock("../src/pages/UserSettingsPage.jsx", () => ({ default: () => <div>UserSettingsPage</div> }));
vi.mock("../src/pages/RobotDashboardPage.jsx", () => ({ default: () => <div>RobotDashboardPage</div> }));

function renderAt(path) {
  return render(
    <MemoryRouter initialEntries={[path]}>
      <App />
    </MemoryRouter>
  );
}

describe("App routing", () => {
  test("renders LoginPage at '/'", () => {
    renderAt("/");
    expect(screen.getByText("LoginPage")).toBeInTheDocument();
  });

  test("renders LoginPage at '/LoginPage'", () => {
    renderAt("/LoginPage");
    expect(screen.getByText("LoginPage")).toBeInTheDocument();
  });

  test("renders HomePage at '/HomePage'", () => {
    renderAt("/HomePage");
    expect(screen.getByText("HomePage")).toBeInTheDocument();
  });

  test("renders ActivityLogPage", () => {
    renderAt("/ActivityLogPage");
    expect(screen.getByText("ActivityLogPage")).toBeInTheDocument();
  });

  test("renders AddRobotPage", () => {
    renderAt("/AddRobotPage");
    expect(screen.getByText("AddRobotPage")).toBeInTheDocument();
  });

  test("renders MapViewPage", () => {
    renderAt("/MapViewPage");
    expect(screen.getByText("MapViewPage")).toBeInTheDocument();
  });

  test("renders SignUpPage", () => {
    renderAt("/SignUpPage");
    expect(screen.getByText("SignUpPage")).toBeInTheDocument();
  });

  test("renders TrashViewPage", () => {
    renderAt("/TrashViewPage");
    expect(screen.getByText("TrashViewPage")).toBeInTheDocument();
  });

  test("renders UserSettingsPage", () => {
    renderAt("/UserSettingsPage");
    expect(screen.getByText("UserSettingsPage")).toBeInTheDocument();
  });

  test("renders ForgotUsernamePage", () => {
    renderAt("/ForgotUsernamePage");
    expect(screen.getByText("ForgotUsernamePage")).toBeInTheDocument();
  });

  test("renders ResetPasswordPage", () => {
    renderAt("/ResetPasswordPage");
    expect(screen.getByText("ResetPasswordPage")).toBeInTheDocument();
  });

  test("renders ControlPage with dynamic robotID", () => {
    renderAt("/ControlPage/RBT-123");
    expect(screen.getByText("ControlPage")).toBeInTheDocument();
  });

  test("renders RobotSettingPage with dynamic robotID", () => {
    renderAt("/RobotSettingPage/RBT-456");
    expect(screen.getByText("RobotSettingPage")).toBeInTheDocument();
  });

  test("renders RobotDashboardPage with dynamic robotID", () => {
    renderAt("/RobotDashboardPage/RBT-789");
    expect(screen.getByText("RobotDashboardPage")).toBeInTheDocument();
  });
});
