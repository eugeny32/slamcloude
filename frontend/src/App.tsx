import { HashRouter, Link, Navigate, Route, Routes } from "react-router-dom";

import { getApiKey } from "./api";
import ApiKeyPage from "./pages/ApiKeyPage";
import ProjectsPage from "./pages/ProjectsPage";
import ScansPage from "./pages/ScansPage";
import ViewerPage from "./pages/ViewerPage";

function TopBar() {
  return (
    <div className="topbar">
      <Link to="/" className="brand">
        slamcloude
      </Link>
      <span className="muted">SHARE S20 — облака точек</span>
      <span className="spacer" />
    </div>
  );
}

function RequireKey({ children }: { children: JSX.Element }) {
  return getApiKey() ? children : <Navigate to="/key" replace />;
}

export default function App() {
  return (
    <HashRouter>
      <TopBar />
      <Routes>
        <Route path="/key" element={<ApiKeyPage />} />
        <Route
          path="/"
          element={
            <RequireKey>
              <ProjectsPage />
            </RequireKey>
          }
        />
        <Route
          path="/projects/:projectId"
          element={
            <RequireKey>
              <ScansPage />
            </RequireKey>
          }
        />
        <Route
          path="/scans/:scanId"
          element={
            <RequireKey>
              <ViewerPage />
            </RequireKey>
          }
        />
      </Routes>
    </HashRouter>
  );
}
