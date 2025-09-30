import React, { useEffect, useState } from "react";
import { useNavigate, useParams } from "react-router-dom";
import { Box, Typography, Button, CircularProgress } from "@mui/material";
import { db } from "../firebaseConfig";
import { query, where, collection, getDocs, updateDoc, writeBatch, arrayRemove, doc, getDoc } from "firebase/firestore";
import { useRobotConnection } from "../hooks/useRobotConnection";
import { useAuth } from "../ContextForAuth";
import BackButton from "../components/BackButton";

function RobotSettingPage() {
  const navigate = useNavigate();
  const { robotID } = useParams();
  const { currentUser } = useAuth();

  const [robot, setRobot] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    (async () => {
      try {
        const snap = await getDoc(doc(db, "robots", robotID));
        if (!snap.exists()) throw new Error("Robot not found");
        setRobot({ id: snap.id, ...snap.data() });
      } catch (e) {
        alert(e.message || String(e));
        navigate("/HomePage");
      } finally {
        setLoading(false);
      }
    })();
  }, [robotID, navigate]);

  const robotIP = robot?.ipAddress || "";
  const { connectionStatus, sendCommand } = useRobotConnection(robotIP, null, null, null);

  const isAdmin = !!robot?.admin && !!currentUser?.email && robot.admin.toLowerCase() === currentUser.email.toLowerCase();

  const send = async (payload) => {
    try { await sendCommand(payload); }
    catch (e) { alert(`Command failed: ${e}`); }
  };

  const needConnected = () => {
    if (connectionStatus !== "connected") {
      alert("Robot not connected yet.");
      return false;
    }
    return true;
  };

  const norm = (s) => (s || "").toString().trim().toLowerCase();

  const onDelete = async () => {
  try {
    if (!currentUser?.email) {
      alert("You must be logged in to delete a robot.");
      return;
    }
    const rid = robotID || robot?.id;
    if (!rid) {
      alert("Missing robot ID.");
      return;
    }

    const meEmail = norm(currentUser.email);
    const robotRef = doc(db, "robots", rid);
    const robotSnap = await getDoc(robotRef);

    if (!robotSnap.exists()) {
      //robot already deleted or does not exist
      await updateDoc(doc(db, "profiles", currentUser.email), {
        robots: arrayRemove(rid),
      });
      alert("Robot removed from your profile.");
      navigate("/HomePage");
      return;
    }

    const data = robotSnap.data();
    const isAdmin = norm(data.admin) === meEmail;

    if (!isAdmin) {
      //not admin, just remove from profile
      const profileRef = doc(db, "profiles", currentUser.email);
      await Promise.all([
        updateDoc(profileRef, { robots: arrayRemove(rid) }),
        updateDoc(robotRef, { users: arrayRemove(currentUser.email) }),
      ]);
      alert("Robot removed from your profile.");
      navigate("/HomePage");
      return;
    }

    //admin, delete robot and remove from all profiles
    if (!window.confirm("Delete this robot for all users?")) return;

    //finding all profiles with this robot
    const qprofiles = query(
      collection(db, "profiles"),
      where("robots", "array-contains", rid)
    );
    const profSnap = await getDocs(qprofiles);
    const batch = writeBatch(db);

    //remove robotid from each profile
    profSnap.forEach((p) => batch.update(p.ref, { robots: arrayRemove(rid) }));
    batch.delete(robotRef);

    await batch.commit();
    alert("Robot deleted for all users.");
    navigate("/HomePage");
  } catch (error) {
    console.error("Error deleting robot:", error);
    alert("Failed to delete robot. See console for details.");
  }
};

  const onShutdown = async () => {
    if (!needConnected()) return;
    if (!window.confirm("Are you sure you want to shutdown robot?")) return;
    await send({ type: "system", action: "shutdown" });
    alert("Shutdown command sent.");
  };

  const onUpdate = async () => {
    if (!isAdmin) return alert("Admins only.");
    if (!needConnected()) return;
    if (!window.confirm("Robot may reboot after update.")) return;
    await send({ type: "system", action: "update" });
    alert("Update triggered.");
  };

  const onWifi = async () => {
    if (!isAdmin) return alert("Admins only.");
    if (!needConnected()) return;
    const ssid = window.prompt("New Wi-Fi SSID:");
    if (!ssid) return;
    const password = window.prompt("Wi-Fi Password:") || "";
    await send({ type: "network", action: "set_wifi", ssid, password });
    alert(`Wi-Fi change requested: ${ssid}`);
  };
  
  if (loading) {
    return (
      <Box sx={{ minHeight: "100vh", display: "grid", placeItems: "center" }}>
        <CircularProgress />
      </Box>
    );
  }

  return (
    <Box
      sx={{
        width: "100%",
        minHeight: "100vh",
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        paddingTop: 7,
        gap: 2
      }}
    >
      <BackButton path={`/RobotDashboardPage/${robot.id}`} />

      <Button
        // Delete Robot Button
        variant="contained"
        onClick={onDelete}
        sx={{
          backgroundColor: "#EC221F",
          border: "1px solid #C00F0C",
          borderRadius: "8px",
          width: 194,
          height: 69,
          mb: 2,
          "&:hover": {
            backgroundColor: "#C00F0C",
          }
        }}
      >
        Delete Robot
      </Button>

      <Button
        // Shutdown Robot Button
        variant="contained"
        onClick={onShutdown}
        sx={{
          backgroundColor: "#EC221F",
          border: "1px solid #C00F0C",
          borderRadius: "8px",
          width: 194,
          height: 69,
          mb: 2,
          "&:hover": {
            backgroundColor: "#C00F0C",
          }
        }}
      >
        Shutdown
      </Button>

      <Button
        // Update Software Button
        variant="contained"
        onClick={onUpdate}
        sx={{
          backgroundColor: "#5A5A5A",
          border: "1px solid #C00F0C",
          borderRadius: "8px",
          width: 194,
          height: 69,
          mb: 2,
          "&:hover": {
            backgroundColor: "#4A4A4A",
          }
        }}
      >
        Update Software
      </Button>

      <Button
        // Change Network Name Button
        variant="contained"
        onClick={onWifi}
        sx={{
          backgroundColor: "#5A5A5A",
          border: "1px solid #C00F0C",
          borderRadius: "8px",
          width: 194,
          height: 69,
          mb: 2,
          "&:hover": {
            backgroundColor: "#4A4A4A"
          }
        }}
      >
        Change Network Name
      </Button>

      <Box
        sx={{
          mt: 25,
          mb: 4,
          textAlign: "center"
        }}
      >
        <Typography
          sx={{
            color: "#000000",
            fontFamily: "Inter",
            fontSize: 16,
            lineHeight: "100%"
          }}
        >
          Operating System: {robot?.os || "Ubuntu 24.04"}
        </Typography>

        <Typography
          sx={{
            color: "#000000",
            fontFamily: "Inter",
            fontSize: 16,
            lineHeight: "100%"
          }}
        >
          ROS Version: {robot?.rosVersion || "Jazzy Jalisco"}
        </Typography>

        <Typography
          sx={{
            color: "#000000",
            fontFamily: "Inter",
            fontSize: 16,
            lineHeight: "100%"
          }}
        >
          IP Address: {robot?.ipAddress || "—"}
        </Typography>

        <Typography
          sx={{
            color: "#000000",
            fontFamily: "Inter",
            fontSize: 16,
            lineHeight: "100%"
          }}
        >
          Date Added To Network: {robot?.createdAt?.toDate().toLocaleString() || "—"}
        </Typography>

        <Typography
          sx={{
            color: "#000000",
            fontFamily: "Inter",
            fontSize: 16,
            lineHeight: "100%"
          }}
        >
          Connection: {connectionStatus}
        </Typography>
        
      </Box>
    </Box>
  );
};

export default RobotSettingPage;
