import React, { useEffect, useMemo, useRef, useState } from "react";
import { useSearchParams, useParams } from "react-router-dom";
import { AppBar, Tabs, Tab, Box, Card, Button, Grid, Typography, GlobalStyles, Dialog, DialogTitle, DialogContent, DialogActions } from "@mui/material";
import { ArrowBack, ArrowDownward, ArrowForward, ArrowUpward } from "@mui/icons-material";
import BackButton from "../components/BackButton";
import SettingsButton from "../components/SettingsButton";
import { db } from "../firebaseConfig";
import { doc, getDoc, collection, query, limit, onSnapshot, orderBy, where, setDoc } from "firebase/firestore";
import { GoogleMap, Marker as GMarker, Circle as GCircle, useLoadScript, DirectionsRenderer } from "@react-google-maps/api";
import { serverTimestamp } from "firebase/firestore";
import LogoutButton from "../components/LogoutButton";
import { useRobotConnection } from "../hooks/useRobotConnection";
import { QuestionMark } from "@mui/icons-material";

function a11yProps(index) { return { id: `tab-${index}`, "aria-controls": `tabpanel-${index}` }; }
function TabPanel({ children, value, index }) {
  return (
  <div
      role="tabpanel"
      style={{ display: value === index ? "block" : "none" }}
      id={`tabpanel-${index}`}
      aria-labelledby={`tab-${index}`}
    >
      <Box sx={{ p: 2 }}>{children}</Box>
    </div>
  );
}

// From Mapview

function clampRadius(meters) {
  if (Number.isNaN(meters)) {
    return 5;
  }
  return Math.max(5, Math.min(100, meters));
}

// Main Dashboard
export default function RobotDashboard() {
  const { robotID } = useParams();
  const [robotIP, setRobotIP] = useState("");
  const videoRef = useRef(null);
  const [connection, setConnection] = useState(null);
  const [params, setParams] = useSearchParams();
  const initialTab = params.get("tab") === "map" ? 1 : 0;
  const [tab, setTab] = useState(initialTab);

  const [batteryVoltage, setBatteryVoltage] = useState(null);
  const [wifiStrength, setWifiStrength] = useState(null);


  function openInNewTab( url ){
    window.open(url, "_blank");
  }
  // Function to fetch the robot's IP address from Firestore using its ID
  useEffect(() => {
  const fetchRobotIP = async () => {
    try {
      const robotDoc = await getDoc(doc(db, "robots", robotID));
      if (robotDoc.exists()) {
        setRobotIP(robotDoc.data().ipAddress || "");
      } else {
        console.warn("No robot found with the given ID.");
      }
    } catch (e) {
      console.error("Error fetching robot IP address:", e);
    }
  };
  fetchRobotIP();
  }, [robotID]);


  // Call the hook at the top level. The hook can internally handle when robotIP changes.
  const { isConnected, connectionStatus, error, sendCommand } = useRobotConnection(
    robotIP,
    videoRef,
    setBatteryVoltage,
    setWifiStrength
  );

  useEffect(() => {
    const next = new URLSearchParams(params);
    if (tab === 1) next.set("tab", "map"); else next.delete("tab");
    setParams(next, { replace: true });
  }, [tab]);

  return (
    <Box sx={{ p: 2 }}>
      <Box sx={{ display: "flex", gap: 1, alignItems: "center", mb: 1 }}>
        <SettingsButton path={`/RobotSettingPage/${robotID}`} />
        <BackButton path="/HomePage" />
        <Typography variant="h5" sx={{ ml: 1 }}>RobotID: {robotID}</Typography>
        <Button color="inherit" size="small" startIcon={<QuestionMark/>}
          onClick={() => openInNewTab('/docs/robot-id')}
        >
        </Button>
      </Box>

      <AppBar position="static" color="default" elevation={1}>
        <Tabs value={tab} onChange={(_, v) => setTab(v)} variant="fullWidth">
          <Tab label="Control" {...a11yProps(0)} />
          <Tab label="Map" {...a11yProps(1)} />
        </Tabs>
      </AppBar>

      <TabPanel value={tab} index={0}>
        <ControlTab
          videoRef={videoRef}
          isConnected={isConnected}
          connectionStatus={connectionStatus}
          batteryVoltage={batteryVoltage}
          wifiStrength={wifiStrength}
          robotIP={robotIP}
          sendCommand={sendCommand}
          error={error}
        />
      </TabPanel>

      <TabPanel value={tab} index={1}>
        <MapTab robotID={robotID} />
      </TabPanel>
    </Box>
  );
}

// Control Tab
function ControlTab({ videoRef, isConnected, connectionStatus, batteryVoltage, wifiStrength, robotIP, sendCommand, error }) {
  const intervalRef = useRef(null);

  const sendContinuous = (command) => {
    // Stop any existing loop
    stopSending();

    // Immediately send first command
    sendCommand(command);

    // Start loop every 100ms (10 Hz, required by ROS)
    intervalRef.current = setInterval(() => {
      sendCommand(command);
    }, 100);
  };

  const stopSending = () => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    sendCommand("stop");
  };

  const buttonEvents = (cmd) => ({
    onMouseDown: () => sendContinuous(cmd),
    onMouseUp: stopSending,
    onMouseLeave: stopSending,
    onTouchStart: () => sendContinuous(cmd),
    onTouchEnd: stopSending
  });

  return (
    <Box sx={{ display: "flex", flexDirection: "column", alignItems: "center" }}>
      <Typography variant="h4" sx={{ mb: 1 }}>Robot Control</Typography>

      <Card
        sx={{
          width: 600,
          height: 400,
          mb: 2,
          border: "1px solid #0001",
          borderRadius: 2,
          overflow: "hidden",
        }}>
        {robotIP ? (
          <video
            ref={videoRef}
            muted
            autoPlay
            playsInline
            style={{
              width: "100%",
              height: "100%",
              objectFit: "cover"
            }}
          />
        ) : (
          <Box sx={{ p: 2, textAlign: "center" }}><Typography variant="h6">No Live Feed Detected…</Typography></Box>
        )}
      </Card>

      <Box sx={{ display: "flex", gap: 2, mb: 2 }}>
        <Typography>Connection Status: <strong>{isConnected ? "Connected" : "Disconnected"}</strong></Typography>
        <Typography>Battery Voltage: <strong>{batteryVoltage ?? "N/A"}</strong></Typography>
        <Typography>Wifi Strength: <strong>{wifiStrength != null ? `${wifiStrength} dB` : "N/A"}</strong></Typography>
      </Box>

      {/* Movement Controls */}
      <Box sx={{ display: "flex", flexDirection: "column", alignItems: "center", gap: 2 }}>
        
        {/* Forward */}
        <Button aria-label="Forward" {...buttonEvents("forward")} variant="contained">
          <ArrowUpward sx={{ fontSize: 50 }} />
        </Button>

        <Box sx={{ display: "flex", gap: 12 }}>
          {/* Left */}
          <Button aria-label="Left" {...buttonEvents("left")} variant="contained">
            <ArrowBack sx={{ fontSize: 50 }} />
          </Button>

          {/* Right */}
          <Button aria-label="Right" {...buttonEvents("right")} variant="contained">
            <ArrowForward sx={{ fontSize: 50 }} />
          </Button>
        </Box>

        {/* Back */}
        <Button aria-label="Back"{...buttonEvents("back")} variant="contained">
          <ArrowDownward sx={{ fontSize: 50 }} />
        </Button>
      </Box>
    </Box>
  );
}
//the function that uses google directions API which will retrun an array of [lat,lng] waypoints for the robot to follow
function getRoutePoints(origin,destination){
  if(!window.google || !window.google.maps){
    return Promise.reject(new Error("Google Maps API not loaded"));
  }

  const directionsService = new window.google.maps.DirectionsService();

  return new Promise((resolve, reject) => {
    directionsService.route(
      {
        origin,
        destination,
        travelMode: window.google.maps.TravelMode.WALKING,
      },
      (result, status) => {
        if (status === "OK" && result.routes && result.routes.length > 0) {
          const route = result.routes[0];
          const overviewPath = route.overview_path || [];
          const points = overviewPath.map((latLng) => ({
            lat: latLng.lat(),
            lng: latLng.lng(),
          }));
          resolve({ points, directions: result });
        } else {
          reject(new Error("Directions request failed due to " + status));
        }
      }
    );
  });
}




// Map Tab
function MapTab({ robotID }) {
  const defaultCenter = useMemo(() => ({ lat: 38.56080, lng: -121.42400 }), []);
  const [position, setPosition] = useState(defaultCenter);
  const [isRadiusOn, setIsRadiusOn] = useState(false);
  const [radiusInput, setRadiusInput] = useState("25");
  const [lockedCenter, setLockedCenter] = useState(null);
  const mapRadius = clampRadius(Number(radiusInput));
  const [waypoint, setWaypoint] = useState(null);
  const [pendingWaypoint, setPendingWaypoint] = useState(null);
  const [dialogOpen, setDialogOpen] = useState(false);
  const [routePath, setRoutePath] = useState(null);
  const [directionsResult, setDirectionsResult] = useState(null);

  const {isLoaded} = useLoadScript({
    googleMapsApiKey: import.meta.env.VITE_GOOGLE_MAPS_API_KEY
  });

  useEffect(() => {
    const q = query(
      collection(db, "gps_data"),
      orderBy("timestamp", "desc"),
      limit(1)
    );

    const unsubscribe = onSnapshot(q, (snapshot) => {
      if (!snapshot.empty) {
        const d = snapshot.docs[0].data();
        setPosition({ lat: d.latitude, lng: d.longitude });
      }
    });
    return () => unsubscribe();
  }, [robotID]);

  async function writeGeofence(isEnabled, center, radius) {
    const ref = doc(db, "geofence", "global");
    await setDoc(ref, {
      enabled: Boolean(isEnabled),
      center: { latitude: center.lat, longitude: center.lng },
      radius,
      timestamp: new Date()
    });
  }

  const handleToggleSwitch = async (checked) => {
    if (checked) {
      const center = { lat: position.lat, lng: position.lng };
      setLockedCenter(center);
      setIsRadiusOn(true);
      await writeGeofence(true, center, mapRadius);
    } else {
      setIsRadiusOn(false);
      setLockedCenter(null);
      await writeGeofence(false, { lat: position.lat, lng: position.lng }, mapRadius);
    }
  };

  useEffect(() => {
    if (isRadiusOn && lockedCenter) {
      writeGeofence(true, lockedCenter, mapRadius);
    }
  }, [mapRadius, isRadiusOn, lockedCenter]);

  const confirmWaypoint = async (clickedLat, clickedLng) => {
    if (!pendingWaypoint) return;
    const finalWaypoint = { lat: clickedLat, lng: clickedLng };
    setWaypoint(finalWaypoint);

    try{
      // storing waypoint in firebase right now, later we can send it to robot directly maybe???
      const origin = { lat: position.lat, lng: position.lng };
      const destination = finalWaypoint;      
      const {points, directions} = await getRoutePoints(origin, destination);
      setRoutePath(points);
      setDirectionsResult(directions);

      const ref = doc(db, "waypoints", robotID);
      await setDoc(ref, {
        latitude: clickedLat,
        longitude: clickedLng,
        path: points,
        timestamp: serverTimestamp()
      });
    } catch (e) {
      console.error("Error setting waypoint or directions:", e);
    }
    setDialogOpen(false);
    setPendingWaypoint(null);
  };

  const handleCancelWaypoint = () => {
    setDialogOpen(false);
    setPendingWaypoint(null);
  };

  const handleConfirmWaypoint = async () => {
    if (!pendingWaypoint) return;
    await confirmWaypoint(pendingWaypoint.lat, pendingWaypoint.lng);
  };

  const mapClicker = async (e) => {
    const clickedLat = e.latLng.lat();
    const clickedLng = e.latLng.lng();

    setDirectionsResult(null);
    setPendingWaypoint({ lat: clickedLat, lng: clickedLng });
    setDialogOpen(true);
  };


  if (!isLoaded)
    return <div>Loading Map...!</div>;

  return (
  <Box className="maptab" sx={{ display: "flex", gap: 4, flexWrap: "wrap", justifyContent: "flex-start", alignItems: "center", ml: -2}}>
    <GlobalStyles styles={`
      .maptab .switch { position: relative; display: inline-block; width: 52px; height: 30px; margin-left: 10px; }
      .maptab .switch input { opacity: 0; width: 0; height: 0; }
      .maptab .slider { position: absolute; cursor: pointer; inset: 0; background-color: #ccc; transition: .2s; border-radius: 999px; }
      .maptab .slider:before { position: absolute; content: ""; height: 24px; width: 24px; left: 3px; top: 3px; background-color: white; transition: .2s; border-radius: 50%; box-shadow: 0 1px 3px rgba(0,0,0,0.3); }
      .maptab input:checked + .slider { background-color: #c61919ff; }
      .maptab input:checked + .slider:before { transform: translateX(22px); }
      .maptab .radiusControls { margin-top: 12px; display: flex; gap: 14px; align-items: center; justify-content: center; flex-wrap: wrap; }
      .maptab .radiusInput { width: 80px; padding: 6px 10px; border-radius: 100px; border: 1px solid #d0d0d0; text-align: center; font-size: 16px; }
      .maptab .labelText { font-weight: 600; }
      .maptab .hint { font-size: 16px; color: #000000ff; margin-left: 4px; }
      .maptab .lockNote { font-size: 12px; color: #333; margin-top: 6px; }
    `} />

    <GoogleMap
      mapContainerStyle={{ width: 600, height: 400, borderRadius: 10, overflow: "hidden" }}
      center={position}
      zoom={15}
      onClick={mapClicker}
      options ={{clickableIcons: false, streetViewControl: false, draggableCursor: "pointer", }}
    >
      <GMarker position={position} />
      {waypoint && (
        <GMarker position={waypoint}
        label="W" />
      )}
      {isRadiusOn && lockedCenter && (
        <GCircle
          center={lockedCenter}
          radius={mapRadius}
          />
      )}
      {directionsResult && (
        <DirectionsRenderer
          directions={directionsResult}
          options={{
            suppressMarkers: true,
            polylineOptions: {
              strokeColor: "#FF0000",
              strokeOpacity: 0.8,
              strokeWeight: 4,
            }
          }}
        />
      )}  
    </GoogleMap>

   

    <Box sx={{ minWidth: 280 }}>
      <Typography variant="h6" sx={{ mb: 1 }}>Location & Geofence</Typography>
      <div><strong>Current Location:</strong>&nbsp;
        {position.lat?.toFixed(5)}, {position.lng?.toFixed(5)}
      </div>

    {waypoint && (
      <div><strong>Waypoint Set To:</strong>&nbsp;
      {waypoint.lat.toFixed(5)}, {waypoint.lng.toFixed(5)}
      </div>
    )}


      <div className="radiusControls">
        <span className="labelText">Search&nbsp;Radius:</span>
        <input
          className="radiusInput"
          type="number"
          inputMode="numeric"
          min={5}
          max={100}
          step={1}
          value={radiusInput}
          onChange={(e) => setRadiusInput(e.target.value)}
          onBlur={(e) => setRadiusInput(String(clampRadius(Number(e.target.value))))}
          placeholder="meters"
        />
        <span className="hint"><strong>meters</strong></span>

        <label className="switch" title="Toggle search radius">
          <input
            type="checkbox"
            checked={isRadiusOn}
            onChange={(e) => handleToggleSwitch(e.target.checked)}
          />
          <span className="slider" />
        </label>
      </div>

      <BackButton path="/HomePage" />
      <Dialog open={dialogOpen} onClose={handleCancelWaypoint}>
        <DialogTitle>Set Waypoint?</DialogTitle>
        <DialogContent sx={{ pt: 2 }}>
          <Typography>
            Send robot to: <br />
            <strong>
              {pendingWaypoint?.lat.toFixed(5)}, {pendingWaypoint?.lng.toFixed(5)}
            </strong>
          </Typography>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleConfirmWaypoint} variant="contained" color="primary">
            Confirm
          </Button>
          <Button onClick={handleCancelWaypoint} color="inherit">
            Cancel
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  </Box>
  );
}
export { MapTab, getRoutePoints, ControlTab };