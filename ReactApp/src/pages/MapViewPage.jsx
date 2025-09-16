import { useState, useEffect } from "react";
import BackButton from "../components/BackButton";
import { MapContainer, TileLayer, Marker, Popup, useMap, Circle } from "react-leaflet";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import { collection, query, doc, setDoc, limit, onSnapshot, orderBy, where } from "firebase/firestore";
import { db } from "../firebaseConfig";

const defaultCenter = [38.56080, -121.42400];
const ROBOT_ID = "ugv1";

const ugvIcon = L.icon({
  iconUrl: "https://cdn.pixabay.com/photo/2013/07/12/13/43/arrow-147174_1280.png",       
  iconSize: [14, 14],
  iconAnchor: [7, 7],
  popupAnchor: [0, -7]
});

function AutoRecenterMap({ lat, lng }) {
  const map = useMap();
  useEffect(() => {
    map.setView([lat, lng]);
  }, [lat, lng, map]);
  return null;
}

function clampRadius(meters) {
  if (Number.isNaN(meters)) {
    return 5;
  }
  return Math.max(5, Math.min(100, meters));
}

function MapView() {
  const [position, setPosition] = useState({ lat: defaultCenter[0], lng: defaultCenter[1] });
  const [isRadiusOn, setIsRadiusOn] = useState(false);
  const [radiusInput, setRadiusInput] = useState("25");
  const [lockedCenter, setLockedCenter] = useState(null);
  const mapRadius = clampRadius(Number(radiusInput));

  useEffect(() => {
    const q = query(
      collection(db, "gps_data"),
      where("robotId", "==", ROBOT_ID),
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
  }, []);

  async function writeGeofence(isEnabled, center, radius) {
    const ref = doc(db, "geofence", ROBOT_ID);
    await setDoc(ref, {
      robotId: ROBOT_ID,
      enabled: Boolean(isEnabled),
      center: {latitude: center.lat, longitude: center.lng},
      radius: radius,
      timestamp: new Date()
    })
  }

  const handleToggleSwitch = async (checked) => {
    if (checked) {
      setLockedCenter({ lat: position.lat, lng: position.lng });
      setIsRadiusOn(true);
      await writeGeofence(true, { lat: position.lat, lng: position.lng }, mapRadius);
    } else {
      setIsRadiusOn(false);
      setLockedCenter(null);
      await writeGeofence(false, { lat: position.lat, lng: position.lng }, mapRadius );
    }
  };

  useEffect(() => {
    if (isRadiusOn && lockedCenter) {
      writeGeofence(true, lockedCenter, mapRadius);
    }
  }, [mapRadius, isRadiusOn, lockedCenter]);

  return (
    <div style={{ display: "flex", width: "100vw", height: "100vh" }}>
      <style>{`
        .switch { position: relative; display: inline-block; width: 52px; height: 30px; margin-left: 10px; }
        .switch input { opacity: 0; width: 0; height: 0; }
        .slider { position: absolute; cursor: pointer; inset: 0; background-color: #ccc; transition: .2s; border-radius: 999px; }
        .slider:before { position: absolute; content: ""; height: 24px; width: 24px; left: 3px; top: 3px; background-color: white; transition: .2s; border-radius: 50%; box-shadow: 0 1px 3px rgba(0,0,0,0.3); }
        input:checked + .slider { background-color: #c61919ff; }
        input:checked + .slider:before { transform: translateX(22px); }
        .radiusControls { margin-top: 12px; display: flex; gap: 14px; align-items: center; justifyContent: center; flexWrap: wrap; }
        .radiusInput { width: 80px; padding: 6px 10px; border-radius: 100px; border: 1px solid #d0d0d0; text-align: center; font-size: 16px; }
        .labelText { font-weight: 600; }
        .hint { font-size: 16px; color: #000000ff; margin-left: 4px; }
        .lockNote { font-size: 12px; color: #333; margin-top: 6px; }
      `}</style>

      <MapContainer
        center={[position.lat, position.lng]}
        zoom={15}
        style={{ width: "30%", height: "40%", borderRadius: "10px", overflow: "hidden" }}
      >
        <TileLayer
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          attribution="&copy; OpenStreetMap contributors"
        />
        {position && (
        <>
          <AutoRecenterMap lat={position.lat} lng={position.lng} />
          <Marker position={[position.lat, position.lng]} icon={ugvIcon}>
            <Popup>
              <div style={{ textAlign: "center" }}><strong>{ROBOT_ID}</strong></div>
            </Popup>
          </Marker>
          {isRadiusOn && lockedCenter && (
            <Circle
              center={[lockedCenter.lat, lockedCenter.lng]}
              radius={mapRadius}
              pathOptions={{ color: "blue", weight: 1.5, fillOpacity: 0.05 }}
            />
          )}
        </>
        )}
      </MapContainer>

      <BackButton path="/ActivityLogPage"/>

      <div style={{ padding: "100px", textAlign: "center", marginTop: 20 }}>
        <strong>Coordinates:</strong> Latitude: {position.lat?.toFixed(5)}, Longitude: {position.lng?.toFixed(5)}
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
      </div>
    </div>
  );
};

export default MapView;
