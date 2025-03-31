import React, { useState, useEffect } from "react";
import { MapContainer, TileLayer, Marker, Popup, useMap } from "react-leaflet";
import L, { latLng } from "leaflet";
import "leaflet/dist/leaflet.css";
import { collection, query, limit, doc, onSnapshot, getDocs, orderBy } from "firebase/firestore";
import { db } from "../firebaseConfig";
import { data } from "react-router-dom";

const defaultCenter = [38.5608, -121.4240];

const ugvIcon = L.icon({
  iconUrl: "https://cdn.pixabay.com/photo/2013/07/12/13/43/arrow-147174_1280.png",       
  iconSize: [16, 16],
  iconAnchor: [8, 16],
  popupAnchor: [0, -14]
});

function AutoRecenterMap({ lat, lng }) {
  const map = useMap();
  useEffect(() => {
    map.setView([lat, lng]);
  }, [lat, lng, map]);
  return null;
}

function MapView() {
  const [position, setPosition] = useState({ lat: defaultCenter[0], lng: defaultCenter[1] });

  useEffect(() => {
    const docRef = doc(db, 'ugv_location', 'KXyXiopjNGAg8LV8WoNF');

    const unsubscribe = onSnapshot(docRef, (docSnap) => {
      if (docSnap.exists()) {
        const data = docSnap.data();
        if (data.location) {
          setPosition({ lat: data.location.latitude, lng: data.location.longitude });
        }
      } else {
        console.warn("UGV location document not found");
      }
    }, (error) => {
      console.error("Error fetching UGV location:", error);
    });
    
    return () => unsubscribe();
  }, []);

  return (
    <div style={{ display: "flex", width: "100vw", height: "100vh" }}>
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
              <div style={{ textAlign: "center" }}>UGV</div>
            </Popup>
          </Marker>
          </>
        )}
      </MapContainer>
      <div style={{ padding: "100px", textAlign: "center", marginTop: 20 }}>
        <strong>Coordinates:</strong> Latitude: {position.lat}, Longitude: {position.lng}
      </div>
    </div>
  );

}

export default MapView;
