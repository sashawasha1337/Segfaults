const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const nmea = require('nmea-simple');
const { initializeApp } = require('firebase/app');
const { getFirestore, collection, addDoc } = require('firebase/firestore');

const serialPortPath = '/dev/tty.usbserial-10'; // For Raspberry Pi or Jetson Nano use = '/dev/ttyUSB0'
const port = new SerialPort({ path: serialPortPath, baudRate: 9600 });
const lineParser = port.pipe(new ReadlineParser({ delimiter: '\r\n' }));

const firebaseConfig = {
  apiKey: import.meta.env.VITE_FIREBASE_API_KEY,
  authDomain: import.meta.env.VITE_FIREBASE_AUTH_DOMAIN,
  projectId: import.meta.env.VITE_FIREBASE_PROJECT_ID,
  storageBucket: import.meta.env.VITE_FIREBASE_STORAGE_BUCKET,
  messagingSenderId: import.meta.env.VITE_FIREBASE_MESSAGING_SENDER_ID,
  appId: import.meta.env.VITE_FIREBASE_APP_ID
};
initializeApp(firebaseConfig);
const db = getFirestore();

const ROBOT_ID = "ugv1";
let lastLat = null
let lastLng = null;

lineParser.on('data', async (line) => {
  try {
    const packet = nmea.parseNmeaSentence(line);
    if (packet.sentenceId === "RMC" && packet.status === "valid") {
      const latitude = packet.latitude;
      const longitude = packet.longitude;
      if (lastLat === null || Math.abs(latitude - lastLat) > 0.000005 || Math.abs(longitude - lastLng) > 0.000005) {
        lastLat  = latitude;
        lastLng = longitude;
      const now = new Date();
      const options = { hour: '2-digit', minute: '2-digit', hour12: false };
      const timestamp = now.toLocaleTimeString('en-US', options);

      console.log(`GPS fix received at: lat=${latitude}, lng=${longitude}`);

      const dataPoint = { 
        robotId: ROBOT_ID, 
        latitude: latitude, 
        longitude: longitude, 
        timestamp: timestamp 
      };
      await addDoc(collection(db, "gpsData"), dataPoint);
      console.log(`Uploaded GPS data for ${ROBOT_ID} to Firebase:`, dataPoint);
    }
    else {
      console.log(`${ROBOT_ID} has not moved`);
    }
  }
  } catch (error) {
      console.error("Failed to parse line:", line, error);
  }
});

console.log(`Listening for GPS data on serial port ${serialPortPath} as ${ROBOT_ID} ...`);
