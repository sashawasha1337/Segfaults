

# Segfaults - Trash Finding Robot Application

Welcome to the **Segfaults Project**, a senior project developed at Sacramento State. This repository contains the codebase for a web application and robot scripts that together power an innovative trash-finding robot. The project demonstrates the integration of robotics, real-time data processing, and a user-friendly web interface to tackle waste management challenges.

---

## 🚀 Project Overview

The **Segfaults** project features:

1. **Trash-Finding Robot**: A robot equipped with advanced sensors and AI to detect, locate, and respond to trash in its environment.
2. **Web Application**: A responsive and intuitive interface that enables users to:
   - Monitor the robot's real-time status and location.
   - Control its movements remotely.
   - Visualize trash detection on a map.
   - Manage and analyze collected data.

This project combines robotics, computer vision, and cloud technologies to create a solution for efficient waste management.

---

## ✨ Key Features

### Web App
- **Real-Time Monitoring**: Track the robot's status, location, and trash detection events.
- **Remote Control**: Navigate the robot via an intuitive React-based interface.
- **Trash Visualization**: Display detected trash locations using Leaflet map integration.
- **User Authentication**: Secure access through Firebase authentication.
- **Data Management**: Store and retrieve robot data using Firestore.

### Robot Scripts
- **Camera Integration**: Use OpenCV and ROS to capture and process images.
- **Object Detection**: Implement YOLO-based AI for detecting trash in the robot's environment.
- **Navigation**:
  - GPS-based movement tracking using `gpsReaderScript.cjs`.
  - Lidar-based obstacle avoidance with `avoid_obstacles.py`.
- **Motor Control**: Control DC motors using ROS and GPIO in `MotorNode.py`.
- **Cloud Connectivity**: Stream data to Firebase for real-time updates.

---

## 🛠️ Technologies Used

### Web Application
- **Frontend**:
  - React
  - Leaflet (for maps)
  - Firebase Authentication
- **Backend**:
  - Firebase Firestore (for data storage)

### Robot Scripts
- **Languages**:
  - Python
  - JavaScript
- **Libraries & Frameworks**:
  - ROS2 (Robot Operating System)
  - OpenCV
  - YOLO (Ultralytics)
  - Flask (for communication)
  - Serialport (for GPS data)

---
## 🖥️ How to Set Up and Run

### Prerequisites

#### Web App 
- [Tailscale](https://tailscale.com/)
- [Firebase](https://firebase.google.com/)

#### Robot
- [Flask](https://flask.palletsprojects.com/en/stable/)
- [ROS2](https://www.ros.org/) (Robot Operating System)

### Installation

#### Authentication 
1. Setup your Tailscale VPN
   - Add Users
   - Add Robots with SSH enabled
  
2. Setup Firebase Authenetication and Database
   - Create Firebase account
   - Configure Cloud Firestore
   - Configure Firestore Authentication
    
#### Robot Scripts
1. Clone the repository:
   ```bash
   git clone https://github.com/sashawasha1337/Segfaults.git
   ```

1. Navigate to the `RobotScripts` folder:
   ```bash
   cd Segfaults/RobotScripts
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. For ROS2 nodes, source the ROS2 environment:
   ```bash
   source /opt/ros/foxy/setup.bash
   ```

4. Run individual scripts as needed, e.g., for the LiDAR node:
   ```bash
   ros2 run RobotScripts avoid_obstacles.py
   ```

#### Web Application
1. Clone the repository:
   ```bash
   git clone https://github.com/sashawasha1337/Segfaults.git
   cd Segfaults/ReactApp
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Run the development server with Vite:
   ```bash
   npm run-dev
   ```

4. Open your browser and navigate to `http://localhost:3000`.


---

## 🗓️ Timeline

---

## 📜 License

This project is licensed under the [MIT License](LICENSE). Feel free to use, modify, and distribute this project as you see fit.

---

## 📧 Contact

For inquiries or feedback, reach out to the project team at **[your-email@example.com]**.

---

Would you like any additional sections (e.g., acknowledgments, troubleshooting tips)? Let me know if you'd like further refinements!
