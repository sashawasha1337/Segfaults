

# Segfaults - Trash Finding Robot Application

![image](Logo.png)

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
- **Camera Integration**: Integrates the onboard camera with the robot's single-board computer to enable the capturing and processing of videos.
- **Object Detection**: Utilizes the YOLO algorithm to scan a live feed video for litter in the robot's environment.
- **Navigation**:
  - Motor scripts are used to move the robot.
  - Autonomous obstacle avoidance is implemented through data received from a Lidar.
- **Movement Tracking**: Utilizes a GPS module to keep track of the robot's location.
- **Cloud Connectivity**: Live video feed data is streamed to a website for real-time updates.

---

## 🛠️ Technologies Used

### Web Application
- **Frontend**:
  - React + Vite
  - Leaflet (for maps)
  - Firebase Authentication
- **Backend**:
  - Firebase Firestore (for data storage)

### Robot Scripts
- **Languages**:
  - Python
- **Libraries & Frameworks**:
  - ROS2 (Robot Operating System)
  - WebRTC
  - OpenCV
  - YOLO (for Image Detection)
  - Flask (for communication with robot and app)
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
  
2. Setup Firebase Authentication and Database
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

## 📅 Timeline

![timeline](timeline.png)


---

#### Data ERD 
![image](SegfaultsERD.png)

---

## 📧 Contact

For inquiries or feedback, reach out to the project team at **[your-email@example.com]**.

---

