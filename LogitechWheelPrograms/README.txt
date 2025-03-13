# QCar Data Collection and Camera Processing System

## Overview

This project is designed to facilitate the operation, control, and data collection of a Quanser QCar robotic vehicle. It integrates real-time image processing, manual driving control, and data logging through multiple programs that communicate via UDP sockets and Pygame-based control interfaces.

### **Programs Included:**

1. **Cameras.py** - Handles live camera feed viewing and preprocessing, including lane detection.
2. **DataCollectionServer.py** - Manages QCar movement, receives control inputs via a UDP socket, and logs sensor and image data.
3. **payload.py** - Defines a payload handler for processing data packets received from the QCar's control system.
4. **TestDriveServer.py** - Provides a UDP-based remote driving interface with real-time gear shifting, throttle, and steering adjustments.

These scripts work together to provide a robust system for manual driving, real-time monitoring, and structured data collection.

---

## File Descriptions

### **1. Cameras.py**

This script is responsible for handling the QCar's camera feeds, detecting lanes in real-time, and displaying the processed images.

#### **Key Features:**

- Captures live camera feeds from multiple cameras (front, back, left, right, and depth sensors).
- Implements **lane detection** using edge detection (Canny) and Hough Line Transform.
- Utilizes **region of interest (ROI) filtering** to focus lane detection on the road.
- Provides real-time camera preview windows for visual feedback.
- Supports depth camera processing and visualization.

#### **How to Run:**

```bash
py3 Cameras.py
```

- Press `ESC` to exit the camera preview.

---

### **2. DataCollectionServer.py**

This script controls the QCar's movement based on external control inputs received over a UDP socket. It captures and logs images along with steering and throttle data.

#### **Key Features:**

- Listens for control commands over **UDP socket communication**.
- Receives and processes **throttle and steering inputs**.
- Logs **image file names, timestamps, and corresponding control values**.
- Implements **braking and gear shift logic** based on controller inputs.
- Saves images and metadata for later machine learning or analysis.

#### **How to Run:**

```bash
py3 DataCollectionServer.py
```

- This will start the server, waiting for incoming UDP commands and logging data accordingly.

---

### **3. payload.py**

This script defines a **payload handler** for processing control data packets sent to the QCar.

#### **Key Features:**

- Handles **controller ID, event type, event dimensions, and event values**.
- Supports **packet serialization and deserialization**.
- Ensures data integrity and prevents errors due to incorrect packet sizes.
- Provides debugging tools to inspect message contents.

#### **How to Run (for testing):**

```bash
py3 payload.py
```

- This will run a test sequence for reading and writing payload data.

---

### **4. TestDriveServer.py**

This script allows for remote control of the QCar via UDP communication, enabling real-time driving adjustments based on received commands.

#### **Key Features:**

- Supports **real-time throttle and steering control** based on incoming UDP packets.
- Implements **gear shifting logic** for multiple driving modes.
- Provides **camera preview functionality** for real-time feedback.
- Monitors **Wi-Fi networks** for connectivity status.
- Ensures **safe driving operation** with structured braking and acceleration logic.

#### **How to Run:**

```bash
py3 TestDriveServer.py
```

- This will start the server, listening for remote control commands.

---

## **How These Programs Work Together**

### **System Workflow:**

1. **Cameras.py** is responsible for capturing and preprocessing image data, displaying real-time lane detection.
2. **DataCollectionServer.py** listens for control commands, manages QCar movement, and logs images along with throttle and steering data.
3. **payload.py** handles structured communication between the control system and QCar, ensuring commands are properly interpreted.
4. **TestDriveServer.py** enables remote driving functionality using UDP-based commands and gear shifting logic.

---

## **Dependencies**

Ensure the following dependencies are installed before running any script:

```bash
pip install pygame numpy opencv-python
```

---

## **Authors**

- Alex Sanna ([ajsanna@cpp.edu](mailto\:ajsanna@cpp.edu))
- Matthew Baldivino ([mabaldivino@cpp.edu](mailto\:mabaldivino@cpp.edu))
- Reyna Nava ([nvnava@cpp.edu](mailto\:nvnava@cpp.edu))

**Last Updated:** February 2025

