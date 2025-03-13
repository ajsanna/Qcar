# QCar Manual Control, Data Collection, and Camera Processing System

## Overview

This project provides a complete system for controlling, monitoring, and collecting data from a QCar robotic vehicle using the Pygame library and integrated camera sensors. The system consists of three primary programs:

1. **Data\_Collection.py** - Handles manual driving, sensor monitoring, and data collection, including camera image storage and depth sensor recording.
2. **Cameras.py** - Processes and displays camera feeds in real time while implementing lane detection and preprocessing.
3. **Test\_Drive.py** - Provides a simplified manual control interface for driving the QCar with keyboard inputs.

These scripts work together to facilitate real-time QCar operation, data collection, and processing for machine learning or autonomous driving research.

---

## File Descriptions

### **1. Data\_Collection.py**

This script allows for manual driving of the QCar while capturing data from multiple cameras, including RGB and depth sensors. The captured images are saved with associated throttle and steering data for later analysis.

#### **Key Features:**

- Provides a **manual control interface** using Pygame.
- Supports **multi-camera data collection** (front, back, left, right, and depth cameras).
- Saves images and depth maps to storage with timestamps and sequential indexing.
- Logs metadata such as throttle, steering angle, and image file paths.
- Manages turn signals, headlights, and reverse mode using the QCar’s LED system.
- Ensures smooth acceleration and deceleration for better driving experience.
- Uses a **catalog file** to track image names and associated driving parameters.
- Supports **time-based execution**, allowing the program to run for a specific duration.

#### **How to Run:**

```bash
py3 Data_Collection.py [depth]
```

- If the `depth` argument is included, depth images will also be captured.
- The captured images and metadata will be stored in a dedicated folder on the external drive or local storage.

---

### **2. Cameras.py**

This script processes camera feeds in real-time, applying preprocessing steps such as lane detection. It provides a live view of multiple cameras and allows for frame analysis.

#### **Key Features:**

- Displays **live camera feeds** from multiple angles (front, back, left, right, and depth sensors).
- Applies **lane detection** using edge detection and Hough Line Transform.
- Uses a **region of interest (ROI) method** to remove irrelevant sections from images.
- Processes images in **HSV color space** to enhance lane markings.
- Supports **real-time edge detection and object filtering**.

#### **How to Run:**

```bash
py3 Cameras.py
```

- This will display live feeds from all available cameras.
- Press `ESC` to exit.

---

### **3. Test\_Drive.py**

This script is a simplified manual driving program that allows users to control the QCar without data collection. It focuses on real-time driving control and LED feedback.

#### **Key Features:**

- Provides a **manual driving interface** with keyboard controls.
- Uses **Pygame for event handling** and QCar movement updates.
- Supports **forward and reverse driving** with smooth acceleration/deceleration.
- Controls **LED indicators** for turn signals, headlights, and reverse mode.
- Runs two separate **threads** for driving control and camera preview.
- Ensures **safe termination** using a keyboard interrupt handler.

#### **How to Run:**

```bash
py3 Test_Drive.py
```

- This opens a control window and allows manual driving via keyboard.

---

## How These Programs Work Together

### **System Workflow:**

1. **Test\_Drive.py** can be used to manually operate the QCar for simple movement testing.
2. **Data\_Collection.py** expands on this by allowing data capture while driving, logging images and driving parameters.
3. **Cameras.py** processes live camera feeds, providing visual feedback and lane detection, which can be used to improve navigation algorithms.

---

## **Keyboard Controls (Applicable to Data\_Collection.py and Test\_Drive.py)**

| Key | Action                 |
| --- | ---------------------- |
| W   | Accelerate             |
| S   | Decelerate/Brake       |
| A   | Steer Left             |
| D   | Steer Right            |
| R   | Engage Reverse Mode    |
| F   | Disengage Reverse Mode |
| Q   | Stop Immediately       |
| H   | Toggle Headlights      |

---

## **Data Storage & Logging**

- **Images are saved** in timestamped folders under `/media/SANDISK1/Images/`.
- **Depth images** are saved as `.npy` files under `/media/378B-14FD/Depth_Images/`.
- **Image metadata** (throttle, steering, image filenames) is logged in `images_catalogs.txt`.
- **Index tracking** is maintained in `index_tracker.txt` to ensure sequential numbering of images.

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
- Sebastian Cursaro ([scursaro@cpp.edu](mailto\:scursaro@cpp.edu))
- Joseph Bui ([jhbui@cpp.edu](mailto\:jhbui@cpp.edu))
- Reyna Nava ([nvnava@cpp.edu](mailto\:nvnava@cpp.edu))
- Rich Chea ([richweichea@cpp.edu](mailto\:richweichea@cpp.edu))

**Last Updated:** February 2025

