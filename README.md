# 🚗 Qcar - Autonomous Vehicle Control System

[![Python](https://img.shields.io/badge/Python-3.x-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Unlicense-lightgrey.svg)](https://unlicense.org/)

Qcar is a comprehensive autonomous vehicle control system that integrates various components including LiDAR processing, computer vision, and control systems for autonomous navigation.

## 📋 Table of Contents

- [Project Overview](#project-overview)
- [Directory Structure](#directory-structure)
- [Key Features](#key-features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [Contact](#contact)

## 🎯 Project Overview

This project implements an autonomous vehicle control system with the following key components:

- 🔍 LiDAR data processing and visualization
- 👁️ Computer vision and deep learning models for object detection
- 🛣️ Autonomous lane following and navigation
- 🎮 Hardware integration with Logitech steering wheels and keyboards
- 🔄 Real-time sensor fusion and processing

## 📁 Directory Structure

```
Qcar/
├── Demos/
│   ├── lidarThread.py
│   ├── testingGPU.py
│   ├── TestFusion.py
│   ├── LidarAndModel.py
│   ├── AutonomousLaneLoop.py
│   └── Models/
├── LogitechWheelPrograms/
├── KeyboardPrograms/
└── LidarStuff/
```

### Demos/
- `lidarThread.py`: LiDAR data processing and visualization
- `testingGPU.py`: GPU acceleration testing
- `TestFusion.py`: Sensor fusion testing
- `LidarAndModel.py`: Integration of LiDAR with deep learning models
- `AutonomousLaneLoop.py`: Autonomous lane following implementation
- `Models/`: Contains trained deep learning models

### Other Directories
- `LogitechWheelPrograms/`: Programs for Logitech steering wheel integration
- `KeyboardPrograms/`: Keyboard control programs
- `LidarStuff/`: LiDAR-related processing and utilities

## ✨ Key Features

- ⚡ Real-time LiDAR data processing and visualization
- 🚀 GPU-accelerated computer vision processing
- 🤖 Autonomous lane following capabilities
- 🎮 Integration with Logitech steering wheels for manual control
- 🔄 Sensor fusion for robust perception
- 🧠 Deep learning model integration for object detection

## 📦 Requirements

- Python 3.x
- PyTorch
- OpenCV
- NumPy
- Matplotlib
- Logitech SDK (for wheel integration)
- LiDAR drivers and SDK

## 🛠️ Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/Qcar.git
cd Qcar
```

2. Install required dependencies:
```bash
pip install -r requirements.txt
```

3. Install Logitech SDK (if using wheel control)
4. Install LiDAR drivers and SDK

## 🚀 Usage

### Running Demos

1. LiDAR Visualization:
```bash
python Demos/lidarThread.py
```

2. Autonomous Lane Following:
```bash
python Demos/AutonomousLaneLoop.py
```

3. Testing GPU Acceleration:
```bash
python Demos/testingGPU.py
```

### Control Options

- 🎮 Use Logitech steering wheel for manual control
- ⌨️ Keyboard control options available
- 🤖 Autonomous mode with lane following

## 🤝 Contributing

Contributions are not supported. This repository serves as an archive for our work in 2023-2025.

## 📞 Contact

With any questions or concerns regarding this project, to be placed in contact with the current project lead please reach out to Professor Wang at CPP: yunshengwang@cpp.edu 