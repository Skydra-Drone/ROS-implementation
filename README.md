# Scout Drone ROS Implementation

A comprehensive ROS-based suite for drone-based perception, sensor fusion, and geolocation. This repository implements a "Scout" drone system capable of detecting targets (YOLOv8), thermal sensing, and precise ground-coordinate calculation.

---

## 🚀 Key Modules

### 1. Perception & YOLO Detection
Implements a real-time object detection pipeline.
- **Camera Node**: Publishes raw BGR images from webcams, IP cameras, or simulated video files to `/camera/image_raw`.
- **YOLO Detector**: Subscribes to images and performs inference using **YOLOv8**. It publishes structured `YoloDetectionArray` messages containing bounding boxes and confidence scores.
- **Optimization**: Supports CUDA acceleration for low-latency detection on hardware like Jetson Nano.

### 2. Geolocation Engine
The "Mission Brain" that calculates real-world GPS coordinates of detected targets.
- **Sensor Fusion**: Uses `ApproximateTimeSynchronizer` to align YOLO detections with Drone GPS (Lat/Lon), IMU (Pitch/Roll/Yaw), and LiDAR (Altitude).
- **Coordinate Projection**: Projects 3D rays from the camera frame onto the ground plane using camera intrinsic parameters and trigonometric ground-intersection math.
- **Global Mapping**: Converts local North/East meter offsets into global Latitude/Longitude using the `pyproj` library.

### 3. Multi-Sensor Integration (Lidar/Camera/Thermal)
Core implementation for physical and simulated sensor nodes.
- **High-Fidelity Simulation**: Includes fallback patterns for testing without hardware:
  - **Thermal**: Simulated Gaussian heat spots with `COLORMAP_JET` visualization.
  - **Camera**: Animated gradient test patterns with crosshair overlays.
  - **LiDAR**: Hovering altitude simulation with sinusoidal noise.
- **Hardware Drivers**: Pre-built support for TFMini Plus (Serial), CSI cameras, and standard UVC thermal sensors.

---

## 🛠 Setup & Installation

### Prerequisites
- **ROS Noetic** (Ubuntu 20.04 / WSL2)
- **Python Dependencies**:
  ```bash
  pip install ultralytics pyproj scipy opencv-python cv_bridge
  ```

### Build Workspace
```bash
# Clone the repository into your catkin workspace source folder
cd ~/catkin_ws/src
# (Paste repository here)

# Build
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 🚦 How to Run

### Testing with Simulation (No Hardware)
The project includes a `test_perception.launch` file that simulates a drone flying while running the full perception pipeline.

1. **Start Master**: `roscore`
2. **Launch System**: `roslaunch scout_nodes test_perception.launch`
3. **Monitor Output**:
   ```bash
   # View final calculated target GPS coordinates
   rostopic echo /mission/target_coordinates
   
   # View camera/thermal streams
   rosrun rqt_image_view rqt_image_view
   ```

### Real Hardware Deployment
Modify the `CAMERA_SOURCE` or `LIDAR_PORT` parameters in the respective node scripts to point to your device paths (e.g., `/dev/video0` or `COM3`).

---

## 📂 Repository Structure
- `camera_yolo/`: YOLOv8 node and custom message definitions.
- `geolocation/`: The core math engine for target positioning and fake drone data publishers.
- `Lidar-Camera-thermal/`: Implementation of the physical/simulated sensor nodes.
- `remaining.md`: Design docs for future Decision and Communication nodes.

## 🔮 Future Roadmap
- **Decision Node**: Autonomous grid-search patterns.
- **Comm Gateway**: Secure telemetry bridge between Drone and Base Station.
- **Delivery Commander**: MAVLink-based payload release for the secondary drone.
