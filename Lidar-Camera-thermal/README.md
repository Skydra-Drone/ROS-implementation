# Extracted ROS Scripts

This directory contains all the code extracted from the main README for easy copy-paste and repository setup.

## Directory Structure

```
extracted_scripts/
├── camera_node.py              # Camera publisher with simulation mode
├── thermal_camera_node.py      # Thermal camera publisher with simulated heat signatures
├── lidar_node.py               # LiDAR publisher with TFMini Plus support
├── TargetPosition.msg          # Custom message definition for GPS coordinates
├── drone_msgs_package.xml      # Package.xml for drone_msgs package
├── drone_msgs_CMakeLists.txt   # CMakeLists.txt reference for drone_msgs package
├── sensors.launch              # Launch file to run all sensor nodes
└── README.md                   # This file
```

## Usage

### For Node Scripts (Python files)
1. Copy the `.py` files to your ROS package's `nodes/` directory
2. Make them executable: `chmod +x *.py`

### For Message Files
1. Copy `TargetPosition.msg` to your `drone_msgs/msg/` directory
2. Copy `drone_msgs_package.xml` content to `drone_msgs/package.xml`
3. Use `drone_msgs_CMakeLists.txt` as a reference to update your `drone_msgs/CMakeLists.txt`

### For Launch Files
1. Copy `sensors.launch` to your package's `launch/` directory

## Configuration

Each node has a configuration section at the top where you can switch between simulation and hardware modes:

**Camera Node:**
```python
CAMERA_SOURCE = None  # None for simulation, 0 for webcam, "http://IP:8080/video" for IP camera
SIMULATION_MODE = (CAMERA_SOURCE is None)
```

**Thermal Camera Node:**
```python
CAMERA_SOURCE = None  # None for simulation, device path for real thermal camera
SIMULATION_MODE = (CAMERA_SOURCE is None)
```

**LiDAR Node:**
```python
LIDAR_PORT = None  # None for simulation, "/dev/ttyUSB0" for real LiDAR
SIMULATION_MODE = (LIDAR_PORT is None)
```

## Quick Start

1. **Copy files to your ROS workspace:**
   ```bash
   cp camera_node.py ~/drone_ws/src/scout_drone/nodes/
   cp thermal_camera_node.py ~/drone_ws/src/scout_drone/nodes/
   cp lidar_node.py ~/drone_ws/src/scout_drone/nodes/
   cp sensors.launch ~/drone_ws/src/scout_drone/launch/
   chmod +x ~/drone_ws/src/scout_drone/nodes/*.py
   ```

2. **Build your workspace:**
   ```bash
   cd ~/drone_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Launch all sensors:**
   ```bash
   roslaunch scout_drone sensors.launch
   ```

## Notes

- All scripts default to **simulation mode** for easy testing without hardware
- Scripts include automatic reconnection logic for hardware failures
- Each node publishes to standard ROS topics that can be visualized with `rqt_image_view` or `rostopic echo`

For detailed setup instructions, refer to the main README in the parent directory.
