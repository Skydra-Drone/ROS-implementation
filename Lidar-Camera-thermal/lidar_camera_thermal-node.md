

## Table of Contents
- [Prerequisites](#prerequisites)
- [Phase 1: Workspace Setup](#phase-1-workspace-setup)
- [Phase 2: Custom Messages](#phase-2-custom-messages)
- [Phase 3: Scout Drone Package](#phase-3-scout-drone-package)
- [Phase 4: Sensor Nodes Implementation](#phase-4-sensor-nodes-implementation)
- [Phase 5: Launch Configuration](#phase-5-launch-configuration)
- [Phase 6: Testing](#phase-6-testing)
- [Phase 7: IP Webcam Integration](#phase-7-ip-webcam-integration)
- [Troubleshooting](#troubleshooting)

---

## Prerequisites

**What we started with:**
- WSL2 with Ubuntu 20.04 ✓
- ROS Noetic installed ✓
- Gazebo installed ✓

---

## Phase 1: Workspace Setup

### Step 1.1: Create Catkin Workspace

Open WSL terminal (Win + R → type `wsl` → Enter) and run:

```bash
cd ~
mkdir -p ~/drone_ws/src
cd ~/drone_ws/
catkin_make
source devel/setup.bash
echo "source ~/drone_ws/devel/setup.bash" >> ~/.bashrc
```

**What this does:**
- Creates the workspace directory structure
- Initializes it as a catkin workspace
- Automatically sources it on every terminal startup

---

## Phase 2: Custom Messages

### Step 2.1: Create the drone_msgs Package

```bash
cd ~/drone_ws/src
catkin_create_pkg drone_msgs std_msgs geometry_msgs sensor_msgs rospy roscpp
mkdir -p drone_msgs/msg
```

### Step 2.2: Define TargetPosition Message

```bash
cat > ~/drone_ws/src/drone_msgs/msg/TargetPosition.msg << 'EOF'
# Custom message for target GPS coordinates
float64 latitude
float64 longitude
float64 altitude
time timestamp
string object_type
float32 confidence
EOF
```

### Step 2.3: Edit package.xml

**CRITICAL FIX:** The initial `package.xml` had XML syntax errors. We fixed it completely:

```bash
cat > ~/drone_ws/src/drone_msgs/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>drone_msgs</name>
  <version>0.0.1</version>
  <description>Custom messages for drone project</description>

  <maintainer email="user@todo.todo">Your Name</maintainer>

  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>
  
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>message_generation</build_depend>
  
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>message_runtime</exec_depend>

</package>
EOF
```

**Key additions:**
- `<build_depend>message_generation</build_depend>`
- `<exec_depend>message_runtime</exec_depend>`

### Step 2.4: Edit CMakeLists.txt

We need to configure message generation. Here are the critical sections:

**Section 1 - find_package:**
```cmake
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)
```

**Section 2 - add_message_files:**
```cmake
add_message_files(
  FILES
  TargetPosition.msg
)
```

**Section 3 - generate_messages:**
```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
```

**Section 4 - catkin_package:**
```cmake
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)
```

### Step 2.5: Build Messages

```bash
cd ~/drone_ws
catkin_make
source devel/setup.bash
```

---

## Phase 3: Scout Drone Package

### Step 3.1: Create Package

```bash
cd ~/drone_ws/src
catkin_create_pkg scout_drone rospy std_msgs sensor_msgs cv_bridge drone_msgs
mkdir -p scout_drone/nodes
mkdir -p scout_drone/launch
```

---

## Phase 4: Sensor Nodes Implementation

### Step 4.1: Camera Node

**Initial Issue:** Friend's code had syntax errors:
- Line 1: Incorrect indentation on imports
- Line 70: `_name_` and `_main_` should be `__name__` and `__main__` (double underscores)

**Final Working Code:**

```bash
cat > ~/drone_ws/src/scout_drone/nodes/camera_node.py << 'EOF'
#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

# --- CONFIG ---
# FOR REAL DRONE: CAMERA_SOURCE = "http://192.168.0.105:8080/video"
# FOR TESTING: Set to None for simulation, 0 for webcam, or path to video file
CAMERA_SOURCE = None  # Set to None for simulation
SIMULATION_MODE = (CAMERA_SOURCE is None)

def camera_publisher():
    """
    Connects to a camera source and publishes frames as ROS Image messages.
    Falls back to simulation mode if no camera is available.
    """
    # Initialize the ROS node
    rospy.init_node('camera_node', anonymous=True)

    # Create a publisher for the image topic
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

    # Create a CvBridge object
    bridge = CvBridge()

    # Set the publishing rate (30 Hz)
    rate = rospy.Rate(30)

    cap = None
    frame_count = 0

    # Determine if we're using a real camera or simulation
    if not SIMULATION_MODE:
        rospy.loginfo(f"Attempting to connect to camera source: {CAMERA_SOURCE}")
    else:
        rospy.loginfo("Running in SIMULATION mode (no physical camera)")

    while not rospy.is_shutdown():
        frame = None

        # --- Real Camera Mode ---
        if not SIMULATION_MODE:
            if cap is None or not cap.isOpened():
                rospy.loginfo(f"Connecting to camera: {CAMERA_SOURCE}")
                cap = cv2.VideoCapture(CAMERA_SOURCE)
                if not cap.isOpened():
                    rospy.logwarn("Failed to connect to camera. Retrying in 5 seconds...")
                    time.sleep(5)
                    continue
                else:
                    rospy.loginfo("Camera connected successfully.")

            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("Failed to grab frame. Reconnecting...")
                cap.release()
                cap = None
                continue

        # --- Simulation Mode ---
        else:
            frame = generate_test_image(frame_count)
            frame_count += 1

        # --- Publishing ---
        if frame is not None:
            try:
                # Convert to ROS Image message
                ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = "camera_frame"

                # Publish
                image_pub.publish(ros_image)

            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")

        rate.sleep()

    # Clean up
    if cap is not None:
        cap.release()


def generate_test_image(frame_count):
    """
    Generate a test pattern image (for testing without hardware).
    """
    width, height = 640, 480
    
    # Create a gradient test pattern
    img = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Add animated gradient
    offset = (frame_count % 255)
    for i in range(height):
        img[i, :, 0] = int((255 * i / height + offset) % 255)  # Blue channel
        img[i, :, 1] = int(255 * (1 - i / height))  # Green channel
        img[i, :, 2] = int((offset) % 255)  # Red channel (animated)
    
    # Add timestamp text
    timestamp = rospy.Time.now()
    cv2.putText(img, f"SIMULATED CAMERA FEED", 
                (150, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
    cv2.putText(img, f"Time: {timestamp.to_sec():.2f}", 
                (10, height - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(img, f"Frame: {frame_count}", 
                (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    # Add crosshair
    cv2.line(img, (width//2 - 30, height//2), 
             (width//2 + 30, height//2), (0, 255, 0), 2)
    cv2.line(img, (width//2, height//2 - 30), 
             (width//2, height//2 + 30), (0, 255, 0), 2)
    cv2.circle(img, (width//2, height//2), 50, (0, 255, 0), 2)
    
    return img


if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
EOF

chmod +x ~/drone_ws/src/scout_drone/nodes/camera_node.py
```

**Key Features Added:**
- Simulation mode with animated test pattern
- Automatic reconnection logic
- Frame counting and timestamps
- Visual crosshair for testing

### Step 4.2: Thermal Camera Node

```bash
cat > ~/drone_ws/src/scout_drone/nodes/thermal_camera_node.py << 'EOF'
#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

# --- CONFIG ---
CAMERA_SOURCE = None  # Set to None for simulation, or device path for real thermal camera
SIMULATION_MODE = (CAMERA_SOURCE is None)

# Thermal camera parameters
TEMP_MIN = 15.0  # Minimum temperature in Celsius
TEMP_MAX = 40.0  # Maximum temperature in Celsius

def thermal_camera_publisher():
    rospy.init_node('thermal_camera_node', anonymous=True)
    thermal_pub = rospy.Publisher('/thermal/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(10)
    cap = None
    frame_count = 0

    if not SIMULATION_MODE:
        rospy.loginfo(f"Attempting to connect to thermal camera: {CAMERA_SOURCE}")
    else:
        rospy.loginfo("Running in SIMULATION mode (no physical thermal camera)")

    while not rospy.is_shutdown():
        thermal_image = None

        # --- Real Camera Mode ---
        if not SIMULATION_MODE:
            if cap is None or not cap.isOpened():
                rospy.loginfo(f"Connecting to thermal camera: {CAMERA_SOURCE}")
                cap = cv2.VideoCapture(CAMERA_SOURCE)
                if not cap.isOpened():
                    rospy.logwarn("Failed to connect to thermal camera. Retrying in 5 seconds...")
                    time.sleep(5)
                    continue
                else:
                    rospy.loginfo("Thermal camera connected successfully.")

            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("Failed to grab thermal frame. Reconnecting...")
                cap.release()
                cap = None
                continue

            # Process thermal frame (apply colormap if needed)
            if len(frame.shape) == 2:  # Grayscale
                thermal_image = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            else:
                thermal_image = frame

        # --- Simulation Mode ---
        else:
            thermal_image = generate_simulated_thermal()

        # --- Publishing ---
        if thermal_image is not None:
            try:
                ros_image = bridge.cv2_to_imgmsg(thermal_image, "bgr8")
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = "thermal_camera_frame"
                thermal_pub.publish(ros_image)
                frame_count += 1
            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")

        rate.sleep()

    if cap is not None:
        cap.release()


def generate_simulated_thermal():
    """Generate a simulated thermal image with hot spots."""
    width, height = 320, 240

    # Create base thermal noise
    thermal_data = np.random.normal(25, 3, (height, width))

    # Add some hot spots (simulating people or warm objects)
    num_hotspots = np.random.randint(1, 4)
    for _ in range(num_hotspots):
        x = np.random.randint(50, width - 50)
        y = np.random.randint(50, height - 50)

        # Create Gaussian hot spot
        Y, X = np.ogrid[:height, :width]
        dist = np.sqrt((X - x)**2 + (Y - y)**2)
        hotspot = 10 * np.exp(-dist**2 / (2 * 20**2))
        thermal_data += hotspot

    # Normalize to 0-255 range
    thermal_normalized = np.clip(thermal_data, TEMP_MIN, TEMP_MAX)
    thermal_normalized = ((thermal_normalized - TEMP_MIN) / 
                          (TEMP_MAX - TEMP_MIN) * 255).astype(np.uint8)

    # Apply colormap (COLORMAP_JET gives the classic thermal look)
    thermal_colored = cv2.applyColorMap(thermal_normalized, cv2.COLORMAP_JET)

    # Add temperature scale and timestamp
    timestamp = rospy.Time.now()
    cv2.putText(thermal_colored, f"Thermal - {timestamp.to_sec():.2f}", 
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(thermal_colored, f"{TEMP_MAX:.1f}C", 
                (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(thermal_colored, f"{TEMP_MIN:.1f}C", 
                (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    return thermal_colored


if __name__ == '__main__':
    try:
        thermal_camera_publisher()
    except rospy.ROSInterruptException:
        pass
EOF

chmod +x ~/drone_ws/src/scout_drone/nodes/thermal_camera_node.py
```

**Key Features:**
- Simulated heat signatures with Gaussian hot spots
- JET colormap for thermal visualization
- Temperature scale overlay

### Step 4.3: LiDAR Node

```bash
cat > ~/drone_ws/src/scout_drone/nodes/lidar_node.py << 'EOF'
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
import serial
import time
import math
import random

# --- CONFIG ---
LIDAR_PORT = None  # Set to None for simulation, or serial port path for real LiDAR
LIDAR_BAUDRATE = 115200  # TFMini Plus default baudrate
SIMULATION_MODE = (LIDAR_PORT is None)

# LiDAR specifications (TFMini Plus)
MIN_RANGE = 0.1  # meters
MAX_RANGE = 12.0  # meters
FIELD_OF_VIEW = 0.04  # radians (~2.3 degrees)

def lidar_publisher():
    rospy.init_node('lidar_node', anonymous=True)
    range_pub = rospy.Publisher('/lidar/range', Range, queue_size=10)
    rate = rospy.Rate(100)
    ser = None
    time_offset = 0.0

    if not SIMULATION_MODE:
        rospy.loginfo(f"Attempting to connect to LiDAR on port: {LIDAR_PORT}")
    else:
        rospy.loginfo("Running in SIMULATION mode (no physical LiDAR)")

    while not rospy.is_shutdown():
        range_value = None

        # --- Real LiDAR Mode ---
        if not SIMULATION_MODE:
            if ser is None or not ser.is_open:
                try:
                    rospy.loginfo(f"Connecting to LiDAR on {LIDAR_PORT}...")
                    ser = serial.Serial(LIDAR_PORT, LIDAR_BAUDRATE, timeout=1)
                    rospy.loginfo("LiDAR connected successfully.")
                except serial.SerialException as e:
                    rospy.logwarn(f"Failed to connect to LiDAR: {e}. Retrying in 5 seconds...")
                    time.sleep(5)
                    continue

            try:
                # Read TFMini Plus data (9-byte frame format)
                if ser.in_waiting >= 9:
                    data = ser.read(9)
                    
                    # Verify header
                    if data[0] == 0x59 and data[1] == 0x59:
                        # Calculate checksum
                        checksum = sum(data[:8]) & 0xFF
                        if checksum == data[8]:
                            # Extract distance (in cm, convert to meters)
                            distance_cm = data[2] + (data[3] << 8)
                            range_value = distance_cm / 100.0
                        else:
                            rospy.logwarn("LiDAR checksum mismatch")
                    else:
                        # Sync to header
                        ser.read(1)
                        
            except serial.SerialException as e:
                rospy.logerr(f"LiDAR read error: {e}. Reconnecting...")
                ser.close()
                ser = None
                continue

        # --- Simulation Mode ---
        else:
            range_value = generate_simulated_range(time_offset)
            time_offset += 0.01

        # --- Publishing ---
        if range_value is not None:
            range_msg = Range()
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = "lidar_frame"
            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = FIELD_OF_VIEW
            range_msg.min_range = MIN_RANGE
            range_msg.max_range = MAX_RANGE
            range_msg.range = max(MIN_RANGE, min(MAX_RANGE, range_value))
            range_pub.publish(range_msg)

        rate.sleep()

    if ser is not None and ser.is_open:
        ser.close()


def generate_simulated_range(time_offset):
    """Generate simulated range data with realistic variations."""
    base_altitude = 10.0  # meters
    altitude_variation = 2.0  # meters
    altitude = base_altitude + altitude_variation * math.sin(time_offset)
    noise = random.gauss(0, 0.05)
    measured_range = altitude + noise
    return measured_range


if __name__ == '__main__':
    try:
        lidar_publisher()
    except rospy.ROSInterruptException:
        pass
EOF

chmod +x ~/drone_ws/src/scout_drone/nodes/lidar_node.py
```

**Key Features:**
- TFMini Plus serial protocol support
- Checksum validation
- Simulated hovering altitude with sinusoidal variation

---

## Phase 5: Launch Configuration

### Step 5.1: Create Launch File

```bash
cat > ~/drone_ws/src/scout_drone/launch/sensors.launch << 'EOF'
<?xml version="1.0"?>
<launch>
    <!-- Scout Drone Sensor Nodes -->
    
    <!-- Camera Node -->
    <node name="camera_node" pkg="scout_drone" type="camera_node.py" output="screen">
        <param name="camera_source" value="0"/>  <!-- 0 for webcam, path for video file -->
    </node>
    
    <!-- Thermal Camera Node -->
    <node name="thermal_camera_node" pkg="scout_drone" type="thermal_camera_node.py" output="screen">
        <!-- Set camera_source to device path for real thermal camera, or leave empty for simulation -->
    </node>
    
    <!-- LiDAR Node -->
    <node name="lidar_node" pkg="scout_drone" type="lidar_node.py" output="screen">
        <!-- Set lidar_port to serial port for real LiDAR, or leave empty for simulation -->
    </node>
    
</launch>
EOF
```

### Step 5.2: Build Everything

```bash
cd ~/drone_ws
catkin_make
source devel/setup.bash
```

---

## Phase 6: Testing

### Step 6.1: Open 4 Separate Terminals

**How to open multiple WSL terminals:**

**Method 1 (Windows Terminal):**
1. Press `Win + R`, type `wt`, press Enter
2. Press `Ctrl + Shift + T` three times (creates 4 tabs total)

**Method 2 (Traditional):**
- Press `Win + R`, type `wsl`, press Enter (repeat 4 times)

### Step 6.2: Run Commands in Each Terminal

**Terminal 1 - ROS Master:**
```bash
roscore
```
Leave this running!

**Terminal 2 - Launch Nodes:**
```bash
source ~/drone_ws/devel/setup.bash
roslaunch scout_drone sensors.launch
```

**Expected output:**
```
[INFO] Running in SIMULATION mode (no physical camera)
[INFO] Running in SIMULATION mode (no physical thermal camera)
[INFO] Running in SIMULATION mode (no physical LiDAR)
```

**Terminal 3 - View Camera Feed:**
```bash
source ~/drone_ws/devel/setup.bash
rosrun rqt_image_view rqt_image_view
```
- Select `/camera/image_raw` from dropdown
- You should see animated test pattern with crosshair

**Terminal 4 - Monitor Topics:**
```bash
source ~/drone_ws/devel/setup.bash

# List all topics
rostopic list

# Check publishing rates
rostopic hz /camera/image_raw        # Should show ~30 Hz
rostopic hz /thermal/image_raw       # Should show ~10 Hz
rostopic hz /lidar/range             # Should show ~100 Hz

# View LiDAR data
rostopic echo /lidar/range
```

---

## Phase 7: IP Webcam Integration

### Step 7.1: Setup IP Webcam on Phone

**For Android:**
1. Download "IP Webcam" app by Pavel Khlebovich from Play Store
2. Open app and scroll down
3. Tap "Start server"
4. Note the IP address (e.g., `http://192.168.1.105:8080`)

**Important:** Phone and PC must be on the same WiFi!

### Step 7.2: Test URL in Browser

1. Open Chrome on Windows
2. Go to `http://192.168.1.105:8080` (your IP)
3. If you see "Connection not private":
   - Click "Advanced"
   - Click "Proceed to [IP] (unsafe)"
4. You should see the IP Webcam interface

### Step 7.3: Configure Camera Node

Stop the nodes (Ctrl+C in Terminal 2), then edit:

```bash
nano ~/drone_ws/src/scout_drone/nodes/camera_node.py
```

Find lines 10-11 and change:
```python
CAMERA_SOURCE = "http://192.168.1.105:8080/video"  # Your phone's IP
SIMULATION_MODE = False
```

Save: `Ctrl + X`, then `Y`, then `Enter`

### Step 7.4: Restart and Test

**Terminal 2:**
```bash
source ~/drone_ws/devel/setup.bash
roslaunch scout_drone sensors.launch
```

You should see:
```
[INFO] Attempting to connect to camera source: http://192.168.1.105:8080/video
[INFO] Camera connected successfully.
```

**Terminal 3:**
The `rqt_image_view` should now show your phone's camera! 📱

---

## Troubleshooting

### Issue 1: `catkin_make` fails with XML error

**Error:**
```
Error(s) in package '/home/user/drone_ws/src/drone_msgs/package.xml':
The manifest contains invalid XML:
junk after document element: line 3, column 2
```

**Solution:**
The `package.xml` file had duplicate content or syntax errors. We replaced it completely with the corrected version (see Step 2.3).

### Issue 2: Camera keeps retrying connection

**Error:**
```
[WARN] Failed to connect to camera. Retrying in 5 seconds...
```

**Cause:** No webcam available and still in hardware mode.

**Solution:** Set `CAMERA_SOURCE = None` and `SIMULATION_MODE = True` for testing without hardware.

### Issue 3: IP Webcam "Connection not private"

**Cause:** Chrome blocks HTTP connections.

**Solution:** Click "Advanced" → "Proceed to [IP] (unsafe)" - this is safe for local network.

### Issue 4: Topics not publishing

**Solution:**
```bash
# Make sure roscore is running in Terminal 1
# Source the workspace in every terminal
source ~/drone_ws/devel/setup.bash
```

---

## Hardware Deployment (Jetson Nano)

When deploying to the actual drone:

### Camera Configuration
```python
# For CSI camera
CAMERA_SOURCE = "/dev/video0"
SIMULATION_MODE = False

# For USB camera
CAMERA_SOURCE = 0  # or 1, 2, etc.
SIMULATION_MODE = False

# For IP camera
CAMERA_SOURCE = "http://192.168.0.105:8080/video"
SIMULATION_MODE = False
```

### Thermal Camera Configuration
```python
CAMERA_SOURCE = "/dev/video1"  # Check with: ls /dev/video*
SIMULATION_MODE = False
```

### LiDAR Configuration
```python
# For USB-to-Serial adapter
LIDAR_PORT = "/dev/ttyUSB0"
SIMULATION_MODE = False

# For GPIO UART on Jetson
LIDAR_PORT = "/dev/ttyAMA0"
SIMULATION_MODE = False
```

**Check serial ports:**
```bash
ls /dev/tty*
```

---
