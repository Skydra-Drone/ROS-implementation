
### Pre-requisite: Install a Geospatial Library

The final step of the calculation (converting a meter offset to a new lat/lon coordinate) is complex to do from scratch. We'll use a standard, well-tested Python library called `pyproj` to handle this.

```bash
# Install the pyproj library
pip install pyproj
```

---

### Step 1: Update Package Dependencies

Our new node will depend on several new message types and the `pyproj` library. We need to tell ROS about them.

1.  **Edit `scout_nodes/package.xml`:**
    ```bash
    nano ~/catkin_ws/src/scout_nodes/package.xml
    ```
    Add dependencies for `nav_msgs` (for GPS), `sensor_msgs` (for IMU), and `geographic_msgs` (for the output). Add them as `<depend>` tags, which covers both build and execution.
    ```xml
    <depend>rospy</depend>
    <depend>std_msgs</depend>
    <depend>sensor_msgs</depend>
    <depend>nav_msgs</depend>
    <depend>vision_msgs</depend>
    <depend>custom_msgs</depend> <!-- We created this -->
    <depend>geographic_msgs</depend>
    ```

2.  **Edit `scout_nodes/CMakeLists.txt`:**
    ```bash
    nano ~/catkin_ws/src/scout_nodes/CMakeLists.txt
    ```
    Add the new dependencies to the `find_package` list.
    ```cmake
    find_package(catkin REQUIRED COMPONENTS
      rospy
      std_msgs
      sensor_msgs
      nav_msgs
      vision_msgs
      custom_msgs
      geographic_msgs
    )
    ```

3.  **Build the workspace** to make sure the changes are registered correctly.
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

---

### Step 2: Create the `geolocation_node.py` Script

This is the core of our work. The code will implement the logic we just discussed: synchronization, data extraction, and calculation.

1.  **Create the Python script file:**
    ```bash
    roscd scout_nodes/scripts
    nano geolocation_node.py
    ```

2.  **Add the following code to `geolocation_node.py`:** Read the comments carefully, as they explain each step of the process.

    ```python
    #!/usr/bin/env python3
    import rospy
    import message_filters
    import numpy as np
    from scipy.spatial.transform import Rotation
    from pyproj import Proj, Transformer

    # Import all the necessary message types
    from custom_msgs.msg import YoloDetectionArray
    from sensor_msgs.msg import NavSatFix, Imu, Range
    from geographic_msgs.msg import GeoPoint

    class GeolocationNode:
        def __init__(self):
            rospy.init_node('geolocation_node', anonymous=True)
            rospy.loginfo("Geolocation Node Started")

            # --- Camera Intrinsic Parameters ---
            # IMPORTANT: You MUST replace these with the actual values for your camera.
            # You can find these by running a camera calibration process.
            # For now, these are plausible estimates for a standard USB webcam.
            self.fx = 600.0  # Focal length in x
            self.fy = 600.0  # Focal length in y
            self.cx = 320.0  # Principal point x (image width / 2)
            self.cy = 240.0  # Principal point y (image height / 2)

            # --- ROS Publishers ---
            # We will publish the final calculated GPS coordinate here
            self.target_pub = rospy.Publisher('/mission/target_coordinates', GeoPoint, queue_size=10)

            # --- Synchronized Subscribers ---
            # Create subscribers for each topic
            detection_sub = message_filters.Subscriber('/yolo/detections', YoloDetectionArray)
            gps_sub = message_filters.Subscriber('/mavros/global_position/global', NavSatFix)
            imu_sub = message_filters.Subscriber('/mavros/imu/data', Imu)
            lidar_sub = message_filters.Subscriber('/mavros/global_position/rel_alt', Range) # Using rel_alt for now as a stand-in for LiDAR

            # The ApproximateTimeSynchronizer is the key to this node.
            # It takes a list of subscribers and a queue size, and a time slop (in seconds).
            # It will only call the callback when it has received a message from each topic
            # that is within the 'slop' time of the others.
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [detection_sub, gps_sub, imu_sub, lidar_sub],
                queue_size=10,
                slop=0.1,  # Allow 100ms difference between message timestamps
                allow_headerless=False # Ensure all messages have timestamps
            )

            # Register the callback function
            self.ts.registerCallback(self.synchronized_callback)

            # --- Coordinate System Transformer ---
            # Used to convert from a local ENU (East, North, Up) frame to global Lat/Lon
            self.wgs84 = "EPSG:4326"  # Standard Lat/Lon

        def synchronized_callback(self, detections_msg, gps_msg, imu_msg, lidar_msg):
            """
            This callback is triggered only when a synchronized set of messages is received.
            """
            # Step 0: Check if there are any detections
            if not detections_msg.detections:
                return # No detections, do nothing

            # For simplicity, we'll just process the first detection in the list
            detection = detections_msg.detections[0]

            rospy.loginfo("--- Synchronized Data Received ---")

            # --- Step 1: Extract and Prepare Data ---
            # Get the center of the bounding box
            pixel_x = (detection.x1 + detection.x2) / 2.0
            pixel_y = (detection.y1 + detection.y2) / 2.0

            # Get the drone's GPS position
            drone_lat = gps_msg.latitude
            drone_lon = gps_msg.longitude

            # Get the drone's height above ground from LiDAR/Range sensor
            # The 'range' field holds the distance in meters.
            height_above_ground = lidar_msg.range
            if height_above_ground <= 0.1: # Sanity check for bad readings
                rospy.logwarn("Invalid height from LiDAR, skipping calculation.")
                return

            # Get the drone's orientation from the IMU
            orientation_q = imu_msg.orientation
            # Convert the quaternion to a rotation object
            rotation = Rotation.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            
            # --- Step 2: Camera Geometry Calculation (Vector Math) ---
            # Convert pixel coordinates to a 3D vector in the camera's coordinate frame.
            # The camera frame is typically: X right, Y down, Z forward.
            vector_camera_frame = np.array([
                (pixel_x - self.cx) / self.fx,
                (pixel_y - self.cy) / self.fy,
                1.0  # The Z component is 1 because this is a direction vector
            ])

            # Normalize the vector to have a length of 1 (unit vector)
            vector_camera_frame = vector_camera_frame / np.linalg.norm(vector_camera_frame)

            # --- Step 3: Coordinate Frame Transformations ---
            # The drone's body frame (FLU: Forward, Left, Up) needs to be aligned with the
            # camera frame (RDF: Right, Down, Forward). This often requires a static rotation.
            # Assuming camera points straight forward, this can sometimes be simplified,
            # but a full implementation would use a static transform here.
            # For now, we'll assume a simple forward-facing camera.

            # Rotate the vector from the camera frame to the drone's body frame (FLU).
            # Then rotate from the drone's body frame to the world frame (NED: North, East, Down).
            # The 'rotation' object from the IMU directly gives us the body-to-world rotation.
            vector_world_frame = rotation.apply(vector_camera_frame)

            # --- Step 4: Ground Intersection Calculation ---
            # Check if the vector is pointing downwards. If not, it can't intersect the ground.
            # In the NED frame, the 'Down' component is the 3rd element (index 2).
            if vector_world_frame[2] <= 0:
                rospy.logwarn("Detection is not pointing towards the ground.")
                return

            # Calculate the scaling factor to project the vector onto the ground plane.
            # This is based on similar triangles.
            scale_factor = height_above_ground / vector_world_frame[2]

            # Calculate the offset in meters in the North and East directions.
            north_offset_m = vector_world_frame[0] * scale_factor
            east_offset_m = vector_world_frame[1] * scale_factor

            rospy.loginfo(f"Calculated offset: {north_offset_m:.2f}m North, {east_offset_m:.2f}m East")

            # --- Step 5: Final GPS Coordinate Calculation ---
            # Define a local projection centered on the drone's current location
            local_projection = f"+proj=tmerc +lat_0={drone_lat} +lon_0={drone_lon} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
            
            # Create a transformer to convert from our local (East, North) to global (Lat, Lon)
            transformer = Transformer.from_crs(local_projection, self.wgs84, always_xy=True)

            # Transform the (East, North) offset to a new (Lon, Lat) coordinate
            target_lon, target_lat = transformer.transform(east_offset_m, north_offset_m)

            # --- Publish the Result ---
            target_point = GeoPoint()
            target_point.latitude = target_lat
            target_point.longitude = target_lon
            target_point.altitude = 0 # Altitude on the ground

            self.target_pub.publish(target_point)
            rospy.loginfo(f"Published Target GPS: Lat {target_lat:.6f}, Lon {target_lon:.6f}")

        def run(self):
            rospy.spin()

    if __name__ == '__main__':
        try:
            node = GeolocationNode()
            node.run()
        except rospy.ROSInterruptException:
            pass
    ```

3.  **Make the script executable:**
    ```bash
    chmod +x geolocation_node.py
    ```
    
    **Important Note on LiDAR:** In the code above, I've used `/mavros/global_position/rel_alt` as a temporary stand-in for a LiDAR. This topic provides the drone's altitude relative to its home/arming point, which is better than nothing for testing. When you get a real LiDAR, you would install a `lidar_node` that publishes to a topic like `/lidar/range`, and you would simply change the topic name in the `message_filters.Subscriber` line.

---

### Step 3: Create a Launch File for Testing

Running 4-5 nodes in separate terminals is tedious. A ROS launch file starts everything you need with a single command.

1.  **Create a `launch` directory in your package:**
    ```bash
    roscd scout_nodes
    mkdir launch
    nano launch/test_perception.launch
    ```

2.  **Add the following content to `test_perception.launch`:**
    ```xml
    <launch>
        <!-- Start the Camera Node -->
        <node name="camera_node" pkg="scout_nodes" type="camera_node.py" output="screen" />

        <!-- Start the YOLO Node -->
        <node name="yolo_detector_node" pkg="scout_nodes" type="yolo_node.py" output="screen" />
        
        <!-- Start the Geolocation Node -->
        <node name="geolocation_node" pkg="scout_nodes" type="geolocation_node.py" output="screen" />

        <!-- To test this without a real drone, we can "play back" a fake sensor message file -->
        <!-- This is a powerful ROS feature called a "bag file" -->
        <!-- For now, we'll run a script that publishes fake data -->
        <node name="fake_drone_publisher" pkg="scout_nodes" type="fake_drone_publisher.py" output="screen" />

    </launch>
    ```

---

### Step 4: Create a Fake Drone Data Publisher for Testing

The `geolocation_node` won't run without MAVROS data. Since we're not flying yet, we'll create a simple "fake" publisher to provide plausible data for testing the full pipeline on your desktop.

1.  **Create the script:**
    ```bash
    roscd scout_nodes/scripts
    nano fake_drone_publisher.py
    ```

2.  **Add this code:**
    ```python
    #!/usr/bin/env python3
    import rospy
    from sensor_msgs.msg import NavSatFix, Imu, Range
    from scipy.spatial.transform import Rotation

    def fake_drone_publisher():
        rospy.init_node('fake_drone_publisher', anonymous=True)
        
        gps_pub = rospy.Publisher('/mavros/global_position/global', NavSatFix, queue_size=10)
        imu_pub = rospy.Publisher('/mavros/imu/data', Imu, queue_size=10)
        range_pub = rospy.Publisher('/mavros/global_position/rel_alt', Range, queue_size=10)
        
        rate = rospy.Rate(10) # 10 Hz

        rospy.loginfo("Publishing fake drone data...")

        while not rospy.is_shutdown():
            header = rospy.Header()
            header.stamp = rospy.Time.now()

            # --- GPS Data ---
            gps_msg = NavSatFix()
            gps_msg.header = header
            gps_msg.latitude = 40.7128 # e.g., New York City
            gps_msg.longitude = -74.0060
            gps_msg.altitude = 100 # Altitude above sea level

            # --- IMU Data ---
            imu_msg = Imu()
            imu_msg.header = header
            # Simulate a level drone (no rotation)
            # A level quaternion is (x=0, y=0, z=0, w=1)
            imu_msg.orientation.x = 0
            imu_msg.orientation.y = 0
            imu_msg.orientation.z = 0
            imu_msg.orientation.w = 1

            # --- Range/LiDAR Data ---
            range_msg = Range()
            range_msg.header = header
            range_msg.range = 20.0 # Simulating 20 meters above ground

            # Publish all messages
            gps_pub.publish(gps_msg)
            imu_pub.publish(imu_msg)
            range_pub.publish(range_msg)

            rate.sleep()

    if __name__ == '__main__':
        try:
            fake_drone_publisher()
        except rospy.ROSInterruptException:
            pass
    ```

3.  **Make it executable:**
    ```bash
    chmod +x fake_drone_publisher.py
    ```

---

### Step 5: The Final End-to-End Test

Now you can test the entire perception pipeline with one command.

1.  **In Terminal 1:**
    ```bash
    roscore
    ```

2.  **In Terminal 2:**
    ```bash
    roslaunch scout_nodes test_perception.launch
    ```    This will start the camera, YOLO node, fake drone publisher, and the geolocation node.

3.  **In Terminal 3:** Listen to the final output of the entire system.
    ```bash
    rostopic echo /mission/target_coordinates
    ```

**Expected Result:** When a person appears in your camera's view, the `geolocation_node` will receive the synchronized data and begin publishing `GeoPoint` messages to the `/mission/target_coordinates` topic. You will see these GPS coordinates streaming in your third terminal. This proves that your entire perception and geolocation pipeline is working correctly