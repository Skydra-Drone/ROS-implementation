
### Goal of the `geolocation_node`

The node has one primary job: to answer the question, "**What are the real-world GPS coordinates of the person I see in this video frame?**"

To do this, it needs to be a central "listener" that gathers pieces of a puzzle from other nodes and puts them together at the right moment.

### The Puzzle Pieces (Inputs)

This node will **subscribe** to several topics to get the data it needs. Think of it as tuning into different radio stations simultaneously.

1.  **WHAT was detected? (From `yolo_node`)**
    *   **Topic:** `/yolo/detections`
    *   **Message Type:** `custom_msgs/YoloDetectionArray`
    *   **Data Provided:** A list of all "PERSON" detections in a single camera frame. For each person, it gives us the bounding box pixel coordinates (`x1`, `y1`, `x2`, `y2`). We will calculate the center of this box, let's call it `(pixel_x, pixel_y)`.
    *   **Timing:** This is our **trigger**. The entire calculation process begins the moment a new detection message arrives.

2.  **WHERE is the drone? (From `flight_controller_interface_node` / MAVROS)**
    *   **Topic:** `/mavros/global_position/global`
    *   **Message Type:** `sensor_msgs/NavSatFix`
    *   **Data Provided:** The drone's current latitude, longitude, and altitude (above sea level).

3.  **HOW is the drone oriented? (From `flight_controller_interface_node` / MAVROS)**
    *   **Topic:** `/mavros/imu/data`
    *   **Message Type:** `sensor_msgs/Imu`
    *   **Data Provided:** The drone's current orientation as a **quaternion**. This is a complex but standard way to represent 3D rotation. We will convert this quaternion into more human-readable **Roll, Pitch, and Yaw** angles. This is *absolutely critical* because the drone is never perfectly level. Its tilt affects where the camera is pointing.

4.  **HOW HIGH is the drone above the ground? (From a future `lidar_node`)**
    *   **Topic:** `/lidar/range` (or similar)
    *   **Message Type:** `sensor_msgs/Range`
    *   **Data Provided:** A highly accurate measurement of the drone's distance to the ground directly beneath it. This is far more reliable than GPS altitude for our calculation.

### The Problem of "Time" - Synchronization

A huge challenge is that these messages arrive at different times and different rates. The IMU might publish at 200 Hz, GPS at 10 Hz, and YOLO detections at 5 Hz. If we just use the "latest" message from each topic, the GPS data might be half a second old by the time we get a YOLO detection, leading to a massive error.

**The Solution: `MessageFilter` and `ApproximateTimeSynchronizer`**

ROS has a beautiful solution for this. We won't subscribe to each topic individually. Instead, we will use a special tool called `message_filters.ApproximateTimeSynchronizer`.

**How it works:**
1.  You tell the synchronizer, "I am interested in these four topics: `/yolo/detections`, `/mavros/global_position/global`, `/mavros/imu/data`, and `/lidar/range`."
2.  The synchronizer listens to all four topics internally.
3.  It then waits until it has received one message from **each** of the four topics that are all very close in time (based on the timestamp in their headers).
4.  Once it finds a matching set, it calls a **single callback function** in your code and passes all four messages to it at once.

This guarantees that every time our calculation runs, we are using a set of sensor readings that were all taken at almost the exact same instant. This is the key to an accurate calculation.

---

### The Calculation Procedure (Inside the Synchronized Callback)

Once the callback function receives the synchronized set of four messages, it will perform the following steps in order:

**Step 1: Extract and Prepare Data**
*   Get the person's center pixel `(pixel_x, pixel_y)` from the YOLO detection message.
*   Get the drone's `(latitude, longitude)` from the GPS message.
*   Get the drone's `height_above_ground` from the LiDAR message.
*   Get the drone's orientation quaternion from the IMU message and convert it into **Roll, Pitch, and Yaw** angles.

**Step 2: The Camera Geometry Calculation (Vector Math)**
This is the core of the algorithm. We are going to create a 3D "ray" pointing from the drone to the person and see where that ray intersects the ground.

1.  **Create a Vector in the Camera's View:** We first calculate a 3D vector that represents the direction of the person *relative to the camera's own coordinate system*.
    *   Imagine the camera is pointing straight ahead along its own Z-axis.
    *   We use the person's pixel coordinates `(pixel_x, pixel_y)` and the camera's known **focal length** and **sensor size** (these are the camera's "intrinsic" parameters) to calculate how many degrees "right" (X-axis) and "down" (Y-axis) the person is from the center of the camera's view.
    *   This gives us a 3D vector like `[dx, dy, 1]`, which points from the camera's lens towards the person.

2.  **Rotate the Vector into the Drone's Frame:** The camera is bolted to the drone, which is tilted. We now use the drone's **Roll and Pitch** angles (from the IMU) to rotate that camera vector. This transforms it from "where the camera sees the person" to "where the drone sees the person".

3.  **Rotate the Vector into the World Frame:** The drone is also facing a specific compass direction (Yaw). We now use the drone's **Yaw** angle to rotate the vector again. This transforms it from "where the drone sees the person" to "the direction of the person relative to North, East, and Down". We now have a final 3D world vector, `V_world = [North_component, East_component, Down_component]`.

**Step 3: The Ground Intersection Calculation (Trigonometry)**
We now have everything we need to solve the final geometry problem.

1.  We have the drone's `height_above_ground` from the LiDAR. This is our `H`.
2.  We have the final 3D world vector `V_world`. The `Down_component` of this vector tells us how much the ray is pointing downwards.
3.  Using simple trigonometry (similar triangles), we can calculate how far the ray travels North and East by the time it has traveled `H` meters down.
    *   `Distance_North = (North_component / Down_component) * H`
    *   `Distance_East = (East_component / Down_component) * H`

**Step 4: The Final GPS Coordinate Calculation**
This is the final step.

1.  We have the drone's starting GPS `(latitude, longitude)`.
2.  We have the calculated offset in meters: `(Distance_North, Distance_East)`.
3.  We use a geospatial library (like `pyproj` or a simple spherical Earth model) to calculate a new GPS coordinate by starting at the drone's position and moving `Distance_North` meters North and `Distance_East` meters East.
4.  The result is the final estimated GPS coordinate of the person on the ground.

### The Output

The `geolocation_node` will **publish** its final result to a new topic.

*   **Topic:** `/mission/target_coordinates`
*   **Message Type:** We can create a new custom message, or simply use `geographic_msgs/GeoPoint`.
*   **Data Provided:** The final calculated latitude and longitude of the target.

This topic can then be subscribed to by the `delivery_commander_node` to task the delivery drone.