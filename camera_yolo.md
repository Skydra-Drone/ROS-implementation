# Navigate to your workspace's source directory
cd ~/catkin_ws/src

# Create a package for all scout-related nodes
# Dependencies: rospy (python), sensor_msgs (for images), std_msgs (for simple data),
# vision_msgs (a standard for detection boxes)
catkin_create_pkg scout_nodes rospy sensor_msgs std_msgs vision_msgs

# Create a package for any custom message types we'll need
catkin_create_pkg custom_msgs rospy std_msgs

# Build the workspace to register the new packages
cd ~/catkin_ws
catkin_make

# Source the new environment
source devel/setup.bash```

---

### Step 1: The Camera Node (`camera_node.py`)

**Goal:** Create a node that connects to your camera (or a simulated video file) and publishes its frames as ROS `Image` messages. This replaces the camera connection part of your existing code.

1.  **Create the Python script:**
    ```bash
    # Go into the new package's scripts folder
    roscd scout_nodes/scripts
    nano camera_node.py
    ```

2.  **Add the following code to `camera_node.py`:**
    This code uses `cv_bridge` to convert OpenCV images into ROS `Image` messages.

    ```python
    #!/usr/bin/env python3
    import rospy
    import cv2
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge, CvBridgeError
    import time

    # --- CONFIG ---
    # FOR REAL DRONE: CAMERA_URL = "http://192.168.0.105:8080/video"
    # FOR TESTING ON DESKTOP: Use a video file or a webcam (0)
    # To use a video file, create a 'videos' folder in your package and put a file there.
    # e.g., VIDEO_PATH = "path/to/your/catkin_ws/src/scout_nodes/videos/test_video.mp4"
    CAMERA_SOURCE = 0 # Use 0 for webcam, or a path to a video file.

    def camera_publisher():
        """
        Connects to a camera source and publishes frames as ROS Image messages.
        """
        # Initialize the ROS node
        rospy.init_node('camera_node', anonymous=True)

        # Create a publisher for the image topic
        # The topic name `/camera/image_raw` is a standard convention
        image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

        # Create a CvBridge object to convert between OpenCV and ROS images
        bridge = CvBridge()

        # Set the publishing rate (e.g., 30 Hz)
        rate = rospy.Rate(30)

        cap = None

        while not rospy.is_shutdown():
            # --- Connection Logic ---
            if cap is None or not cap.isOpened():
                rospy.loginfo(f"Attempting to connect to camera source: {CAMERA_SOURCE}")
                cap = cv2.VideoCapture(CAMERA_SOURCE)
                if not cap.isOpened():
                    rospy.logwarn("Failed to connect to camera. Retrying in 5 seconds...")
                    time.sleep(5)
                    continue
                else:
                    rospy.loginfo("Camera connected successfully.")

            # --- Frame Capture and Publishing ---
            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("Failed to grab frame. Reconnecting...")
                cap.release()
                cap = None
                continue

            try:
                # Convert the OpenCV image (BGR) to a ROS Image message
                ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
                
                # Stamp the message with the current time
                ros_image.header.stamp = rospy.Time.now()
                
                # Publish the message
                image_pub.publish(ros_image)
            
            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")

            # Sleep to maintain the desired publishing rate
            rate.sleep()

        # Clean up on shutdown
        if cap is not None:
            cap.release()

    if __name__ == '__main__':
        try:
            camera_publisher()
        except rospy.ROSInterruptException:
            pass
    ```

3.  **Make the script executable:**
    ```bash
    chmod +x camera_node.py
    ```

#### **Testing the Camera Node:**

This is a crucial first test.

1.  **Open a new terminal and start `roscore`:**
    ```bash
    roscore
    ```

2.  **In a second terminal, launch your new `camera_node`:**
    ```bash
    rosrun scout_nodes camera_node.py
    ```
    You should see the "Camera connected successfully" message.

3.  **In a third terminal, visualize the output:** ROS has a built-in image viewer.
    ```bash
    rqt_image_view /camera/image_raw
    ```
    A window should pop up showing the live video feed from your webcam or video file. If you see this, your first node is working perfectly! You can stop all nodes with `Ctrl+C`.

---

### Step 2: Create Custom Messages for Detections

Your YOLO code outputs a bounding box (`x1, y1, x2, y2`), a confidence score, and a class label. While ROS has a standard `vision_msgs/Detection2D`, creating our own simple message is a great learning exercise and can be tailored to our exact needs.

1.  **Create the message definition file:**
    ```bash
    # Go to your custom messages package
    roscd custom_msgs
    mkdir msg
    nano msg/YoloDetection.msg
    ```

2.  **Add the following content to `YoloDetection.msg`:** This defines the structure of a single detection.
    ```
    # Defines a single YOLO detection
    int64 x1
    int64 y1
    int64 x2
    int64 y2
    float64 confidence
    string class_name
    ```

3.  **Create another message for an array of detections:**
    ```bash
    nano msg/YoloDetectionArray.msg
    ```
    Add this content:
    ```
    # Defines a list of YOLO detections from a single frame
    # The header is important for timestamping
    std_msgs/Header header
    YoloDetection[] detections
    ```

4.  **Modify the package configuration to build these messages:**
    ```bash
    # Go back to the custom_msgs package root
    roscd custom_msgs
    nano package.xml
    ```
    Find the lines for `build_depend` and `exec_depend` and add `message_generation` and `message_runtime` respectively. It should look like this (add the new lines if they're missing):
    ```xml
    <!-- ... other dependencies ... -->
    <build_depend>message_generation</build_depend>
    <build_depend>std_msgs</build_depend>
    <!-- ... other dependencies ... -->
    <exec_depend>message_runtime</exec_depend>
    <exec_depend>std_msgs</exec_depend>
    <!-- ... other dependencies ... -->
    ```
    Now, edit the `CMakeLists.txt`:
    ```bash
    nano CMakeLists.txt
    ```
    Find the `find_package` section and add `message_generation` and `std_msgs`:
    ```cmake
    find_package(catkin REQUIRED COMPONENTS
      message_generation
      std_msgs
    )
    ```
    Find the `add_message_files` section (you'll need to uncomment it) and add your message files:
    ```cmake
    add_message_files(
      FILES
      YoloDetection.msg
      YoloDetectionArray.msg
    )
    ```
    Find the `generate_messages` section (uncomment it) and add `std_msgs`:
    ```cmake
    generate_messages(
      DEPENDENCIES
      std_msgs
    )
    ```
    Find `catkin_package` and add `std_msgs` to `CATKIN_DEPENDS`:
    ```cmake
    catkin_package(
      CATKIN_DEPENDS message_runtime std_msgs
    )
    ```

5.  **Build the workspace to create the message files:**
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```    If this builds without errors, ROS now "knows" about your custom `YoloDetection` and `YoloDetectionArray` messages, and you can import them in your Python scripts.

---

### Step 3: The YOLO Node (`yolo_node.py`)

**Goal:** Create a node that subscribes to the `/camera/image_raw` topic, performs YOLO inference on the frames, and publishes the results using our new custom messages.

1.  **Install Ultralytics YOLO:** Your Python script depends on it.
    ```bash
    pip install ultralytics
    ```

2.  **Create the Python script:**
    ```bash
    roscd scout_nodes/scripts
    nano yolo_node.py
    ```

3.  **Add the following code to `yolo_node.py`:** This is a refactored version of your code, adapted for ROS.

    ```python
    #!/usr/bin/env python3
    import rospy
    import cv2
    import torch
    from ultralytics import YOLO
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge, CvBridgeError

    # Import our custom messages
    from custom_msgs.msg import YoloDetection, YoloDetectionArray

    class YoloDetectorNode:
        def __init__(self):
            rospy.init_node('yolo_detector_node', anonymous=True)

            # --- Model and Device Setup ---
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            rospy.loginfo(f"Using device: {self.device}")
            
            # Load your model (ensure the .pt file is accessible)
            # You might need to provide an absolute path or place it in your package
            self.model = YOLO("yolov8n.pt")
            if torch.cuda.is_available():
                self.model = self.model.to("cuda")
            rospy.loginfo("YOLO model loaded successfully.")

            # --- ROS Setup ---
            self.bridge = CvBridge()
            
            # Publisher for detection results
            self.detection_pub = rospy.Publisher('/yolo/detections', YoloDetectionArray, queue_size=10)
            
            # Subscriber to the raw image topic
            self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback, queue_size=1, buff_size=2**24)

            rospy.loginfo("YOLO node is ready.")

        def image_callback(self, msg):
            """
            Callback function for the image subscriber.
            Performs inference and publishes detections.
            """
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")
                return

            # --- Inference ---
            # Using your provided logic for inference
            results = self.model.predict(
                cv_image,
                device=self.device,
                verbose=False,
                classes=[0],  # Only persons
                conf=0.5
            )

            # --- Create and Populate Detection Messages ---
            detection_array_msg = YoloDetectionArray()
            detection_array_msg.header = msg.header # Use the same timestamp as the input image

            if results[0].boxes is not None:
                for box in results[0].boxes:
                    # Create a single detection message
                    detection_msg = YoloDetection()
                    
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    detection_msg.x1 = int(x1)
                    detection_msg.y1 = int(y1)
                    detection_msg.x2 = int(x2)
                    detection_msg.y2 = int(y2)
                    
                    detection_msg.confidence = float(box.conf[0])
                    detection_msg.class_name = "PERSON" # Since we only detect class 0

                    # Add the single detection to our array of detections
                    detection_array_msg.detections.append(detection_msg)

            # Publish the array of detections
            self.detection_pub.publish(detection_array_msg)

        def run(self):
            rospy.spin()

    if __name__ == '__main__':
        try:
            node = YoloDetectorNode()
            node.run()
        except rospy.ROSInterruptException:
            pass
    ```

4.  **Make the script executable:**
    ```bash
    chmod +x yolo_node.py
    ```

#### **Testing the YOLO Node:**

This is a multi-step test to see the whole pipeline so far.

1.  **Terminal 1:** `roscore`
2.  **Terminal 2:** `rosrun scout_nodes camera_node.py`
3.  **Terminal 3:** `rosrun scout_nodes yolo_node.py`
    You should see the YOLO model load messages.
4.  **Terminal 4:** Now, we'll "listen" to the output topic.
    ```bash
    rostopic echo /yolo/detections
    ```    If you have a person (or something the model thinks is a person) in the video feed, you will see your custom messages being printed in this terminal with coordinates and confidence scores! If there are no detections, this terminal will be silent.

**At this point, you have a functional perception system.** The next step will be to create the `geolocation_node` that consumes these detections and data from the simulated drone to calculate a real-world position. We'll tackle that once you have this pipeline working.