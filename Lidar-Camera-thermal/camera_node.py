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
