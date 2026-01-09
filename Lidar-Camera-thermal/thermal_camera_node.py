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
