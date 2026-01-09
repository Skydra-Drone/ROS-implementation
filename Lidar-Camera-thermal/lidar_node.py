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
