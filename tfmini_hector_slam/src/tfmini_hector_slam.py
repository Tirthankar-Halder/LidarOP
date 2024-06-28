#!/usr/bin/env python

import rospy
import serial
from sensor_msgs.msg import LaserScan

# Initialize serial communication with TF Mini
ser = serial.Serial('/dev/ttyTHS1', 115200)

def get_distance():
    if ser.in_waiting > 0:
        data = ser.read(9)
        if data[0] == 0x59 and data[1] == 0x59:
            distance = data[2] + data[3] * 256
            return distance / 100.0  # Convert to meters
    return None

def publish_scan():
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rospy.init_node('tfmini_hector_slam', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    scan_msg = LaserScan()
    scan_msg.header.frame_id = 'laser'
    scan_msg.angle_min = 0
    scan_msg.angle_max = 3.14
    scan_msg.angle_increment = 3.14 / 180
    scan_msg.time_increment = 0.1
    scan_msg.range_min = 0.3
    scan_msg.range_max = 12.0
    scan_msg.ranges = [float('inf')] * 180  # Initialize with invalid range values

    while not rospy.is_shutdown():
        angle = int(input("Enter the angle in degrees (0-179): "))
        distance = get_distance()
        if distance:
            scan_msg.ranges[angle] = distance
            scan_msg.header.stamp = rospy.Time.now()
            pub.publish(scan_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_scan()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close()
