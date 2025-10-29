#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Header
from collections import deque
import threading

imu_buffer = deque(maxlen=1000)
buffer_lock = threading.Lock()
synced_imu_pub = None

def imu_callback(msg):
    with buffer_lock:
        imu_buffer.append(msg)

def scan_callback(scan_msg):
    with buffer_lock:
        if not imu_buffer:
            rospy.logwarn("IMU buffer is empty, skipping this scan")
            return

        scan_time = scan_msg.header.stamp.to_sec()
        closest_imu = min(imu_buffer, key=lambda imu: abs(imu.header.stamp.to_sec() - scan_time))

        time_diff = abs(closest_imu.header.stamp.to_sec() - scan_time)
        max_time_diff = 0.01
        if time_diff > max_time_diff:
            rospy.logwarn(f"Time diff too large: {time_diff:.6f}s, skip")
            return

        synced_imu = Imu()
        synced_imu.header = Header()
        synced_imu.header.stamp = scan_msg.header.stamp
        synced_imu.header.frame_id = "imu_link"

        synced_imu.orientation = closest_imu.orientation
        synced_imu.angular_velocity = closest_imu.angular_velocity
        synced_imu.linear_acceleration = closest_imu.linear_acceleration

        synced_imu_pub.publish(synced_imu)
        #rospy.loginfo(f"Published synced IMU at time: {scan_msg.header.stamp.to_sec():.6f}")

def main():
    global synced_imu_pub

    rospy.init_node('imu_lidar_sync_node')
    rospy.Subscriber("/imu", Imu, imu_callback, queue_size=1000)
    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=20)
    synced_imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=20)

    rospy.spin()

if __name__ == '__main__':
    main()