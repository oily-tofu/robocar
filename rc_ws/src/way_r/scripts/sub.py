#!/usr/bin/env python3

import rospy
from way_r.msg import way_def, way_save

def callback(msg):
    for idx, point in enumerate(msg.way):
        rospy.loginfo("Point %d: x=%f, y=%f, z=%f, timestamp=%s", idx, point.x, point.y, point.z, point.timestamp)

def subscriber():

    rospy.init_node('sub_kfs_node', anonymous=True)
    

    rospy.Subscriber('kfs', way_save, callback)
    

    rospy.spin()

if __name__ == '__main__':
    subscriber()