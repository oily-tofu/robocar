#!/usr/bin/env python3

#发布伪kfs识别信息

import rospy
# from std_msgs.msg import String
import re, time
from way_r.msg import way_def, way_save

coordinates = way_save() 


def monitor_coordinates():
    print("开始监听坐标（(x, y)）", flush=True)

    pattern = re.compile(r'\((\d+),(\d+),(\d+)\)')

    line = input("输入坐标: ").strip()
    matches = pattern.findall(line)

    if matches:
        for match in matches:
            x, y ,z= map(float, match)
            timestamp = time.strftime("%H:%M:%S")
            t_pkg= way_def()
            t_pkg.x = x
            t_pkg.y = y
            t_pkg.z = z
            t_pkg.timestamp = timestamp
            coordinates.way.append(t_pkg)
            print(" 已记录坐标", flush=True)
            pub.publish(coordinates)
    else:
        print(" 格式错误(x, y)", flush=True)



if __name__ == '__main__':

    print("Ctrl-C to quit")
    rospy.init_node('pub_kfs_node', anonymous=True)

    pub = rospy.Publisher('kfs', way_save, queue_size=10)
        
    rate = rospy.Rate(10) 

    try:
        while not rospy.is_shutdown():
            monitor_coordinates()
            rate.sleep()
    except KeyboardInterrupt:
        print("\n停止监听")
        



        


        
