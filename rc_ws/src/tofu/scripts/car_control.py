#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# W/A/S/D 键对应的速度
moveBindings = {
    'w': (1, 0),   # 前进
    's': (-1, 0),  # 后退
    'a': (0, 1),   # 左转
    'd': (0, -1),  # 右转
}

# 线速度和角速度大小
LIN_SPEED = 1.5  # m/s
ANG_SPEED = 5.0  # rad/s

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('teleop_diff_car')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    try:
        print("Use WASD keys to move the robot, Ctrl-C to quit")
        while not rospy.is_shutdown():
            key = getKey()
            if key in moveBindings.keys():
                lin, ang = moveBindings[key]
                twist = Twist()
                twist.linear.x = lin * LIN_SPEED
                twist.angular.z = ang * ANG_SPEED
                pub.publish(twist)
            elif key == '\x03':  # Ctrl-C
                break
            else:
                # 停止
                twist = Twist()
                pub.publish(twist)
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
