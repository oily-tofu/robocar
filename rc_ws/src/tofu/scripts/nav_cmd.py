#!/usr/bin/env python3
import rospy
import serial
from geometry_msgs.msg import Twist

class CmdVelToSerial:
    def __init__(self):
        # 初始化节点
        rospy.init_node('cmd_vel_to_serial')

        # 从参数服务器读取串口配置
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 115200)
        self.timeout = rospy.get_param('~timeout', 0.1)

        # 初始化串口
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            rospy.loginfo(f"Connected to {self.port} at {self.baud} baud")
        except serial.SerialException as e:
            rospy.logerr(f"Serial port error: {e}")
            raise

        # 订阅cmd_vel话题
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # 控制频率
        self.rate = rospy.Rate(50)  # 50Hz

    def cmd_vel_callback(self, msg):
        """处理cmd_vel消息并发送到串口"""
        try:
            # 提取线速度和角速度
            linear_x = msg.linear.x
            linear_y = msg.linear.y
            angular_z = msg.angular.z

            # 构造串口协议（自定义协议示例）
            
            send_str1 = f"AD{linear_x:.3f}\n"
            self.ser.write(send_str1.encode('utf-8'))
            rospy.loginfo(f"Sent: {send_str1}")

            send_str2 = f"B{linear_y:.3f}E\n"
            self.ser.write(send_str2.encode('utf-8'))
            rospy.loginfo(f"Sent: {send_str2}")


            send_str3 = f"C{angular_z:.3f}F\n"
            self.ser.write(send_str3.encode('utf-8'))
            rospy.loginfo(f"Sent: {send_str3}")            

            # # 发送到串口
            # self.ser.write(cmd_str.encode('utf-8'))
            # rospy.loginfo_once(f"Sent first command: {cmd_str.strip()}")

        except Exception as e:
            rospy.logerr(f"Error processing cmd_vel: {e}")

    def run(self):
        """主循环"""
        rospy.loginfo("Ready to forward cmd_vel to serial...")
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = CmdVelToSerial()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if hasattr(node, 'ser'):
            node.ser.close()
            rospy.loginfo("Serial port closed")