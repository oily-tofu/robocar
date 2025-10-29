#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import TransformStamped
import serial
from tf.transformations import euler_from_quaternion
import math


class SerialSender:

    def __init__(self):
        # 从参数服务器读取配置
        self.port = rospy.get_param('~serial_port', '/dev/ch340')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.default_data = rospy.get_param('~send_data', 'Hello from ROS!')

        # 初始化串口
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            rospy.loginfo(f"Serial port {self.port} opened at {self.baud_rate} baud")
        except Exception as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            raise

        # 初始化 TF 监听器
        self.listener = tf.TransformListener()

    def send(self):
        rate = rospy.Rate(20)  # 1Hz
        while not rospy.is_shutdown():
            try:
                # 监听 map 到 base_link 的变换
                self.listener.waitForTransform('/camera_init', '/body', rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform('/camera_init', '/body', rospy.Time(0))

                roll, pitch, yaw = euler_from_quaternion(rot)
                yaw_deg = math.degrees(yaw)

                # 构造串口发送内容
                # send_str = f"AD{trans[0]:.3f},{trans[1]:.3f}AE\n"
                send_str1 = f"AD{trans[0]:.3f}\n"
                self.ser.write(send_str1.encode('utf-8'))
                rospy.loginfo(f"Sent: {send_str1}")
                send_str2 = f"B{trans[1]:.3f}E\n"
                self.ser.write(send_str2.encode('utf-8'))
                rospy.loginfo(f"Sent: {send_str2}")
                
                send_str3 = f"C{yaw_deg:.2f}F\n"
                self.ser.write(send_str3.encode('utf-8'))
                rospy.loginfo(f"Sent: {send_str3}")

            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"TF lookup failed: {e}")
            except Exception as e:
                rospy.logerr(f"Serial send failed: {e}")

            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('uart_sender')
        sender = SerialSender()
        sender.send()
    except rospy.ROSInterruptException:
        pass
