#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import TransformStamped
import serial


class SerialSender:

    def __init__(self):
        # 从参数服务器读取配置
        self.port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 921600)
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
        rate = rospy.Rate(1)  # 1Hz
        # while not rospy.is_shutdown():
        try:
            for i in range(5):
                send_str = "0xAA,0x00,0x00,0x0D"
                self.ser.write(send_str.encode('utf-8'))
                rospy.loginfo(f"Sent: {send_str}")

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed: {e}")
        except Exception as e:
            rospy.logerr(f"Serial send failed: {e}")

        rate.sleep()    


if __name__ == '__main__':
    try:
        rospy.init_node('uart_sender1')
        sender = SerialSender()
        sender.send()
    except rospy.ROSInterruptException:
        pass
