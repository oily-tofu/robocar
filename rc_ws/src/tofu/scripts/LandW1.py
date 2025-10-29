#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import TransformStamped
import serial
from tf.transformations import euler_from_quaternion
import math
from sensor_msgs.msg import Imu  # 新增：导入 IMU 消息类型

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

        # 初始化 TF 监听器（仅用于位置，不再用于姿态）
        self.listener = tf.TransformListener()

        # 新增：订阅 IMU 数据
        self.yaw_deg = 0.0  # 存储当前 yaw 角度（单位：度）
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

    def imu_callback(self, imu_msg):
        # 从 IMU 四元数计算欧拉角
        orientation = imu_msg.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.yaw_deg = math.degrees(yaw)  # 转换为角度并存储

    def send(self):
        rate = rospy.Rate(20)  # 20Hz
        while not rospy.is_shutdown():
            try:
                # 监听 map 到 base_link 的变换（仅获取位置，姿态来自 IMU）
                self.listener.waitForTransform('/camera_init', '/body', rospy.Time(0), rospy.Duration(1.0))
                (trans, _) = self.listener.lookupTransform('/camera_init', '/body', rospy.Time(0))  # 忽略旋转

                # 构造串口发送内容
                send_str1 = f"AD{trans[0]:.3f}\n"
                self.ser.write(send_str1.encode('utf-8'))
                rospy.loginfo(f"Sent: {send_str1}")

                send_str2 = f"B{trans[1]:.3f}E\n"
                self.ser.write(send_str2.encode('utf-8'))
                rospy.loginfo(f"Sent: {send_str2}")

                # 使用 IMU 计算的 yaw
                send_str3 = f"C{self.yaw_deg:.2f}F\n"
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
