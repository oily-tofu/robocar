#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    # 初始化节点，命名为'publisher_node'
    rospy.init_node('publisher_node', anonymous=True)
    
    # 创建一个Publisher，发布到'topic_name'话题，消息类型为String
    pub = rospy.Publisher('topic_name', String, queue_size=10)
    
    # 设置发布频率(Hz)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        # 准备要发布的消息
        hello_str = "hello world %s" % rospy.get_time()
        
        # 发布消息
        pub.publish(hello_str)
        
        # 打印日志
        rospy.loginfo(hello_str)
        
        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass