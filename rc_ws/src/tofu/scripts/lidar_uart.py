#!/usr/bin/env python
import rospy
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped

def get_tf_transform():
    rospy.init_node('tf1_subscriber')
    
    # 创建TF监听器
    listener = tf.TransformListener()
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        try:
            # 监听map到lidar的变换（等待最多1秒）
            (trans, rot) = listener.lookupTransform(
                '/map',    # 目标坐标系
                '/body',  # 源坐标系（注意：ROS1中通常需要斜杠/）
                rospy.Time(0)  # 获取最新可用变换
            ) 
            
            # 转换为欧拉角（可选）
            euler = euler_from_quaternion(rot)
            
            # 打印结果（格式与您的示例一致）
            #print(f"At time {rospy.Time.now().to_sec()}")
            print(f"- Translation: [{trans[0]:.3f}, {trans[1]:.3f}, {trans[2]:.3f}]")
            # print("- Rotation: in Quaternion [{:.3f}, {:.3f}, {:.3f}, {:.3f}]".format(*rot))
            # print("            in RPY (radian) [{:.3f}, {:.3f}, {:.3f}]".format(*euler))
            # print("            in RPY (degree) [{:.3f}, {:.3f}, {:.3f}]".format(
            #     *[angle * 180/3.14159 for angle in euler]))
            print("="*50)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF获取失败: {str(e)}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        get_tf_transform()
    except rospy.ROSInterruptException:
        pass