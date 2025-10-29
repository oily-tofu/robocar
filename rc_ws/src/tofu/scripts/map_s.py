#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np

def pointcloud_callback(msg):
    rospy.loginfo("接收到地图点云，正在保存 map.pcd...")

    # 提取 XYZ 数据
    cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if len(cloud_points) == 0:
        rospy.logwarn("收到的点云为空")
        return

    xyz_array = np.array(cloud_points)[:, :3]

    # 转换为 Open3D 点云并保存
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz_array)
    o3d.io.write_point_cloud("/home/tofu/catkin1_ws/src/tofu/map/map2.pcd", pcd)

    rospy.loginfo("保存完成 map.pcd")
    rospy.signal_shutdown("地图保存完毕，退出程序")

def main():
    rospy.init_node('save_fastlio_map', anonymous=True)
    rospy.Subscriber("/cloud_registered", PointCloud2, pointcloud_callback)
    rospy.loginfo("等待话题 /cloud_registered ...")
    rospy.spin()

if __name__ == '__main__':
    main()



