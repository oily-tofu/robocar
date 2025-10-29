#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_pub");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;
    std::string file_path;
    nh.param<std::string>("file_path", file_path, "/home/dji/Desktop/RM/src/FAST_LIO/PCD/scans.pcd");
    
    pcl::io::loadPCDFile(file_path, cloud); // Load the PCD file

    // Iterate over each point and reverse its coordinates (example: invert x, y, and z)
    for (auto& point : cloud.points) {
        point.x = point.x; // Reverse the x-coordinate
        point.y = point.y; // Reverse the y-coordinate
        point.z = point.z; // Reverse the z-coordinate (if needed)
    }

    pcl::toROSMsg(cloud, output); // Convert PointCloud to ROS msg
    output.header.frame_id = "map"; // Set the frame_id

    ros::Rate loop_rate(1); // Set the loop rate
    while (ros::ok()) {
        pcl_pub.publish(output); // Publish the point cloud
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
