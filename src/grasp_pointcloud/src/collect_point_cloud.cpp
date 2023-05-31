#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include<ros/console.h>


using namespace std;

void collect_point_cloud(const std_msgs::String::ConstPtr& name)
{
    ROS_INFO("collect: [%s]", name->data.c_str());
    string path = "/home/jay/grasp_ws/src/grasp_pointcloud/script/volume_estimate/";
    // 保存点云
    sensor_msgs::PointCloud2ConstPtr pointcloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/color/points", ros::Duration(5));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*pointcloud_msg, *cloud);
    path += "pcd/"+name->data+".pcd";
    // ROS_INFO_STREAM(path);
    pcl::io::savePCDFileASCII(path, *cloud);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "collect_point_cloud");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("start collect pointcloud");
    ros::Subscriber sub = nh.subscribe ("collect/collectable", 1, collect_point_cloud);
    // Spin
    ros::spin ();
}