#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <time.h>
#include <string>
// #include<iostream>

using namespace std;

// ros::Publisher pub;
// pcl::visualization::CloudViewer viewer("Cloud Viewer");

void collect_data (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.
    // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    // pcl_conversions::toPCL(*input, *cloud);
    // pcl::io::savePCDFileASCII("test_pcd.pcd", *cloud);

    // sensor_msgs::PointCloud2 output;
    // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // output = *input;
    // pcl::fromROSMsg(output, *cloud);
    // pcl::io::savePCDFileASCII("./src/grasp_pointcloud/pcd/test_pcd.pcd", *cloud);
    // viewer.showCloud(cloud);

    char buf[128]= {0};
    time_t t = time(NULL); //获取目前秒时间
    tm* local = localtime(&t); //转为本地时间
    strftime(buf, 64, "%Y-%m-%d %H:%M:%S", local);
    string s = buf;
    string path = "./src/grasp_pointcloud/pcd/" + s + ".pcd";
    cout<<path<<endl;

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    output = *input;
    pcl::fromROSMsg(output, *cloud);
    pcl::io::savePCDFileASCII(path, *cloud);

    exit(0);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "collect_data");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, collect_data);
    // Spin
    ros::spin ();
}