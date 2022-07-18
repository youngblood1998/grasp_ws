#include <ros/ros.h>
#include <iostream>
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "grasp_pointcloud/PointBoundingBox.h"


using namespace std;
using namespace sensor_msgs;
// using namespace message_filters;
using namespace grasp_pointcloud;


// // 测试无法跳转到回调函数
// void callback(const PointCloud2::ConstPtr& pointcloud, const PointBoundingBox::ConstPtr& boundingbox)
// {
//     cout<<"hello"<<endl;
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "grasp_generator");
//     cout<<"start!"<<endl;
//     ros::NodeHandle nh;
//     message_filters::Subscriber<PointCloud2> pointcloud_sub(nh, "/camera/depth/color/points", 1);
//     message_filters::Subscriber<PointBoundingBox> boundingboox_sub(nh, "/real_detect/PointBoundingBox", 1);
//     typedef message_filters::sync_policies::ApproximateTime<PointCloud2, PointBoundingBox> MySyncPolicy;
//     message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), pointcloud_sub, boundingboox_sub);
//     sync.registerCallback(boost::bind(&callback, _1, _2));
//     // sync.registerCallback(&callback);
//     ros::spin();
// }


// class Grasp_generator
// {
// public:
//     Grasp_generator();
//     virtual ~Grasp_generator();

//     static void point_callback(const PointCloud2::ConstPtr& point)
//     {
//         cout<<"point"<<endl;
//     }

//     static void bound_callback(const PointBoundingBox::ConstPtr& bound)
//     {
//         cout<<"bound"<<endl;
//     }

// private:
//     /* data */
//     ros::NodeHandle nh;
//     ros::Subscriber point_sub = nh.subscribe("/camera/depth/color/points", 1, this->point_callback);
//     ros::Subscriber bound_sub = nh.subscribe("/real_detect/PointBoundingBox", 1, this->bound_callback);
//     PointCloud2 point_cloud;
//     PointBoundingBox bounding_box;
// };


pcl::PointCloud<pcl::PointXYZ> global_cloud;


// 分开订阅测试
void point_callback(const PointCloud2::ConstPtr& input)
{
    // ros转pcl
    pcl::fromROSMsg(*input, global_cloud);
}

void bound_callback(const PointBoundingBox::ConstPtr& bound)
{
    cout<<"get bound"<<endl;
    // pcl::passthrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(global_cloud);
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_generator");
    cout<<"start"<<endl;
    ros::NodeHandle nh;
    ros::Subscriber point_sub = nh.subscribe("/camera/depth/color/points", 1, point_callback);
    ros::Subscriber bound_sub = nh.subscribe("/real_detect/PointBoundingBox", 1, bound_callback);
    // Grasp_generator grasp_generator();
    ros::spin();
}