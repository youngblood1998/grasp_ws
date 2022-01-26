#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

int main(int argc, char **argv) {
    // 创建PointCloud的智能指针
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // 加载pcd文件到cloud
    string name = argv[1];
    string path = "/home/jay/grasp_ws/src/grasp_pointcloud/pcd/" + name + ".pcd";
    pcl::io::loadPCDFile(path, *cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //这里会一直阻塞直到点云被渲染

    //直通滤波
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.2, 0.2);
    pass.filter(*cloud_filter);
    pass.setInputCloud(cloud_filter);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.2, 0.2);
    pass.filter(*cloud_filter);

    //平面去除
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setMaxIterations(100);
    seg.setInputCloud (cloud_filter);
    seg.segment (*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud(cloud_filter);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filter);

    //离群点滤波
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud_filter);
    sor.setMeanK(10);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filter);

    string path1 = "/home/jay/grasp_ws/src/grasp_pointcloud/pcd_treat/" + name + " treat.pcd";
    pcl::io::savePCDFileASCII(path1, *cloud_filter);

    // viewer.showCloud(cloud_filter);

    // // 循环判断是否退出
    // while (!viewer.wasStopped()) {
    //     // 你可以在这里对点云做很多处理
    // }
    return 0;
}