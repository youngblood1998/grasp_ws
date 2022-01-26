#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <string>

int main (int argc, char** argv){
    std::string name = argv[1];
    std::string path = "/home/jay/grasp_ws/src/grasp_pointcloud/pcd_treat/" + name + " treat.pcd";
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGBA> (path, *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    pcl::search::Search<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (20);
    normal_estimator.compute (*normals);

    //   pcl::IndicesPtr indices (new std::vector <int>);
    //   pcl::PassThrough<pcl::PointXYZRGBA> pass;
    //   pass.setInputCloud (cloud);
    //   pass.setFilterFieldName ("z");
    //   pass.setFilterLimits (0.0, 1.0);
    //   pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> reg;
    reg.setMinClusterSize (100);
    reg.setMaxClusterSize (500);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (10);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (4.2 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    // std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    // std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
    // std::cout << "These are the indices of the points of the initial" <<
    //     std::endl << "cloud that belong to the first cluster:" << std::endl;
    // int counter = 0;
    // while (counter < clusters[0].indices.size ())
    // {
    //     std::cout << clusters[0].indices[counter] << ", ";
    //     counter++;
    //     if (counter % 10 == 0)
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;

    // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    // pcl::visualization::CloudViewer viewer ("Cluster viewer");
    // viewer.showCloud(colored_cloud);
    // while (!viewer.wasStopped ())
    // {
    // }

    // return (0);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // 为了从点云索引向量中分割出每个簇，必须迭代访问点云索引，
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin();
         it != clusters.end(); ++it) {

        // 每次创建一个新的点云数据集，并且将所有当前簇的点写入到点云数据集中。
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
        const std::vector<int> &indices = it->indices;

        for (std::vector<int>::const_iterator pit = indices.begin(); pit != indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Clusters: " << cloud_cluster->points.size() << " data points."
                  << std::endl;
        /*
            std::stringstream ss;
            ss << "cloud_cluster_" << j << ".pcd";
            writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //
        */
        std::stringstream ss;
        ss << "cloud_cluster_" << j;
        // Generate a random (bright) color
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGBA> single_color(cloud_cluster);
        viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_cluster, single_color, ss.str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());

        j++;
    }
    std::cout << "cloud size: " << clusters.size() << std::endl;

    viewer->addCoordinateSystem(0.5);
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return (0);
}