#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

int main(int argc, char **argv) {
    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_f(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::string name = argv[1];
    std::string path = "/home/jay/grasp_ws/src/grasp_pointcloud/pcd_treat/" + name + " treat.pcd";
    reader.read(path, *cloud_filtered);
    std::cout << "PointCloud before filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*

    // Creating the KdTree object for the search method of the extraction
    // 为提取算法的搜索方法创建一个KdTree对象
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud_filtered);

    /**
     * 在这里，我们创建一个PointIndices的vector，该vector在vector <int>中包含实际的索引信息。
     * 每个检测到的簇的索引都保存在这里-请注意，cluster_indices是一个vector，包含多个检测到的簇的PointIndices的实例。
     * 因此，cluster_indices[0]包含我们点云中第一个 cluster(簇)的所有索引。
     *
     * 从点云中提取簇（集群）,并将点云索引保存在 cluster_indices 中。
     */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.001); // 设置临近搜索的搜索半径（搜索容差）为0.1cm
    ec.setMinClusterSize(100);    // 每个簇（集群）的最小大小
    ec.setMaxClusterSize(500);  // 每个簇（集群）的最大大小
    ec.setSearchMethod(tree);     // 设置点云搜索算法
    ec.setInputCloud(cloud_filtered);   // 设置输入点云
    ec.extract(cluster_indices);        // 设置提取到的簇，将每个簇以索引的形式保存到cluster_indices;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // 为了从点云索引向量中分割出每个簇，必须迭代访问点云索引，
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {

        // 每次创建一个新的点云数据集，并且将所有当前簇的点写入到点云数据集中。
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
        const std::vector<int> &indices = it->indices;

        for (std::vector<int>::const_iterator pit = indices.begin(); pit != indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
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
    std::cout << "cloud size: " << cluster_indices.size() << std::endl;

    viewer->addCoordinateSystem(0.5);
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return (0);
}