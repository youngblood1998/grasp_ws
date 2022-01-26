/**
 * @file don_segmentation.cpp
 * Difference of Normals Example for PCL Segmentation Tutorials.
 *
 * @author Yani Ioannou
 * @date 2012-09-24
 */
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;

int main (int argc, char *argv[])
{
    ///The smallest scale to use in the DoN filter.
    double scale1;
    ///The largest scale to use in the DoN filter.
    double scale2;
    ///The minimum DoN magnitude to threshold by
    double threshold;
    ///segment scene into clusters with given distance tolerance using euclidean clustering
    double segradius;

    if (argc < 6)
    {
    std::cerr << "usage: " << argv[0] << " inputfile smallscale largescale threshold segradius" << std::endl;
    exit (EXIT_FAILURE);
    }

    std::string name = argv[1];
    std::string infile = "/home/jay/grasp_ws/src/grasp_pointcloud/pcd_treat/" + name + " treat.pcd";
    /// the file to read from.
    // std::string infile = argv[1];
    /// small scale
    std::istringstream (argv[2]) >> scale1;
    /// large scale
    std::istringstream (argv[3]) >> scale2;
    std::istringstream (argv[4]) >> threshold;   // threshold for DoN magnitude
    std::istringstream (argv[5]) >> segradius;   // threshold for radius segmentation

    // Load cloud in blob format
    // pcl::PCLPointCloud2 blob;
    // pcl::io::loadPCDFile (infile.c_str (), blob);
    pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
    // pcl::fromPCLPointCloud2 (blob, *cloud);
    pcl::io::loadPCDFile (infile, *cloud);

    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<PointXYZRGB>::Ptr tree;
    if (cloud->isOrganized ())
    {
    tree.reset (new pcl::search::OrganizedNeighbor<PointXYZRGB> ());
    }
    else
    {
    tree.reset (new pcl::search::KdTree<PointXYZRGB> (false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud (cloud);

    if (scale1 >= scale2)
    {
    std::cerr << "Error: Large scale must be > small scale!" << std::endl;
    exit (EXIT_FAILURE);
    }

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);

    /**
    * NOTE: setting viewpoint is very important, so that we can ensure
    * normals are all pointed in the same direction!
    */
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    // calculate normals with the small scale
    std::cout << "Calculating normals for scale..." << scale1 << std::endl;
    pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

    ne.setRadiusSearch (scale1);
    ne.compute (*normals_small_scale);

    // calculate normals with the large scale
    std::cout << "Calculating normals for scale..." << scale2 << std::endl;
    pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

    ne.setRadiusSearch (scale2);
    ne.compute (*normals_large_scale);

    // Create output cloud for DoN results
    PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
    copyPointCloud (*cloud, *doncloud);

    std::cout << "Calculating DoN... " << std::endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
    don.setInputCloud (cloud);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    if (!don.initCompute ())
    {
    std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature (*doncloud);

    // Save DoN features
    // pcl::PCDWriter writer;
    // std::string donfile = "/home/jay/grasp_ws/src/grasp_pointcloud/pcd_treat/" + name + " don.pcd";
    // writer.write<pcl::PointNormal> (donfile, *doncloud, false); 

    // Filter by magnitude
    std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;

    // Build the condition for filtering
    pcl::ConditionOr<PointNormal>::Ptr range_cond (
    new pcl::ConditionOr<PointNormal> ()
    );
    range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                                new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                                );
    // Build the filter
    pcl::ConditionalRemoval<PointNormal> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (doncloud);

    pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

    // Apply filter
    condrem.filter (*doncloud_filtered);

    doncloud = doncloud_filtered;

    // Save filtered output
    std::cout << "Filtered Pointcloud: " << doncloud->size () << " data points." << std::endl;

    // writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

    // Filter by magnitude
    std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << std::endl;

    pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
    segtree->setInputCloud (doncloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointNormal> ec;

    ec.setClusterTolerance (segradius);
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (500);
    ec.setSearchMethod (segtree);
    ec.setInputCloud (doncloud);
    ec.extract (cluster_indices);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // 为了从点云索引向量中分割出每个簇，必须迭代访问点云索引，
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {

        // 每次创建一个新的点云数据集，并且将所有当前簇的点写入到点云数据集中。
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        const std::vector<int> &indices = it->indices;

        for (std::vector<int>::const_iterator pit = indices.begin(); pit != indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]);

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
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> single_color(cloud_cluster);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud_cluster, single_color, ss.str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());

        j++;
    }
    std::cout << "cloud size: " << cluster_indices.size() << std::endl;

    viewer->addCoordinateSystem(0.5);
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return (0);

    // int j = 0;
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
    // {
    // pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
    // for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    // {
    //     cloud_cluster_don->points.push_back ((*doncloud)[*pit]);
    // }

    // cloud_cluster_don->width = cloud_cluster_don->size ();
    // cloud_cluster_don->height = 1;
    // cloud_cluster_don->is_dense = true;

    // //Save cluster
    // std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->size () << " data points." << std::endl;
    // std::stringstream ss;
    // ss << "don_cluster_" << j << ".pcd";
    // writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
    // }
}