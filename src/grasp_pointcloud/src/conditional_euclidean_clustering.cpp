#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <Eigen/Dense>
#include <cstdlib>

typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

double getAngleTwoVectors(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2) {
  double radian_angle = atan2(v1.cross(v2).norm(), v1.transpose() * v2);
  return radian_angle;   //[0,PI]
}

bool customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
  Eigen::Vector3f v1(point_a_normal[0], point_a_normal[1], point_a_normal[2]),
  v2(point_b_normal[0], point_b_normal[1], point_b_normal[2]);

  float Angle = getAngleTwoVectors(v1, v2);
  if (squared_distance < 0.0000001)
  {
    cout<<squared_distance<<"\t"<<Angle<<endl;
    if (std::abs (Angle) < 0.08f){
      return true;
    }
    return (false);
  }
  else
  {
    cout<<squared_distance<<"\t"<<Angle<<endl;
    return (false);
  }
}

int main (int argc, char** argv)
{
    // Data containers used
    pcl::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
    pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);

    std::string name = argv[1];
    std::string path = "/home/jay/grasp_ws/src/grasp_pointcloud/pcd_treat/" + name + " treat.pcd";

    // Load the input point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile (path, *cloud);
    pcl::copyPointCloud(*cloud, *cloud_in);

    // Set up a Normal Estimation class and merge data in cloud_with_normals
    pcl::copyPointCloud (*cloud_in, *cloud_with_normals);
    pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
    ne.setInputCloud (cloud_in);
    ne.setSearchMethod (search_tree);
    ne.setRadiusSearch (0.01);
    ne.compute (*cloud_with_normals);

    // Set up a Conditional Euclidean Clustering class
    pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
    cec.setInputCloud (cloud_with_normals);
    cec.setConditionFunction (&customRegionGrowing);
    cec.setClusterTolerance (0.001);
    cec.setMinClusterSize (100);
    cec.setMaxClusterSize (500);
    cec.segment (*clusters);
    cec.getRemovedClusters (small_clusters, large_clusters);

    std::cout<<clusters->size()<<endl;
    std::cout<<small_clusters->size()<<endl;
    std::cout<<large_clusters->size()<<endl;

    // Using the intensity channel for lazy visualization of the output
    for (int i = 0; i < small_clusters->size (); ++i)
    {
        for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
        {
            (*cloud_in)[(*small_clusters)[i].indices[j]].intensity = -2.0;
        }
    }
    for (int i = 0; i < large_clusters->size (); ++i)
    {
        for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
        {
            (*cloud_in)[(*large_clusters)[i].indices[j]].intensity = +10.0;
        }
    }
    for (int i = 0; i < clusters->size (); ++i)
    {
        int label = rand () % 8;
        for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
        {
            (*cloud_in)[(*clusters)[i].indices[j]].intensity = label;
        }
    }

    std::string path1 = "/home/jay/grasp_ws/src/grasp_pointcloud/pcd_treat/" + name + " cec.pcd";
    pcl::io::savePCDFileASCII(path1, *cloud_in);

    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->addPointCloud<pcl::PointXYZI> (cloud_in, "sample cloud");
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    // viewer->addCoordinateSystem(0.5);
    // while (!viewer->wasStopped()) {
    //     viewer->spinOnce();
    // }

    return (0);
}