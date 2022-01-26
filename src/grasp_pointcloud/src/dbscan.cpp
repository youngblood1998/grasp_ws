#include <iostream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ pointT;
typedef pcl::PointCloud<pointT> cloud;

bool dbscan(const cloud::Ptr& cloud_in, std::vector<std::vector<int> > &clusters_index, const double& r, const int& size)
{
    std::cout<<r<<endl;
    std::cout<<size<<endl;
    if (!cloud_in->size())
        return false;
    //LOG()
    pcl::KdTreeFLANN<pointT> tree;
    tree.setInputCloud(cloud_in);
    std::vector<bool> cloud_processed(cloud_in->size(), false);

    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        if (cloud_processed[i])
        {
            continue;
        }

        std::vector<int>seed_queue;
        //检查近邻数是否大于给定的size（判断是否是核心对象）
        std::vector<int> indices_cloud;
        std::vector<float> dists_cloud;
        if (tree.radiusSearch(cloud_in->points[i], r, indices_cloud, dists_cloud) >= size)
        {
            seed_queue.push_back(i);
            cloud_processed[i] = true;
        }
        else
        {
            //cloud_processed[i] = true;
            continue;
        }

        int seed_index = 0;
        while (seed_index < seed_queue.size())
        {
            std::vector<int> indices;
            std::vector<float> dists;
            if (tree.radiusSearch(cloud_in->points[seed_queue[seed_index]], r, indices, dists) < size)//函数返回值为近邻数量
            {
                //cloud_processed[i] = true;//不满足<size可能是边界点，也可能是簇的一部分，不能标记为已处理
                ++seed_index;
                continue;
            }
            for (size_t j = 0; j < indices.size(); ++j)
            {
                if (cloud_processed[indices[j]])
                {
                    continue;
                }
                seed_queue.push_back(indices[j]);
                cloud_processed[indices[j]] = true;
            }
            ++seed_index;                
        }
        clusters_index.push_back(seed_queue);
      
    }
   // std::cout << clusters_index.size() << std::endl;

    if (clusters_index.size())
        return true;
    else
        return false;
}

int main(int argc, char **argv)
{
    cloud::Ptr cloud_in(new cloud);
    std::vector<std::vector<int> > clusters_index;
    std::string name = argv[1];
    std::string path = "/home/jay/grasp_ws/src/grasp_pointcloud/pcd_treat/" + name + " treat.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud_in);
    dbscan(cloud_in, clusters_index, 0.0015, 9);
    // dbscan(cloud_in, clusters_index, atof(argv[2]), atof(argv[3]));
    pcl::PointCloud<pcl::PointXYZRGB> visual_cloud;
    visual_cloud.width = cloud_in->size();
    visual_cloud.height = 1;
    visual_cloud.resize(cloud_in->size());
    int cluster_num = 0;
    for (size_t i = 0; i < clusters_index.size(); ++i)
    {
        if(clusters_index[i].size()>100 && clusters_index[i].size()<500){
            uint8_t r = rand()%(256);
            uint8_t g = rand()%(256);
            uint8_t b = rand()%(256);
            int32_t rgb = (r << 16) | (g << 8) | b; 
            // std::cout<<rgb<<endl;
            for (size_t j = 0; j < clusters_index[i].size(); ++j)
            {
                visual_cloud.points[clusters_index[i][j]].x = cloud_in->points[clusters_index[i][j]].x;
                visual_cloud.points[clusters_index[i][j]].y = cloud_in->points[clusters_index[i][j]].y;
                visual_cloud.points[clusters_index[i][j]].z = cloud_in->points[clusters_index[i][j]].z;
                // visual_cloud.points[clusters_index[i][j]].intensity = color;
                visual_cloud.points[clusters_index[i][j]].rgb = *(float *)(&rgb);
                //std::cout << clusters_index[i][j] << std::endl;
            }
            cluster_num++;
        // std::cout << clusters_index[i].size() << std::endl;
        }else{
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;
            int32_t rgb = (r << 16) | (g << 8) | b;
            for (size_t j = 0; j < clusters_index[i].size(); ++j)
            {
                visual_cloud.points[clusters_index[i][j]].x = cloud_in->points[clusters_index[i][j]].x;
                visual_cloud.points[clusters_index[i][j]].y = cloud_in->points[clusters_index[i][j]].y;
                visual_cloud.points[clusters_index[i][j]].z = cloud_in->points[clusters_index[i][j]].z;
                // visual_cloud.points[clusters_index[i][j]].intensity = color;
                visual_cloud.points[clusters_index[i][j]].rgb = *(float *)(&rgb);
                //std::cout << clusters_index[i][j] << std::endl;
            }
        }
    }
    std::cout<<cluster_num<<endl;
    pcl::visualization::CloudViewer viewer("DBSCAN cloud viewer.");
    viewer.showCloud(visual_cloud.makeShared());
    while (!viewer.wasStopped())
    {
        // viewer->spinOnce();
    }
    // pcl::io::savePCDFile("dbscan.pcd", visual_cloud,true);
}