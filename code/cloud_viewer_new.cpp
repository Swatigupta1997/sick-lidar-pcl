
/*
* Author: Swati Gupta (gswati@seas.upenn.edu)
* References: https://pcl.readthedocs.io/projects/tutorials/en/latest/region_growing_segmentation.html#region-growing-segmentation
*             https://github.com/PointCloudLibrary/pcl/issues/1852
*/

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>
#include <string>
#include <pcl/filters/passthrough.h>


pcl::PointCloud <pcl::PointXYZRGB>::Ptr region_seg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud(*cloud, *indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (0);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  return colored_cloud;
}

// Can be modified to point to dataset dir
std::string basedir = "/home/swati/Documents/SICK/PCL/repo/sick-lidar-pcl/code/data_collect_22Mar/";

int main(int argc, const char **argv)
{
    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
    viewer.setBackgroundColor(0,0,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (argc != 3) {
        std::cout << "Usage: ./filter_point_cloud <x-limit> <y-limit>" << std::endl;
        return -1;
    }
    
    int t=1;
    while(1)
    {
       // Clear the view
       viewer.removeAllShapes();
       viewer.removeAllPointClouds();

       // Get the PCD file
       pcl::io::loadPCDFile <pcl::PointXYZ> (basedir+"record_"+std::to_string(t)+".pcd", *cloud);
       t++;

        // Filter point cloud data to remove far away points/walls
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-std::abs(std::stof(argv[1])), std::abs(std::stof(argv[1]))); // clamp points to the range of [-x, x] in the x-axis
        pass.filter(*cloud);

        pass.setInputCloud(cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-std::abs(std::stof(argv[2])), std::abs(std::stof(argv[2]))); // clamp points to the range of [-y, y] in the y-axis
        pass.filter(*cloud);


       // Apply seg
       pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud  = region_seg(cloud);

       
        // The pointcloud
        viewer.addPointCloud<pcl::PointXYZRGB> (colored_cloud, "Nube cargada");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Nube cargada");         

        viewer.spinOnce(100);
     }
}