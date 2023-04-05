#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "pcl/kdtree/kdtree_flann.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
int user_data;
    
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
    
}
    
void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}
    
int main (int argc, char* argv[])
{
    if (argc != 4) {
        std::cout << "Usage: ./filter_point_cloud <x-limit> <y-limit> <z-limit>" << std::endl;
        return -1;
    }
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
  
    viewer.runOnVisualizationThread (viewerPsycho);
    int t=1;

    while (!viewer.wasStopped ()) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

        // cloud_filtered->resize(10000);


        // Load point cloud data from file
        // pcl::io::loadPCDFile ("data/record_"+ std::to_string(t)+".pcd", *cloud);
        pcl::io::loadPCDFile ("data_collect_22Mar/record_"+ std::to_string(t)+".pcd", *cloud);

        //         pcl::getMinMax3D	(cloud, "", 0, 100
        // float 	min_distance,
        // float 	max_distance,
        // Eigen::Vector4f & 	min_pt,
        // Eigen::Vector4f & 	max_pt,
        // bool 	limit_negative = false 
        // )	

        // Filter point cloud data
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-std::abs(std::stof(argv[1])), std::abs(std::stof(argv[1]))); // clamp points to the range of [-x, x] in the x-axis
        pass.filter(*cloud_filtered);

        cout << "Im here 1" << endl;

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-std::abs(std::stof(argv[2])), std::abs(std::stof(argv[2]))); // clamp points to the range of [-y, y] in the y-axis
        pass.filter(*cloud_filtered);

        cout << "Im here 2" << endl;

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, std::abs(std::stof(argv[3]))); // clamp points to the range of [0.0, z] in the z-axis
        pass.filter(*cloud_filtered);

        cout << "Im here 3" << endl;

        // viewer.showCloud(cloud_filtered);
        // viewer.removeAllPointClouds();

        pcl::io::savePCDFileASCII("filtered_data/record_"+ std::to_string(t)+".pcd", *cloud_filtered);

        cout << "Im here 4" << endl;

        t++;
        user_data++;

        cloud.reset();
        cloud_filtered.reset();
        cout << "t:" << t << endl;
        

        // Reset time step to 1 if it exceeds 100
        if (t >= 100) {
            // t = 1;
            break;
        }
    }
    return 0;
}