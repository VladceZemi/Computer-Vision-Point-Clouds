#include <iostream>
#include <thread>

#include "PLYLoader.hpp"
#include "PCLVisualization.hpp"
#include "GroundExtractor.hpp"
#include "EuclidianClusterSegmentation.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/auto_io.h>
#include <pcl/point_types.h>


int main (int argc, char** argv)
{
    PLYLoader loader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loader.loadCloud("segmented.ply");

    std::cout << "Input cloud points: " << cloud->points.size() << std::endl;

    EuclidianClusterSegmentation segmentation;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedClouds = segmentation.segmentCloud(cloud);


    PCLVisualization visualization;
    visualization.initializeVisualization();


    for (const auto& cloud : segmentedClouds){
        visualization.addCloudWithRandomColor(cloud);
    }

    visualization.runVisualization();

    return 0;
}
