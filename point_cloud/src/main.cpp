#include <iostream>
#include <thread>

#include "PLYLoader.hpp"
#include "PCLVisualization.hpp"
#include "GroundExtractor.hpp"
#include "EuclidianClusterSegmentation.hpp"
#include <pcl/filters/voxel_grid.h>



int main (int argc, char** argv)
{
 
    PLYLoader loader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loader.loadCloud("urban_cloud.ply");
    
    std::cout << "Input cloud points: " << cloud->points.size() <<std::endl;
    
    GroundExtractor extractor;
    extractor.extractGround(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground = extractor.getGround();
    pcl::PointCloud<pcl::PointXYZ>::Ptr everythingElse = extractor.getEverythingElse();
    
    
    EuclidianClusterSegmentation segmentation;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedClouds = segmentation.segmentCloud(everythingElse);
    
    
    PCLVisualization visualization;
    visualization.initializeVisualization();
    
    
    for (const auto& cloud : segmentedClouds){
        visualization.addCloudWithRandomColor(cloud);
    }
     
    //visualization.addCloud(everythingElse, Color(255, 0, 0));
    visualization.runVisualization();
    
    return 0;
}
