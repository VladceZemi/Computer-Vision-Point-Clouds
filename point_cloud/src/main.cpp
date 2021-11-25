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


    // for (const auto& cloud : segmentedClouds){
    //     visualization.addCloudWithRandomColor(cloud);
    // }




    pcl::PointCloud<pcl::PointXYZ>::Ptr c = segmentedClouds.at(13);

    float zMax = 0;

    for (int i = 0; i < c->size(); i++){
        std::cout << c->at(i).z << std::endl;
        if (zMax < c->at(i).z){
            zMax = c->at(i).z;
        }
    }
    std::cout << "Z max: " << zMax << std::endl;

    // filtrace pomoci z

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCbyZ(new pcl::PointCloud<pcl::PointXYZ>());

    float toleration = 1;

    for (int i = 0; i < c->size(); i++){
        if (zMax-toleration < c->at(i).z){
            filteredCbyZ->push_back(c->at(i));
        }
    }



    visualization.addCloudWithRandomColor(filteredCbyZ);
    visualization.runVisualization();

    return 0;
}
