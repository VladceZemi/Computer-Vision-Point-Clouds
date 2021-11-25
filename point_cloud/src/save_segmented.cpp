#include <iostream>
#include "GroundExtractor.hpp"
#include "EuclidianClusterSegmentation.hpp"

void save_segmented() {
    PLYLoader loader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedClouds = loader.loadCloud("segmented.ply");

    std::cout << "Input cloud points: " << cloud->points.size() << std::endl;

    //GroundExtractor extractor;
    //extractor.extractGround(cloud);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr ground = extractor.getGround();
    //pcl::PointCloud<pcl::PointXYZ>::Ptr everythingElse = extractor.getEverythingElse();

    //EuclidianClusterSegmentation segmentation;
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedClouds = segmentation.segmentCloud(everythingElse);

    pcl::PointCloud<pcl::PointXYZ>::Ptr multiCloud(new pcl::PointCloud<pcl::PointXYZ>);
    int size = 0;

    for (const auto& cloud : segmentedClouds) {
        size += cloud->size();
    }

    multiCloud->width = size;
    multiCloud->height = 1;
    multiCloud->is_dense = true;

    for (const auto& cloud : segmentedClouds) {
        for (int i = 0; i < cloud->points.size(); i++) {
            multiCloud->points.push_back(cloud->points.at(i));
        }
    }

    pcl::io::save("segmented.ply", *multiCloud);
}
