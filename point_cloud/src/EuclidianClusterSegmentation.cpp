#include "EuclidianClusterSegmentation.hpp"

EuclidianClusterSegmentation::EuclidianClusterSegmentation() {}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
EuclidianClusterSegmentation::segmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud) {

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedClouds;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree (new pcl::search::KdTree<pcl::PointXYZ>);
    kdTree->setInputCloud (inputCloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance (1.2);
    extraction.setMinClusterSize (100);
    extraction.setMaxClusterSize (100000);
    extraction.setSearchMethod (kdTree);
    extraction.setInputCloud (inputCloud);
    extraction.extract(clusterIndices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
      {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for (const auto& idx : it->indices){
            cluster->push_back ((*inputCloud)[idx]);
          }
          cluster->width = cluster->size();
          cluster->height = 1;
          cluster->is_dense = true;
          //std::cout << "Mračno má: " << cluster->size () << " bodů." << std::endl;
          j++;
          segmentedClouds.push_back(cluster);
      }
    return segmentedClouds;

}
