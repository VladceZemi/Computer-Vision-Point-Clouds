#include "EuclidianClusterSegmentation.hpp"

EuclidianClusterSegmentation::EuclidianClusterSegmentation(long minSize, float tolerance) {
  m_minSize = minSize;
  m_tolerance = tolerance;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
EuclidianClusterSegmentation::segmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud) {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedClouds;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdTree->setInputCloud(inputCloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(m_tolerance);
    extraction.setMinClusterSize(m_minSize);
    extraction.setMaxClusterSize(100000);
    extraction.setSearchMethod(kdTree);
    extraction.setInputCloud(inputCloud);
    extraction.extract(clusterIndices);

    for(std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

          for (const auto& idx : it->indices){
            cluster->push_back((*inputCloud)[idx]);
          }
          cluster->width = cluster->size();
          cluster->height = 1;
          cluster->is_dense = true;
          segmentedClouds.push_back(cluster);
    }
    return segmentedClouds;

}
