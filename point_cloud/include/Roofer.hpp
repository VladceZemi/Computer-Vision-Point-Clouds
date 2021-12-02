#ifndef ROOFER_H
#define ROOFER_H

#include <cmath>
#include <tuple>
#include <limits>
#include <map>

#include <pcl/common/centroid.h>
#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "EuclidianClusterSegmentation.hpp"

class Roofer {
private:
    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::min();
    float maxY = std::numeric_limits<float>::min();
    float maxZ = std::numeric_limits<float>::min();

    const float TOLERATION = 0.5;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    pcl::CentroidPoint<pcl::PointXYZ> m_centroid;
    std::vector<pcl::PointXYZ> m_roofPoints;

    pcl::PointXYZ getMaxZPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointXYZ getMinZPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointXYZ getMaxZPointNear(pcl::PointXYZ point, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterRoofNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getRoofRidge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getRoofBottom(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::tuple<pcl::PointXYZ, pcl::PointXYZ> getFarthestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void getCornerPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fromRidgesAndWholeCluster();
public:
    Roofer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void roof();
    void visualize(const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);
};

#endif