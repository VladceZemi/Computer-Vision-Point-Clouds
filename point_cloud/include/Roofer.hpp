#ifndef ROOFER_H
#define ROOFER_H

#include <cmath>
#include <tuple>
#include <limits>

#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "EuclidianClusterSegmentation.hpp"

class Roofer {
private:
    const float TOLERATION = 2.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    std::vector<pcl::PointXYZ> m_roofPoints;

    pcl::PointXYZ getMaxZPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointXYZ getMaxZPointNear(pcl::PointXYZ point, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getRoofRidge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::tuple<pcl::PointXYZ, pcl::PointXYZ> getFarthestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void getCornerPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
public:
    Roofer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void roof();
    void visualize(const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);
};

#endif