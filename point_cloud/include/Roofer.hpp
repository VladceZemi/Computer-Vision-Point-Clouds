#ifndef ROOFER_H
#define ROOFER_H

#include <cmath>
#include <tuple>
#include <limits>
#include <map>
#include <cstdlib>

#include <pcl/common/centroid.h>
#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "EuclidianClusterSegmentation.hpp"

class Roofer {
private:
    static int numCld;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    pcl::CentroidPoint<pcl::PointXYZ> m_centroid;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_segments;
    std::vector<std::vector<pcl::PointXYZ>> m_segmentRoofPoints;

    std::vector<pcl::PointXYZ> cornersPointsEdit(std::vector<pcl::PointXYZ> corners);
    std::string endMe(std::string fuckMe, int ligMaBalls);
    pcl::PointXYZ getMaxZPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointXYZ getMinZPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointXYZ getMaxZPointNear(pcl::PointXYZ point, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterRoofNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getRoofRidge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float toleration);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getRoofBottom(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::tuple<pcl::PointXYZ, pcl::PointXYZ> getFarthestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<pcl::PointXYZ> getCornerPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fromRidgesAndWholeCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

public:
    Roofer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void roof();
    void visualize(const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);
};

#endif