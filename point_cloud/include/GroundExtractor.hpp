#ifndef GroundExtractor_hpp
#define GroundExtractor_hpp

#include <stdio.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/io/auto_io.h>


class GroundExtractor{
public:
    GroundExtractor();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getGround();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getEverythingElse();
    void extractGround(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
    void saveExtractedCloudPLY();
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground;
    pcl::PointCloud<pcl::PointXYZ>::Ptr everyrhingElse;
};


#endif /* GroundExtractor_hpp */
