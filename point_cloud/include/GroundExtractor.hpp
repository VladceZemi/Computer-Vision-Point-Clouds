#ifndef GroundExtractor_hpp
#define GroundExtractor_hpp

#include <stdio.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>


class GroundExtractor{
public:
    GroundExtractor();
    void extractGround(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getGround();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getEverythingElse();
    
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground;
    pcl::PointCloud<pcl::PointXYZ>::Ptr everyrhingElse;
    
    
    
};


#endif /* GroundExtractor_hpp */
