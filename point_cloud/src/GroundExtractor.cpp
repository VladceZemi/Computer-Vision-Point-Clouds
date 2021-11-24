#include "GroundExtractor.hpp"

GroundExtractor::GroundExtractor() {}

void GroundExtractor::extractGround(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr otherPoints (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointIndicesPtr groundIndicies (new pcl::PointIndices);

    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> filter;
    filter.setInputCloud(inputCloud);
    filter.setMaxWindowSize(100);
    filter.setSlope(10.0f);
    filter.setInitialDistance(5.0f);
    filter.setMaxDistance(10.0f);

    filter.extract(groundIndicies->indices);

    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(inputCloud);
    extractor.setIndices(groundIndicies);
    extractor.filter(*groundPoints);

    extractor.setNegative(true);
    extractor.filter(*otherPoints);

    ground = groundPoints;
    everyrhingElse = otherPoints;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr GroundExtractor::getGround(){
    return ground;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GroundExtractor::getEverythingElse(){
    return everyrhingElse;
}
