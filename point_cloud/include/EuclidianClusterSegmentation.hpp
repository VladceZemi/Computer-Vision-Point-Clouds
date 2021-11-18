#ifndef EuclidianClusterSegmentation_hpp
#define EuclidianClusterSegmentation_hpp

#include <stdio.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class EuclidianClusterSegmentation {
  
public:
    EuclidianClusterSegmentation();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    segmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
};

#endif /* EuclidianClusterSegmentation_hpp */
