//
//  PCLVisualization.hpp
//  PCL
//
//  Created by Jaromír Landa on 21/10/2021.
//

#ifndef PCLVisualization_hpp
#define PCLVisualization_hpp

#include <stdio.h>
#include <iostream>

#include <ctime>

#include <cstdlib>

#include "Color.hpp"

#include <vector>




#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>

#include <pcl/features/normal_3d.h>

#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/parse.h>

#include <pcl/surface/gp3.h>

#include <Eigen/src/Core/Array.h>




class PCLVisualization {
private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
    int m_numberOfClouds;
public:
    PCLVisualization();
    void runVisualization();
    void addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Color pointsColor);
    void addCloudWithRandomColor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void initializeVisualization();
    
};



#endif /* PCLVisualization_hpp */
