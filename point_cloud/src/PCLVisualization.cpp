//
//  PCLVisualization.cpp
//  PCL
//
//  Created by Jaromír Landa on 21/10/2021.
//

#include "PCLVisualization.hpp"

PCLVisualization::PCLVisualization() {
    m_numberOfClouds = 0;
}

void PCLVisualization::initializeVisualization(){
    // inicoalizace visuzalizeru
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // nasetuju ho
    m_viewer = viewer;
    // nastavim pozadi
    m_viewer->setBackgroundColor (0, 0, 0	);
}

void PCLVisualization::runVisualization(){
    m_viewer->initCameraParameters();
    m_viewer->resetCameraViewpoint("cloud0");
    while (!m_viewer->wasStopped ()) {

        m_viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    m_viewer->close();
}

void PCLVisualization::addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Color pointsColor){
    
    std::stringstream stream;
    stream << "cloud" << m_numberOfClouds;
    std::string cloudName = stream.str();
    
    m_viewer->addPointCloud<pcl::PointXYZ>(cloud, cloudName);
    
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)pointsColor.R/255, (double)pointsColor.G/255, (double)pointsColor.B/255, cloudName);
        
    
    m_numberOfClouds++;
    
}

void PCLVisualization::addCloudWithRandomColor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    
    std::stringstream stream;
    stream << "cloud" << m_numberOfClouds;
    std::string cloudName = stream.str();
    
    int r = (rand() % 255);
    int g = (rand() % 255);
    int b = (rand() % 255);
    
    m_viewer->addPointCloud<pcl::PointXYZ>(cloud, cloudName);
    
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)r/255, (double)g/255, (double)b/255, cloudName);
        	
    
    m_numberOfClouds++;
    
}


