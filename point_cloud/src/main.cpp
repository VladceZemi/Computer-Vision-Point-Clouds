#include <iostream>
#include <thread>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/visualization/keyboard_event.h>

#include "PLYLoader.hpp"
#include "PCLVisualization.hpp"
#include "EuclidianClusterSegmentation.hpp"
#include "Roofer.hpp"

long currentSegment = 6;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudSegments;
PCLVisualization visualization;

void keyboardEventCallback(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
void roofAndVisualizeSegment();

int main (int argc, char** argv)
{
    PLYLoader loader;
    cloud = loader.loadCloud("../data/segmented.ply");

    EuclidianClusterSegmentation segmentation;
    cloudSegments = segmentation.segmentCloud(cloud);

    visualization.initializeVisualization();
    visualization.m_viewer->registerKeyboardCallback(keyboardEventCallback, (void *)&visualization.m_viewer);
//    visualization.addCloudWithRandomColor()
    visualization.runVisualization();

//    visualization.m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "Sp1");

    return 0;
}

void roofAndVisualizeSegment() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr segment = cloudSegments.at(currentSegment);
    Roofer roofer(segment);
    roofer.roof();

    visualization.m_viewer->removeAllPointClouds();
    visualization.m_viewer->removeAllShapes();

    visualization.addCloudWithRandomColor(segment);
    roofer.visualize(visualization.m_viewer);
}

void keyboardEventCallback(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);

    if (event.keyUp()) {
        if (event.getKeyCode() == 97) {
            currentSegment -= 1;
            if (currentSegment < 0) {
                currentSegment = cloudSegments.size() - 1;
            }
            roofAndVisualizeSegment();
        }

        if (event.getKeyCode() == 100) {
            currentSegment += 1;
            if (currentSegment >= cloudSegments.size()) {
                currentSegment = 0;
            }
            roofAndVisualizeSegment();
        }

        std::cout << currentSegment << std::endl;

    }
}
