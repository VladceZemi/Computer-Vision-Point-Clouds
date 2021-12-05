#include "Roofer.hpp"

int Roofer::numCld = 0;

bool pointsEq(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    return (
        p1.x == p2.x &&
        p1.y == p2.y &&
        p1.z == p2.z
    );
}


Roofer::Roofer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    numCld += 1;
    for(auto point : *cloud) {
        m_centroid.add(point);
    }
    m_cloud = cloud;
}

void Roofer::roof() {
    m_cloud = filterRoofNoise(m_cloud);
    m_segments = fromRidgesAndWholeCluster(m_cloud);

    for (int i = 0; i < m_segments.size(); i++) {
        m_segmentRoofPoints.push_back(std::vector<pcl::PointXYZ>());
        auto segment = m_segments.at(i);
        auto roofRidge = getRoofRidge(segment,2);
        auto farthestPoints = getFarthestPoints(roofRidge);
        pcl::PointXYZ ridgePoint1 = getMaxZPointNear(std::get<0>(farthestPoints), 1.5, roofRidge);
        pcl::PointXYZ ridgePoint2 = getMaxZPointNear(std::get<1>(farthestPoints), 1.5, roofRidge);
        m_segmentRoofPoints.at(i).push_back(ridgePoint1);
        m_segmentRoofPoints.at(i).push_back(ridgePoint2);
        for (auto cornerPoint : getCornerPoints(segment)) {
            m_segmentRoofPoints.at(i).push_back(cornerPoint);
        }
    }
}

pcl::PointXYZ Roofer::getMaxZPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointXYZ zMaxPoint;
    float zMax = 0;

    for (int i = 0; i < cloud->size(); i++) {
        if (zMax < cloud->at(i).z) {
            zMax = cloud->at(i).z;
            zMaxPoint = cloud->at(i);
        }
    }
    return zMaxPoint;
}

pcl::PointXYZ Roofer::getMinZPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointXYZ zMinPoint;
    float zMin = std::numeric_limits<float>::max();

    for (int i = 0; i < cloud->size(); i++) {
        if (zMin > cloud->at(i).z) {
            zMin = cloud->at(i).z;
            zMinPoint = cloud->at(i);
        }
    }

    return zMinPoint;
}

pcl::PointXYZ Roofer::getMaxZPointNear(pcl::PointXYZ point, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::PointXYZ maxZPoint;

    kdtree.setInputCloud(cloud);
    if (kdtree.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) < 0) {
        return point;
    }

    for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
        if (cloud->at(pointIdxRadiusSearch.at(i)).z > maxZPoint.z) {
            maxZPoint = cloud->at(pointIdxRadiusSearch.at(i));
        }
    }

    return maxZPoint;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Roofer::filterRoofNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::map<int, int> histogram;

    for (auto point : *cloud) {
        float h = round(point.z);
        if (histogram.find(h) == histogram.end()) {
            histogram[h] = 0;
        }
        histogram[h] += 1;
    }

    for (auto it = histogram.begin(); it != histogram.end(); it ++) {
        std::cout << it->first << ": " << it->second << std::endl;
    }

    for (auto point : *cloud) {
        float h = round(point.z);
        if (histogram[h] > 20) {
            filteredCloud->push_back(point);
        }
    }

    return filteredCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Roofer::getRoofRidge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float toleration) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCbyZ(new pcl::PointCloud<pcl::PointXYZ>());
    float zMax = getMaxZPoint(cloud).z;

    for (int i = 0; i < cloud->size(); i++) {
        if (zMax - toleration < cloud->at(i).z) {
            filteredCbyZ->push_back(cloud->at(i));
        }
    }

    return filteredCbyZ;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr Roofer::getRoofBottom(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCbyZ(new pcl::PointCloud<pcl::PointXYZ>());
    float zMin = getMinZPoint(cloud).z;

    std::cout << "Pred filtraci bottom: " << cloud->size() << std::endl;

    for (int i = 0; i < cloud->size(); i++) {
        if ((zMin + 2) > cloud->at(i).z) {
            filteredCbyZ->push_back(cloud->at(i));
        }
    }

    return filteredCbyZ;
}

std::tuple<pcl::PointXYZ, pcl::PointXYZ> Roofer::getFarthestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    float maxEuclidLen = 0;
    pcl::PointXYZ point1;
    pcl::PointXYZ point2;

    for (int i = 0; i < cloud->size(); i++) {
        for (int j = 0; j < cloud->size(); j++) {
            float currEuclid = abs(cloud->at(i).x - cloud->at(j).x) + abs(cloud->at(i).y - cloud->at(j).y);
            if (maxEuclidLen < currEuclid) {
                point1 = cloud->at(i);
                point2 = cloud->at(j);
                maxEuclidLen = currEuclid;
            }
        }
    }

    return std::make_tuple(point1, point2);
}

std::vector<pcl::PointXYZ> Roofer::getCornerPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointXYZ firstCorner(std::numeric_limits<float>::max(), std::numeric_limits<float>::min(), 0);
    pcl::PointXYZ secondCorner(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0);
    pcl::PointXYZ thirdCorner(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), 0);
    pcl::PointXYZ fourthCorner(std::numeric_limits<float>::min(), std::numeric_limits<float>::max(), 0);

    for (auto point : *cloud) {
        if ((point.y - point.x) > (firstCorner.y - firstCorner.x)) {
            firstCorner = point;
        }

        if ((point.x + point.y)  < (secondCorner.x + secondCorner.y)) {
            secondCorner = point;
        }

        if ((point.x + point.y) > (thirdCorner.x + thirdCorner.y)) {
            thirdCorner = point;
        }

        if ((point.x - point.y) > (fourthCorner.x - fourthCorner.y))  {
            fourthCorner = point;
        }
    }

    std::vector<pcl::PointXYZ> cornerPoints;
    cornerPoints.push_back(firstCorner);
    cornerPoints.push_back(secondCorner);
    cornerPoints.push_back(thirdCorner);
    cornerPoints.push_back(fourthCorner);
    return cornerPoints;
}

std::string Roofer::endMe(std::string fuckMe, int ligMaBalls) {
    std::stringstream ss;
    ss << "c" << numCld << "s" << ligMaBalls << fuckMe;
    return ss.str();
}

void Roofer::visualize(const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer) {
    // pcl::PointXYZ center;
    // m_centroid.get(center);
    // viewer->addLine(center, pcl::PointXYZ(center.x + 10, center.y, center.z), 1.0, 0, 0, "X");
    // viewer->addLine(center, pcl::PointXYZ(center.x, center.y + 10, center.z), 0, 1.0, 0, "Y");
    // viewer->addLine(center, pcl::PointXYZ(center.x, center.y, center.z + 10), 0, 0, 1.0, "Z");
    viewer->addPointCloud(m_cloud, endMe("cloud", 100));
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, endMe("cloud", 100));
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, endMe("cloud", 100));

    for (int i = 0; i < m_segmentRoofPoints.size(); i++) {
        auto roofPoints = m_segmentRoofPoints.at(i);

        pcl::PointXYZ nearest01Point;
        pcl::PointXYZ nearest02Point;
        pcl::PointXYZ nearest11Point;
        pcl::PointXYZ nearest12Point;

        float dist0LT = abs(roofPoints.at(2).x - roofPoints.at(0).x) + abs(roofPoints.at(2).y - roofPoints.at(0).y);
        float dist1LT = abs(roofPoints.at(2).x - roofPoints.at(1).x) + abs(roofPoints.at(2).y - roofPoints.at(1).y);

        float dist0LD = abs(roofPoints.at(3).x - roofPoints.at(0).x) + abs(roofPoints.at(3).y - roofPoints.at(0).y);
        float dist1LD = abs(roofPoints.at(3).x - roofPoints.at(1).x) + abs(roofPoints.at(3).y - roofPoints.at(1).y);

        if (dist0LT < dist1LT) {
            if (dist0LD < dist1LD) {
                nearest01Point = roofPoints.at(2); //
                nearest02Point = roofPoints.at(3); //
                nearest11Point = roofPoints.at(4);
                nearest12Point = roofPoints.at(5);
            }
            else {
                nearest01Point = roofPoints.at(2); //
                nearest02Point = roofPoints.at(4);
                nearest11Point = roofPoints.at(3); //
                nearest12Point = roofPoints.at(5);
            }
        }
        else {
            if (dist0LD < dist1LD) {
                nearest01Point = roofPoints.at(3); //
                nearest02Point = roofPoints.at(5);
                nearest11Point = roofPoints.at(2); //
                nearest12Point = roofPoints.at(4);
            }
            else {
                nearest01Point = roofPoints.at(4);
                nearest02Point = roofPoints.at(5);
                nearest11Point = roofPoints.at(2); //
                nearest12Point = roofPoints.at(3); //
            }
        }


        int r = (rand() % 255);
        int g = (rand() % 255);
        int b = (rand() % 255);

        std::string cloudName = endMe("roofPoints", i);
        viewer->addPointCloud(m_segments.at(i), cloudName);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)r/255, (double)g/255, (double)b/255, cloudName);


        viewer->addLine(roofPoints.at(0), roofPoints.at(1), 255, 0, 0, endMe("roofTop1", i));

        viewer->addLine(roofPoints.at(0), nearest01Point, 255, 0, 0, endMe("roofSide2", i));
        viewer->addLine(roofPoints.at(0), nearest02Point, 255, 0, 0, endMe("roofSide3", i));

        viewer->addLine(roofPoints.at(1), nearest11Point, 255, 0, 0, endMe("roofSide4", i));
        viewer->addLine(roofPoints.at(1), nearest12Point, 255, 0, 0, endMe("roofSide5", i));

        viewer->addLine(roofPoints.at(5), roofPoints.at(3), 255, 0, 0, endMe("statueLine1", i));
        viewer->addLine(roofPoints.at(5), roofPoints.at(4), 255, 0, 0, endMe("statueLine2", i));
        viewer->addLine(roofPoints.at(4), roofPoints.at(2), 255, 0, 0, endMe("statueLine3", i));
        viewer->addLine(roofPoints.at(2), roofPoints.at(3), 255, 0, 0, endMe("statueLine4", i));

        //Kresleni podstavy
        auto down1 = roofPoints.at(2);
        auto down2 = roofPoints.at(3);
        auto down3 = roofPoints.at(4);
        auto down4 = roofPoints.at(5);
        down1.z = 582;
        down2.z = 582;
        down3.z = 582;
        down4.z = 582;
        viewer->addLine(roofPoints.at(2), down1, 255, 0, 0, endMe("downLine1", i));
        viewer->addLine(roofPoints.at(3), down2, 255, 0, 0, endMe("downLine2", i));
        viewer->addLine(roofPoints.at(4), down3, 255, 0, 0, endMe("downLine3", i));
        viewer->addLine(roofPoints.at(5), down4, 255, 0, 0, endMe("downLine4", i));

        viewer->addLine(down4, down2, 255, 0, 0, endMe("downStatue1", i));
        viewer->addLine(down4, down3, 255, 0, 0, endMe("downStatue2", i));
        viewer->addLine(down3, down1, 255, 0, 0, endMe("downStatue3", i));
        viewer->addLine(down1, down2, 255, 0, 0, endMe("downStatue4", i));


        // viewer->addText3D("0", roofPoints.at(0), 1.0, 1.0, 1.0, 1.0, endMe("pt0", i));
        // viewer->addText3D("1", roofPoints.at(1), 1.0, 1.0, 1.0, 1.0, endMe("pt1", i));
        // viewer->addText3D("leftTop", roofPoints.at(2), 1.0, 1.0, 1.0, 1.0, endMe("leftTop", i));
        // viewer->addText3D("leftDown", roofPoints.at(3), 1.0, 1.0, 1.0, 1.0, endMe("leftDown", i));
        // viewer->addText3D("rightTop", roofPoints.at(4), 1.0, 1.0, 1.0, 1.0, endMe("rightTop", i));
        // viewer->addText3D("rightDown", roofPoints.at(5), 1.0, 1.0, 1.0, 1.0, endMe("rightDown", i));
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Roofer::fromRidgesAndWholeCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    EuclidianClusterSegmentation segmentation;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    bool bcloud[cloud->size()] = {};
    std::map<pcl::PointXYZ, bool> cloud_bmap;
    const float radius = 2;

    std::cout << "Pocet bodu v cloud: " << cloud->size() << std::endl;
    auto roofRidges = getRoofRidge(cloud,4);
    std::cout << "Pocet bodu ve hrbetu: " << roofRidges->size() << std::endl;
    auto ridgeSegments = segmentation.segmentCloud(roofRidges);
    std::cout << "Pocet segmentu: " << ridgeSegments.size() << std::endl;

    for (auto ridgeSegment : ridgeSegments) {
        std::vector<pcl::PointXYZ> toPush;

        do {
            toPush.clear();
            for (auto rsit = ridgeSegment->begin(); rsit != ridgeSegment->end(); rsit++) {
                if (cloud->size() != 0) {
                    std::vector<int> pointIdxRadiusSearch;
                    std::vector<float> pointRadiusSquaredDistance;

                    if (kdtree.radiusSearch((*rsit), 2, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                        for (int i : pointIdxRadiusSearch) {
                            if (bcloud[i] == false && (rsit->z > cloud->at(i).z)) {
                                toPush.push_back(cloud->at(i));
                                bcloud[i] = true;
                            }
                        }
                    }
                }
            }
            for (auto pointToAdd : toPush) {
                ridgeSegment->push_back(pointToAdd);
            }
        }
        while (!toPush.empty());

    }

    return ridgeSegments;
}
