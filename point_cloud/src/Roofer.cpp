#include "Roofer.hpp"

Roofer::Roofer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    m_cloud = cloud;
}

void Roofer::roof() {
    auto roofRidge = getRoofRidge(m_cloud);

    auto farthestPoints = getFarthestPoints(roofRidge);
    pcl::PointXYZ ridgePoint1 = getMaxZPointNear(std::get<0>(farthestPoints), 1.5, roofRidge);
    pcl::PointXYZ ridgePoint2 = getMaxZPointNear(std::get<1>(farthestPoints), 1.5, roofRidge);

    m_roofPoints.push_back(ridgePoint1);
    m_roofPoints.push_back(ridgePoint2);

    getCornerPoints(m_cloud);
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

pcl::PointCloud<pcl::PointXYZ>::Ptr Roofer::getRoofRidge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCbyZ(new pcl::PointCloud<pcl::PointXYZ>());
    float zMax = getMaxZPoint(m_cloud).z;

    for (int i = 0; i < m_cloud->size(); i++) {
        if (zMax - TOLERATION < m_cloud->at(i).z) {
            filteredCbyZ->push_back(m_cloud->at(i));
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

void Roofer::getCornerPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointXYZ leftTop(std::numeric_limits<float>::max(), std::numeric_limits<float>::min(), 0);
    pcl::PointXYZ leftDown(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0);
    pcl::PointXYZ rightTop(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), 0);
    pcl::PointXYZ rightDown(std::numeric_limits<float>::min(), std::numeric_limits<float>::max(), 0);

    for (auto point : *cloud) {
        if (point.y > leftTop.y && point.x < leftTop.x) {
            leftTop = point;
        }
        if (point.y < leftDown.y && point.x < leftDown.x) {
            leftDown = point;
        }
        if (point.y > rightTop.y && point.x > rightTop.x) {
            rightTop = point;
        }
        if (point.y < rightDown.y && point.x > rightDown.x) {
            rightDown = point;
        }
    }

    m_roofPoints.push_back(leftTop);
    m_roofPoints.push_back(leftDown);
    m_roofPoints.push_back(rightTop);
    m_roofPoints.push_back(rightDown);
}

void Roofer::visualize(const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer) {
//    for (int i = 1; i < m_roofPoints.size(); i++) {
//        std::stringstream stream;
//        stream << "roof" << i - 1 << "-" << i;
//        std::string lineName = stream.str();
//
//        viewer->addLine(m_roofPoints.at(i - 1), m_roofPoints.at(i), 255, 0, 0, lineName);
//    }

    viewer->addLine(m_roofPoints.at(0), m_roofPoints.at(1), 255, 0, 0, "roofTop1");
    viewer->addLine(m_roofPoints.at(0), m_roofPoints.at(2), 255, 0, 0, "roofTop2");
    viewer->addLine(m_roofPoints.at(0), m_roofPoints.at(3), 255, 0, 0, "roofTop3");

    viewer->addLine(m_roofPoints.at(1), m_roofPoints.at(4), 255, 0, 0, "roofTop4");
//    viewer->addLine(m_roofPoints.at(1), m_roofPoints.at(5), 255, 0, 0, "roofTop5");

    viewer->addLine(m_roofPoints.at(2), m_roofPoints.at(3), 255, 0, 0, "roof1");
    viewer->addLine(m_roofPoints.at(3), m_roofPoints.at(4), 255, 0, 0, "roof2");
//    viewer->addLine(m_roofPoints.at(4), m_roofPoints.at(5), 255, 0, 0, "roof3");
    viewer->addLine(m_roofPoints.at(5), m_roofPoints.at(2), 255, 0, 0, "roof4");
}

void jirkuvMagic() {
//    if(uhelDegress > 90 || uhelDegress > 270){
//        if (uhelDegress > 270){
//            uhelDegress -= 270;
//            uhelDegress = uhelDegress/90;
//            posunutyBod1.x = bod1.x + 10*uhelDegress;
//            posunutyBod1.y = bod1.y - 10/uhelDegress;
//            //  pricitani x odecitani na ose y
//        }
//        else{
//            uhelDegress -= 0;
//            uhelDegress = uhelDegress/90;
//            posunutyBod1.x = bod1.x + 10/uhelDegress;
//            posunutyBod1.y = bod1.y + 10*uhelDegress;
//            // pricitani x pricitani na ose y
//        }
//    }
//    else{
//        if (uhelDegress > 180){
//            uhelDegress -= 180;
//            uhelDegress = uhelDegress/90;
//            posunutyBod1.x = bod1.x - 10/uhelDegress;
//            posunutyBod1.y = bod1.y - 10*uhelDegress;
//            // odecitani na x odecitani na ose y
//        }
//        else
//        {
//            uhelDegress -= 90;
//            uhelDegress = uhelDegress/90;
//            posunutyBod1.x = bod1.x + 10/uhelDegress;
//            posunutyBod1.y = bod1.y - 10*uhelDegress;
//            // pricitani na x odecitani na ose y
//        }
//    }
}