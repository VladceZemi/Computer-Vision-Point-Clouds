#include "Roofer.hpp"

Roofer::Roofer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    m_cloud = cloud;
}

void Roofer::roof() {
    auto roofRidge = getRoofRidge(m_cloud);
    auto bottomOfRoof = getRoofBottom(m_cloud);

    auto farthestPoints = getFarthestPoints(roofRidge);
    pcl::PointXYZ ridgePoint1 = getMaxZPointNear(std::get<0>(farthestPoints), 1.5, roofRidge);
    pcl::PointXYZ ridgePoint2 = getMaxZPointNear(std::get<1>(farthestPoints), 1.5, roofRidge);

    m_roofPoints.push_back(ridgePoint1);
    m_roofPoints.push_back(ridgePoint2);

    getCornerPoints(bottomOfRoof);
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

pcl::PointCloud<pcl::PointXYZ>::Ptr Roofer::getRoofRidge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCbyZ(new pcl::PointCloud<pcl::PointXYZ>());
    float zMax = getMaxZPoint(m_cloud).z;

    for (int i = 0; i < m_cloud->size(); i++) {
        if ((zMax - TOLERATION) < m_cloud->at(i).z) {
            filteredCbyZ->push_back(m_cloud->at(i));
        }
    }

    return filteredCbyZ;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr Roofer::getRoofBottom(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCbyZ(new pcl::PointCloud<pcl::PointXYZ>());
    float zMin = getMinZPoint(m_cloud).z;

    std::cout << "Pred filtraci: " << m_cloud->size() << std::endl;

    for (int i = 0; i < m_cloud->size(); i++) {
        if ((zMin + 10) > m_cloud->at(i).z) {
            filteredCbyZ->push_back(m_cloud->at(i));
        }
    }
    std::cout << "Po filtraci: " << filteredCbyZ->size() << std::endl;
    std::cout << zMin + TOLERATION << std::endl;
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
    pcl::PointXYZ firstCorner(std::numeric_limits<float>::max(), std::numeric_limits<float>::min(), 0);
    pcl::PointXYZ secondCorner(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0);
    pcl::PointXYZ thirdCorner(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), 0);
    pcl::PointXYZ fourthCorner(std::numeric_limits<float>::min(), std::numeric_limits<float>::max(), 0);

    //std::cout << "Minimalni hodnota na ose X: " << minX << std::endl;
    //std::cout << "Minimalni hodnota na ose Y: " << minY << std::endl;
    //std::cout << "Minimalni hodnota na ose Z: " << minZ << std::endl;
    //std::cout << "Maximalni hodnota na ose X: " << maxX << std::endl;
    //std::cout << "Maximalni hodnota na ose Y: " << maxY << std::endl;
    //std::cout << "Maximalni hodnota na ose Z: " << maxZ << std::endl;

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

        //if (point.x < minX){
        //    minX = point.x;
        //}
        //if (point.y < minY){
        //    minY = point.y;
        //}
        //if (point.z < minZ){
        //    minZ = point.z;
        //}
//
        //if (point.x > maxX){
        //    maxX = point.x;
        //}
        //if (point.y > maxY){
        //    maxY = point.y;
        //}
        //if (point.z > maxZ){
        //    maxZ = point.z;
        //}
    }
    m_roofPoints.push_back(firstCorner);
    m_roofPoints.push_back(secondCorner);
    m_roofPoints.push_back(thirdCorner);
    m_roofPoints.push_back(fourthCorner);

    //std::cout << "Rozsah na ose X: " << maxX - minX << std::endl;
    //std::cout << "Rozsah na ose Y: " << maxY - minY << std::endl;
    //std::cout << "Rozsah na ose Z: " << maxZ - minZ << std::endl;

}

void Roofer::visualize(const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer) {
//    for (int i = 1; i < m_roofPoints.size(); i++) {
//        std::stringstream stream;
//        stream << "roof" << i - 1 << "-" << i;
//        std::string lineName = stream.str();
//
//        viewer->addLine(m_roofPoints.at(i - 1), m_roofPoints.at(i), 255, 0, 0, lineName);
//    }

    std::cout << "Pocet bodu v mroof: " << m_roofPoints.size() << std::endl;

    std::cout << "firstCorner: osa X: " << m_roofPoints.at(2).x-minX <<  ", osa Y: " << m_roofPoints.at(2).y-minY << std::endl;
    std::cout << "secondCorner: osa X: " << m_roofPoints.at(3).x-minX <<  ", osa Y: " << m_roofPoints.at(3).y-minY << std::endl;
    std::cout << "thirdCorner: osa X: " << m_roofPoints.at(4).x-minX <<  ", osa Y: " << m_roofPoints.at(4).y-minY << std::endl;
    std::cout << "fourthCorner: osa X: " << m_roofPoints.at(5).x-minX <<  ", osa Y: " << m_roofPoints.at(5).y-minY << std::endl;

    float nearest01 = std::numeric_limits<float>::max();
    pcl::PointXYZ nearest01Point;
    float nearest02 = std::numeric_limits<float>::max();
    pcl::PointXYZ nearest02Point;
    float nearest11 = std::numeric_limits<float>::max();
    pcl::PointXYZ nearest11Point;
    float nearest12 = std::numeric_limits<float>::max();
    pcl::PointXYZ nearest12Point;


    for(int i = 2; i < m_roofPoints.size(); i++){
        float vzd0 = abs(m_roofPoints.at(i).x - m_roofPoints.at(0).x) + abs(m_roofPoints.at(i).y - m_roofPoints.at(0).y) + abs(m_roofPoints.at(i).z - m_roofPoints.at(0).z);
        float vzd1 = abs(m_roofPoints.at(i).x - m_roofPoints.at(1).x) + abs(m_roofPoints.at(i).y - m_roofPoints.at(1).y) + abs(m_roofPoints.at(i).z - m_roofPoints.at(1).z);
        if (nearest01 > vzd0){
            nearest02Point = nearest01Point;
            nearest02 = nearest01;

            nearest01Point = m_roofPoints.at(i);
            nearest01 = vzd0;
        };
        if (nearest02 > vzd0 && nearest01Point.x != m_roofPoints.at(i).x && nearest01Point.y != m_roofPoints.at(i).y && nearest01Point.z != m_roofPoints.at(i).z){
            nearest02Point = m_roofPoints.at(i);
            nearest02 = vzd0;
        };

        if (nearest11 > vzd1){
            nearest12Point = nearest11Point;
            nearest12 = nearest11;

            nearest11Point = m_roofPoints.at(i);
            nearest11 = vzd1;
        };
        if (nearest12 > vzd1 && nearest11Point.x != m_roofPoints.at(i).x && nearest11Point.y != m_roofPoints.at(i).y && nearest11Point.z != m_roofPoints.at(i).z){
            nearest12Point = m_roofPoints.at(i);
            nearest12 = vzd1;
        };
    }


    viewer->addLine(m_roofPoints.at(0), m_roofPoints.at(1), 255, 0, 0, "roofTop1");

    viewer->addLine(m_roofPoints.at(0), nearest01Point, 255, 0, 0, "roofTop2");
    viewer->addLine(m_roofPoints.at(0), nearest02Point, 255, 0, 0, "roofTop3");

    viewer->addLine(m_roofPoints.at(1), nearest11Point, 255, 0, 0, "roofTop4");
    viewer->addLine(m_roofPoints.at(1), nearest12Point, 255, 0, 0, "roofTop5");

    viewer->addLine(m_roofPoints.at(5), m_roofPoints.at(3), 255, 0, 0, "statueLine1");
    viewer->addLine(m_roofPoints.at(5), m_roofPoints.at(4), 255, 0, 0, "statueLine2");
    viewer->addLine(m_roofPoints.at(4), m_roofPoints.at(2), 255, 0, 0, "statueLine3");
    viewer->addLine(m_roofPoints.at(2), m_roofPoints.at(3), 255, 0, 0, "statueLine4");
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