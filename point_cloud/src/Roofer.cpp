#include "Roofer.hpp"

bool pointsEq(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    return (
        p1.x == p2.x &&
        p1.y == p2.y &&
        p1.z == p2.z
    );
}


Roofer::Roofer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    for(auto point : *cloud) {
        m_centroid.add(point);
    }
    m_cloud = cloud;
}

void Roofer::roof() {
    auto filteredCloud = filterRoofNoise(m_cloud);
    std::cout << "m_cloud: " << m_cloud->size() << " filteredCloud: " << filteredCloud->size() << std::endl;
    m_cloud = filteredCloud;

    auto roofRidge = getRoofRidge(filteredCloud);
    auto bottomOfRoof = getRoofBottom(filteredCloud);

    auto farthestPoints = getFarthestPoints(roofRidge);
    pcl::PointXYZ ridgePoint1 = getMaxZPointNear(std::get<0>(farthestPoints), 1.5, roofRidge);
    pcl::PointXYZ ridgePoint2 = getMaxZPointNear(std::get<1>(farthestPoints), 1.5, roofRidge);

    m_roofPoints.push_back(ridgePoint1);
    m_roofPoints.push_back(ridgePoint2);
    std::cout << "RidgePoint 1 x a y: " << m_roofPoints.at(0).x << ", " << m_roofPoints.at(0).y << std::endl;
    std::cout << "RidgePoint 2 x a y: " << m_roofPoints.at(1).x << ", " << m_roofPoints.at(1).y << std::endl;

    std::cout << "pred from ridgesandwhole cluster" << std::endl;
    auto segmentedByRidges = fromRidgesAndWholeCluster(); // tohle je vector segmentu, proto posilam do dalsi metody zatim jen z prvni pozice

    std::cout << "Po from ridgesandwhole cluster" << std::endl;
    std::cout << "Pocet bodu v segmentu na prvni pozici: " << segmentedByRidges.at(0)->size() << std::endl;

    auto bottomOfRoof = getRoofBottom(segmentedByRidges.at(0));
    std::cout << "Bottom of roof: " << bottomOfRoof->size() << std::endl;

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
        if (histogram[h] > 10) {
            filteredCloud->push_back(point);
        }
    }

    return filteredCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Roofer::getRoofRidge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCbyZ(new pcl::PointCloud<pcl::PointXYZ>());
    float zMax = getMaxZPoint(m_cloud).z;

    for (int i = 0; i < m_cloud->size(); i++) {
        if (zMax - 5< m_cloud->at(i).z) {
            filteredCbyZ->push_back(m_cloud->at(i));
        }
    }
    std::cout << "Pocet po filtraci roof: " << filteredCbyZ->size() << std::endl;
    return filteredCbyZ;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr Roofer::getRoofBottom(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCbyZ(new pcl::PointCloud<pcl::PointXYZ>());
    float zMin = getMinZPoint(cloud).z;

    std::cout << "Pred filtraci bottom: " << cloud->size() << std::endl;

    for (int i = 0; i < cloud->size(); i++) {
        if ((zMin + 5) > cloud->at(i).z) {
            filteredCbyZ->push_back(cloud->at(i));
        }
    }
    std::cout << "Po filtraci bottom: " << filteredCbyZ->size() << std::endl;
    std::cout << zMin + 2 << std::endl;
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

    std::cout << "GetCorners cloud: " << cloud->size() << std::endl;

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

    std::cout<< "First corner osa X: " << m_roofPoints.at(2).x << std::endl;

    //std::cout << "Rozsah na ose X: " << maxX - minX << std::endl;
    //std::cout << "Rozsah na ose Y: " << maxY - minY << std::endl;
    //std::cout << "Rozsah na ose Z: " << maxZ - minZ << std::endl;

}

void Roofer::visualize(const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer) {
    pcl::PointXYZ nearest01Point;
    pcl::PointXYZ nearest02Point;
    pcl::PointXYZ nearest11Point;
    pcl::PointXYZ nearest12Point;

    float dist0LT = abs(m_roofPoints.at(2).x - m_roofPoints.at(0).x) + abs(m_roofPoints.at(2).y - m_roofPoints.at(0).y);
    float dist1LT = abs(m_roofPoints.at(2).x - m_roofPoints.at(1).x) + abs(m_roofPoints.at(2).y - m_roofPoints.at(1).y);

    float dist0LD = abs(m_roofPoints.at(3).x - m_roofPoints.at(0).x) + abs(m_roofPoints.at(3).y - m_roofPoints.at(0).y);
    float dist1LD = abs(m_roofPoints.at(3).x - m_roofPoints.at(1).x) + abs(m_roofPoints.at(3).y - m_roofPoints.at(1).y);

    if (dist0LT < dist1LT) {
        if (dist0LD < dist1LD) {
            nearest01Point = m_roofPoints.at(2); //
            nearest02Point = m_roofPoints.at(3); //
            nearest11Point = m_roofPoints.at(4);
            nearest12Point = m_roofPoints.at(5);
        }
        else {
            nearest01Point = m_roofPoints.at(2); //
            nearest02Point = m_roofPoints.at(4);
            nearest11Point = m_roofPoints.at(3); //
            nearest12Point = m_roofPoints.at(5);
        }
    }
    else {
        if (dist0LD < dist1LD) {
            nearest01Point = m_roofPoints.at(3); //
            nearest02Point = m_roofPoints.at(4);
            nearest11Point = m_roofPoints.at(2); //
            nearest12Point = m_roofPoints.at(5);
        }
        else {
            nearest01Point = m_roofPoints.at(4);
            nearest02Point = m_roofPoints.at(5);
            nearest11Point = m_roofPoints.at(2); //
            nearest12Point = m_roofPoints.at(3); //
        }
    }

    viewer->addPointCloud(m_cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");

    viewer->addLine(m_roofPoints.at(0), m_roofPoints.at(1), 255, 0, 0, "roofTop1");

    viewer->addLine(m_roofPoints.at(0), nearest01Point, 255, 0, 0, "roofSide2");
    viewer->addLine(m_roofPoints.at(0), nearest02Point, 255, 0, 0, "roofSide3");

    viewer->addLine(m_roofPoints.at(1), nearest11Point, 255, 0, 0, "roofSide4");
    viewer->addLine(m_roofPoints.at(1), nearest12Point, 255, 0, 0, "roofSide5");

    viewer->addLine(m_roofPoints.at(5), m_roofPoints.at(3), 255, 0, 0, "statueLine1");
    viewer->addLine(m_roofPoints.at(5), m_roofPoints.at(4), 255, 0, 0, "statueLine2");
    viewer->addLine(m_roofPoints.at(4), m_roofPoints.at(2), 255, 0, 0, "statueLine3");
    viewer->addLine(m_roofPoints.at(2), m_roofPoints.at(3), 255, 0, 0, "statueLine4");

    viewer->addText3D("0", m_roofPoints.at(0), 1.0, 1.0, 1.0, 1.0, "pt0");
    viewer->addText3D("1", m_roofPoints.at(1), 1.0, 1.0, 1.0, 1.0, "pt1");
    viewer->addText3D("leftTop", m_roofPoints.at(2), 1.0, 1.0, 1.0, 1.0, "leftTop");
    viewer->addText3D("leftDown", m_roofPoints.at(3), 1.0, 1.0, 1.0, 1.0, "leftDown");
    viewer->addText3D("rightTop", m_roofPoints.at(4), 1.0, 1.0, 1.0, 1.0, "rightTop");
    viewer->addText3D("rightDown", m_roofPoints.at(5), 1.0, 1.0, 1.0, 1.0, "rightDown");

    pcl::PointXYZ center;
    m_centroid.get(center);
    viewer->addLine(center, pcl::PointXYZ(center.x + 10, center.y, center.z), 1.0, 0, 0, "X");
    viewer->addLine(center, pcl::PointXYZ(center.x, center.y + 10, center.z), 0, 1.0, 0, "Y");
    viewer->addLine(center, pcl::PointXYZ(center.x, center.y, center.z + 10), 0, 0, 1.0, "Z");
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Roofer::fromRidgesAndWholeCluster(){


    pcl::PointCloud<pcl::PointXYZ>::Ptr copyM_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    float radius = 2;


    for( int i = 0; i < m_cloud->size(); i++){
        copyM_cloud->push_back(m_cloud->at(i));
        }

    EuclidianClusterSegmentation segmentation;

    std::cout << "Pocet bodu v m_cloud: " << copyM_cloud->size() << std::endl;
    auto roofRidges = getRoofRidge(copyM_cloud);
    std::cout << "Pocet bodu ve hrbetu: " << roofRidges->size() << std::endl;
    auto ridgeSegments = segmentation.segmentCloud(roofRidges);
    std::cout << "Pocet segmentu: " << ridgeSegments.size() << std::endl;
    for (int i = 0; i < ridgeSegments.size(); i++){
        std::cout << "first cajklus u this" << std::endl;
        for (int j = 0; j < ridgeSegments.at(i)->size(); j++){
            //std::cout << "second cajklus u this" << std::endl;
            if (copyM_cloud->size() != 0){
                pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
                kdtree.setInputCloud(copyM_cloud);

                if(kdtree.radiusSearch(ridgeSegments.at(i)->at(j), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance))
                {
                    for (int k = 0; k < pointIdxRadiusSearch.size(); k++){

                        std::cout <<"i: " << i << ", j: " << j << ", k: " << k << std::endl;
                        std::cout << "ridgeSegments.at(i)->push_back(copyM_cloud->at(pointIdxRadiusSearch.at(k)));" << std::endl;
                        ridgeSegments.at(i)->push_back(copyM_cloud->at(pointIdxRadiusSearch.at(k)));
                        std::cout << "copyM_cloud->erase(copyM_cloud->begin() + pointIdxRadiusSearch.at(k));" << std::endl;
                        copyM_cloud->erase(copyM_cloud->begin() + pointIdxRadiusSearch.at(k));
                        }

                }
            }
            else{
                break;
            }


        }
    }
    std::cout << "end of insojd u this" << std::endl;

    return ridgeSegments;







    //if (kdtree.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) < 0) {
    //    return point;
    //}
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