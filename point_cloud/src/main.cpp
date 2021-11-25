#include <iostream>
#include <thread>

#include "PLYLoader.hpp"
#include "PCLVisualization.hpp"
#include "GroundExtractor.hpp"
#include "EuclidianClusterSegmentation.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/auto_io.h>
#include <pcl/point_types.h>


int main (int argc, char** argv)
{
    PLYLoader loader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loader.loadCloud("segmented.ply");

    std::cout << "Input cloud points: " << cloud->points.size() << std::endl;

    EuclidianClusterSegmentation segmentation;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedClouds = segmentation.segmentCloud(cloud);


    PCLVisualization visualization;
    visualization.initializeVisualization();


    // for (const auto& cloud : segmentedClouds){
    //     visualization.addCloudWithRandomColor(cloud);
    // }




    pcl::PointCloud<pcl::PointXYZ>::Ptr c = segmentedClouds.at(13);

    float zMax = 0;

    for (int i = 0; i < c->size(); i++){
        std::cout << c->at(i).z << std::endl;
        if (zMax < c->at(i).z){
            zMax = c->at(i).z;
        }
    }
    std::cout << "Z max: " << zMax << std::endl;

    // filtrace pomoci z

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCbyZ(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr maxPoints(new pcl::PointCloud<pcl::PointXYZ>());

    float toleration = 4;

    for (int i = 0; i < c->size(); i++){
        if (zMax-toleration < c->at(i).z){
            filteredCbyZ->push_back(c->at(i));
        }
    }


    //tvorba linie

    float maxEuklidVzd = 0;

    pcl::PointXYZ bod1;
    pcl::PointXYZ bod2;

    for (int i = 0; i < filteredCbyZ->size(); i++){
        for (int j = 0; j < filteredCbyZ->size(); j++){
            float aktEuklid = abs(filteredCbyZ->at(i).x - filteredCbyZ->at(j).x) + abs(filteredCbyZ->at(i).y - filteredCbyZ->at(j).y);
            if (maxEuklidVzd < aktEuklid){
                bod1 = filteredCbyZ->at(i);
                bod2 = filteredCbyZ->at(j);
                maxEuklidVzd = aktEuklid;
            }
        }
    }

    float uhel = abs(bod1.x-bod2.x)/abs(bod1.y-bod2.y);

    std::cout << uhel <<std::endl;

    // TODO : podle uhlu vytvorit kruhovou vysec, avsak neni to z toho vypoctu jasne (radek 79), viz. toto : 0 = 0,90,180,270,360 1 = 45,135,225,315 - neni treba resit kazdou zvlast, ale
    // staci dve skupiny a to: 0,180,"360" a 90,270 je to stejne, jen otocene o 180 ze, takze vysec je stejna. Takze staci jen jedno vetveni, ktere pujde vymyslet
    // urcite na to jestli ta euklidovska vzdalenost je kladna nebo zaporna  na ose x nebo y apod. Resit z nema moc smysl protoze ta strecha je priblizne rovna, stejne po tomto
    // kroku se vybere podle toho z ten vrcholny bod, a nebo treba average 25% nejvyssich bodu apod.


    maxPoints->push_back(bod1);
    maxPoints->push_back(bod2);

    visualization.addCloudWithRandomColor(filteredCbyZ);
    visualization.addCloudWithRandomColor(maxPoints);
    visualization.runVisualization();

    return 0;
}
