#ifndef GestureRecognizer_hpp
#define GestureRecognizer_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "GestureClassifier.hpp"

class GestureRecognizer {
private:
    int groupPointTreshold = 50;
    int ungroupPointTreshold = 300;

    cv::Mat image;
    cv::Mat groupsImage;
    cv::Mat preprocessedImage;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hiearchy;
    std::vector<std::vector<cv::Point>> handsPath;
    std::vector<std::vector<cv::Point>> normalizedHandsPath;
    
    void preprocessImage();
    void morphologyOperations();
    void findAndFilterContours();
    void normalizeMovementPath();
    void filterMotionLessObjects();
    cv::Point createHandPoint(std::vector<cv::Point> polygon);
    void groupPoints(cv::Point handPoint);
    std::vector<cv::Point> getLastPointsFromHandsPath();
    void filterMotionlessObjects();
    void drawGroups();
public:
    GestureRecognizer();
    void process(cv::Mat image);
    std::string recognize();
};

#endif /* GestureRecognizer_hpp */
