#include "ColorTracker.hpp"

ColorTracker::ColorTracker() {
    
}

cv::Mat ColorTracker::preproccesImage(cv::Mat image) {
    cv::Mat preproccessedImage;
    cv::Mat hsvImage;
    
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);
    cv::inRange(hsvImage, cv::Scalar(100, 100, 50), cv::Scalar(120, 255, 255), preproccessedImage);
    
    // otevreni. uzavreni je stejne, akorat prohodit poradi
    cv::erode(preproccessedImage, preproccessedImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));
    cv::dilate(preproccessedImage, preproccessedImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));
    
    // uzavreni
    cv::dilate(preproccessedImage, preproccessedImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));
    cv::erode(preproccessedImage, preproccessedImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));
    
    cv::imshow("Preprocessed Image", preproccessedImage);
    
    return preproccessedImage;
}

cv::Point ColorTracker::getColorPosition(cv::Mat image) {
    cv::Point colorPosition = cv::Point(0, 0);
    cv::Mat preprocessedImage = preproccesImage(image);
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hiearchy;
    
    cv::findContours(preprocessedImage, contours, hiearchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    double largestArea = 0;
    int largetAreaIndex = -1;
    
    for(int i = 0; i < contours.size(); i++) {
        
        std::vector<cv::Point> polygon;
        cv:approxPolyDP(contours.at(i), polygon, 5, true);

        
        double area = cv::contourArea(polygon);
        
        if (area > largestArea) {
            largestArea = area;
            largetAreaIndex = i;
        }
    }
    
    if (largetAreaIndex > -1) {
        cv::Rect boundingBox = cv::boundingRect(contours.at(largetAreaIndex));
        int centerX = (boundingBox.width / 2) + boundingBox.tl().x;
        int centerY = (boundingBox.height / 2) + boundingBox.tl().y;
        colorPosition = cv::Point(centerX, centerY);
    }
    
    return colorPosition;
}
