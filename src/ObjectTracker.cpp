#include "ObjectTracker.hpp"

ObjectTracker::ObjectTracker() {
    
}

std::vector<cv::Point2f> ObjectTracker::getFeatures(cv::Mat image) {
    std::vector<cv::Point2f> points;
    
    cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
    cv::goodFeaturesToTrack(image, points, 100, 0.01, 0.01);
    
    return points;
}

std::vector<cv::Point2f> ObjectTracker::trackObject(cv::Mat oldImage, cv::Mat currentImage, std::vector<cv::Point2f> oldPoints) {
    std::vector<cv::Point2f> newPoints;
    
    if (oldPoints.size() == 0)
        return newPoints;
    
    std::vector<uchar> status;
    std::vector<float> errors;
    
    cv::cvtColor(oldImage, oldImage, cv::COLOR_RGB2GRAY);
    cv::cvtColor(currentImage, currentImage, cv::COLOR_RGB2GRAY);
        
    cv::calcOpticalFlowPyrLK(oldImage, currentImage, oldPoints, newPoints, status, errors);

    return newPoints;
}

void ObjectTracker::drawPoints(cv::Mat image, std::vector<cv::Point2f> points, cv::Scalar color) {
    for (int i = 0; i < points.size(); i++) {
        cv::circle(image, points.at(i), 4, color, -1);
    }
}
