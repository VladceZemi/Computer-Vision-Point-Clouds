#ifndef Gesture_hpp
#define Gesture_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

class Gesture {
private:
    std::string name;
public:
    std::vector<cv::Point> gesturePoints;
    std::vector<cv::Point> gesturePointsSec;

    Gesture(std::string name);
    void addPoint(cv::Point point);
    void addPointSec(cv::Point point);
    std::string getName();
    bool isTwoHanded();
};

#endif /* Gesture_hpp */
