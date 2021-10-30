#ifndef Gesture_hpp
#define Gesture_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

class Gesture {
private:
    std::vector<cv::Point> gesturePoints;
    std::string name;
public:
    Gesture(std::string name);
    void addPoint(cv::Point point);
};

#endif /* Gesture_hpp */
