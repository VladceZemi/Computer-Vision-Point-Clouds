#ifndef GestureRecognizer_hpp
#define GestureRecognizer_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "GestureClassifier.hpp"

class GestureRecognizer {
private:
    cv::Mat image;
    cv::Mat preprocessedImage;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hiearchy;
    std::vector<cv::Point> gesturePath;
    
    void preprocessImage();
    void morphologyOperations();
    void findAndFilterContours();
    void normalizeMovementPath();
public:
    GestureRecognizer();
    void process(cv::Mat image);
    void recognize();
};

#endif /* GestureRecognizer_hpp */
