#ifndef FaceDetector_hpp
#define FaceDetector_hpp

#include <stdio.h>
#include "opencv2/opencv.hpp"

class FaceDetector{
private:
    cv::CascadeClassifier classifier;
    
public:
    FaceDetector();
    std::vector<cv::Rect> getFaces(cv::Mat image);
    
    
};

#endif /* FaceDetector_hpp */
