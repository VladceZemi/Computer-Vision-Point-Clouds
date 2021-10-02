#ifndef Engine_hpp
#define Engine_hpp

#include <stdio.h>
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "ColorTracker.hpp"
#include "ObjectTracker.hpp"
#include "FaceDetector.hpp"


class Engine {
public:
    Engine();
    void run();
private:
    cv::VideoCapture videoCapture;
    cv::Mat cameraFrame;
    cv::Size windowSize;
    
    cv::Mat previousFrame;
    cv::Mat currentFrame;
    std::vector<cv::Point2f> pointsToTrack;
    
    void captureCameraFrame();
    void drawFaces(std::vector<cv::Rect>);
    
    
    
};

#endif /* Engine_hpp */
