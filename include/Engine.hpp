#ifndef Engine_hpp
#define Engine_hpp

#include <stdio.h>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "GestureRecognizer.hpp"

class Engine {
public:
    Engine();
    void run();
    
private:
    bool isRunning;
    std::vector<std::string> videos;
    cv::VideoCapture videoCapture;
    cv::Mat frame;
    cv::Size windowSize;
    GestureRecognizer gestureRecognizer;

    
    void processCameraFrame(cv::Mat frame);
    void processVideo(std::string videoName);
};

#endif /* Engine_hpp */
