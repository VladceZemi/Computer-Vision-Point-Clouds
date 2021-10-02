#ifndef ColorTracker_hpp
#define ColorTracker_hpp

#include <stdio.h>
#include "opencv2/opencv.hpp"


class ColorTracker{
private:
    cv::Mat preproccesImage(cv::Mat image);
    
public:
    ColorTracker();
    cv::Point getColorPosition(cv::Mat image);
    
};


#endif /* ColorTracker_hpp */
