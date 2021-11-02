#ifndef GestureClassifier_hpp
#define GestureClassifier_hpp

#include <string>
#include <opencv2/opencv.hpp>
#include "Gesture.hpp"

class GestureClassifier {
private:
    std::vector<Gesture> allGestures;
    int toleration = 1;

    bool findGestureInPath(std::vector<cv::Point> gesturePoints, std::vector<cv::Point> gesturePath);
public:
    GestureClassifier();
    std::string classify(std::vector<cv::Point> gesturePath, std::vector<cv::Point> gesturePathSec);
    std::string classifyOneHanded(std::vector<cv::Point> gesturePath);
    std::string classifyTwoHanded(std::vector<cv::Point> gesturePath, std::vector<cv::Point> gesturePathSec);
    std::vector<Gesture> getOneHandedGestures();
    std::vector<Gesture> getTwoHandedGestures();
};

#endif /* GestureClassifier_hpp */
