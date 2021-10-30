#ifndef GestureClassifier_hpp
#define GestureClassifier_hpp

#include <string>
#include <opencv2/opencv.hpp>
#include "Gesture.hpp"

class GestureClassifier {
private:
    std::vector<Gesture> gestures;
    int toleration = 20;
public:
    GestureClassifier();
    std::string classify(std::vector<cv::Point> gesturePath);
};

#endif /* GestureClassifier_hpp */
