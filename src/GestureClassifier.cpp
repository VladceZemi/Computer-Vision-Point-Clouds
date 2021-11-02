#include "GestureClassifier.hpp"

GestureClassifier::GestureClassifier() {
    Gesture gesture1 = Gesture("open");
    gesture1.addPoint(cv::Point(19, 21));
    gesture1.addPoint(cv::Point(19, 17));
    gesture1.addPoint(cv::Point(18, 14));
    gesture1.addPoint(cv::Point(17, 12));
    gesture1.addPoint(cv::Point(15, 10));
    gesture1.addPoint(cv::Point(14,  8));
    gesture1.addPoint(cv::Point(13,  6));
    gesture1.addPoint(cv::Point(11,  5));
    gesture1.addPoint(cv::Point( 9,  3));
    gesture1.addPoint(cv::Point( 7,  2));
    gesture1.addPoint(cv::Point( 5,  1));
    gesture1.addPoint(cv::Point( 3,  0));
    gesture1.addPoint(cv::Point( 0,  0));

    Gesture gesture2 = Gesture("close");
    gesture2.addPoint(cv::Point( 0, 14));
    gesture2.addPoint(cv::Point( 0, 13));
    gesture2.addPoint(cv::Point( 0, 11));
    gesture2.addPoint(cv::Point( 1, 10));
    gesture2.addPoint(cv::Point( 2,  8));
    gesture2.addPoint(cv::Point( 2,  7));
    gesture2.addPoint(cv::Point( 4,  6));
    gesture2.addPoint(cv::Point( 5,  4));
    gesture2.addPoint(cv::Point( 6,  4));
    gesture2.addPoint(cv::Point( 7,  2));
    gesture2.addPoint(cv::Point( 9,  1));
    gesture2.addPoint(cv::Point(10,  1));
    gesture2.addPoint(cv::Point(12,  0));
    gesture2.addPoint(cv::Point(13,  0));
    gesture2.addPoint(cv::Point(16,  0));
    gesture2.addPoint(cv::Point(18,  0));
    gesture2.addPoint(cv::Point(20,  0));

    allGestures.push_back(gesture1);
    allGestures.push_back(gesture2);
}

std::string GestureClassifier::classify(std::vector<cv::Point> gesturePath, std::vector<cv::Point> gesturePathSec) {   
    if (gesturePathSec.empty()) {
        return classifyOneHanded(gesturePath);
    }
    else {
        return classifyTwoHanded(gesturePath, gesturePathSec);
    }
};

std::string GestureClassifier::classifyOneHanded(std::vector<cv::Point> gesturePath) {
    std::vector<Gesture> gestures = getOneHandedGestures();

    for (int i = 0; i < gestures.size(); i++) {
        std::cout << "Pokus o klasifikaci klasifikatorem: " << gestures.at(i).getName() << "\n";

        if (findGestureInPath(gestures.at(i).gesturePoints, gesturePath)) {  
            return gestures.at(i).getName();
        }
    }

    return "None";
}

std::string GestureClassifier::classifyTwoHanded(std::vector<cv::Point> gesturePath, std::vector<cv::Point> gesturePathSec) {
    std::vector<Gesture> gestures = getTwoHandedGestures();

    for (int i = 0; i < gestures.size(); i++) {
        std::cout << "Pokus o klasifikaci klasifikatorem: " << gestures.at(i).getName() << "\n";

        if (
            findGestureInPath(gestures.at(i).gesturePoints, gesturePath) &&
            findGestureInPath(gestures.at(i).gesturePointsSec, gesturePathSec)
        ) {
            return gestures.at(i).getName();
        }
    }

    return "None";
}

std::vector<Gesture> GestureClassifier::getOneHandedGestures() {
    std::vector<Gesture> oneHandedGestures;
    for (Gesture g : allGestures) {
        if (!g.isTwoHanded()) {
            oneHandedGestures.push_back(g);
        }
    }
    return oneHandedGestures;
}

std::vector<Gesture> GestureClassifier::getTwoHandedGestures() {
    std::vector<Gesture> twoHandedGestures;
    for (Gesture g : allGestures) {
        if (g.isTwoHanded()) {
            twoHandedGestures.push_back(g);
        }
    }
    return twoHandedGestures;
}

bool GestureClassifier::findGestureInPath(std::vector<cv::Point> gesturePoints, std::vector<cv::Point> gesturePath) {
    for (cv::Point gesturePoint : gesturePoints) {
        bool pointFound = false;
        int eraseTo = 0;
        
        for (int i = 0; i < gesturePath.size(); i++) {
            
            cv::Point pathPoint = gesturePath.at(i);
            int distance = abs(gesturePoint.x - pathPoint.x) + abs(gesturePoint.y - pathPoint.y);
            if (distance <= toleration) {
                pointFound = true;
                eraseTo = i + 1;
                //std::cout << gesturePath.at(i) << "=?" << gesturePoint << "\n";
                //std::cout << "Match v cyklu: "  << i << ", pocet vymazanych pointu: " << eraseTo << "\n";
                break;
            }
        }

        if (pointFound)
            gesturePath.erase(gesturePath.begin(), gesturePath.begin() + eraseTo);
        else                                                   
            return false;
    }

    return true;
}
