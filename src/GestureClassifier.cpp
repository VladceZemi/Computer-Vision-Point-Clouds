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

    Gesture gesture3 = Gesture("next");
    gesture3.addPoint(cv::Point( 9,  6));
    gesture3.addPoint(cv::Point(10,  4));
    gesture3.addPoint(cv::Point(11,  2));
    gesture3.addPoint(cv::Point(12,  1));
    gesture3.addPoint(cv::Point(15,  0));
    gesture3.addPoint(cv::Point(17,  0));
    gesture3.addPoint(cv::Point(19,  1));
    gesture3.addPoint(cv::Point(19,  4));
    gesture3.addPoint(cv::Point(19,  8));
    gesture3.addPoint(cv::Point(18,  9));
    
    gesture3.addPointSec(cv::Point(0, 7));
    gesture3.addPointSec(cv::Point(0, 4));
    gesture3.addPointSec(cv::Point(1, 2));
    gesture3.addPointSec(cv::Point(4, 2));
    gesture3.addPointSec(cv::Point(7, 2));
    gesture3.addPointSec(cv::Point(9, 1));
    gesture3.addPointSec(cv::Point(8, 3));
    gesture3.addPointSec(cv::Point(7, 5));
    gesture3.addPointSec(cv::Point(7, 8));
    gesture3.addPointSec(cv::Point(5, 9));

    Gesture gesture4 = Gesture("zoom");
    gesture4.addPoint(cv::Point(11, 80));
    gesture4.addPoint(cv::Point(7, 88));
    gesture4.addPoint(cv::Point(5, 95));
    gesture4.addPoint(cv::Point(6, 103));
    gesture4.addPoint(cv::Point(1, 118));
    gesture4.addPoint(cv::Point(1, 121));
    gesture4.addPoint(cv::Point(1, 127));
    gesture4.addPoint(cv::Point(2, 135));
    gesture4.addPoint(cv::Point(1, 144));
    gesture4.addPoint(cv::Point(2, 142));

    gesture4.addPointSec(cv::Point(11, 43));
    gesture4.addPointSec(cv::Point(13, 36));
    gesture4.addPointSec(cv::Point(14, 30));
    gesture4.addPointSec(cv::Point(15, 25));
    gesture4.addPointSec(cv::Point(15, 21));
    gesture4.addPointSec(cv::Point(15, 16));
    gesture4.addPointSec(cv::Point(15, 12));
    gesture4.addPointSec(cv::Point(17, 7));
    gesture4.addPointSec(cv::Point(17, 2));
    gesture4.addPointSec(cv::Point(20, 0));


    allGestures.push_back(gesture1);
    allGestures.push_back(gesture2);
    allGestures.push_back(gesture3);
    allGestures.push_back(gesture4);
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
        else if (
            findGestureInPath(gestures.at(i).gesturePoints, gesturePathSec) &&
            findGestureInPath(gestures.at(i).gesturePointsSec, gesturePath)
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
