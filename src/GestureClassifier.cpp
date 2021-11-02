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
    gesture2.addPoint(cv::Point(0, 14));
    gesture2.addPoint(cv::Point(0, 13));
    gesture2.addPoint(cv::Point(0, 11));
    gesture2.addPoint(cv::Point(1, 10));
    gesture2.addPoint(cv::Point(2, 8));
    gesture2.addPoint(cv::Point(2, 7));
    gesture2.addPoint(cv::Point(4, 6));
    gesture2.addPoint(cv::Point(5, 4));
    gesture2.addPoint(cv::Point(6, 4));
    gesture2.addPoint(cv::Point(7, 2));
    gesture2.addPoint(cv::Point(9, 1));
    gesture2.addPoint(cv::Point(10, 1));
    gesture2.addPoint(cv::Point(12, 0));
    gesture2.addPoint(cv::Point(13, 0));
    gesture2.addPoint(cv::Point(16, 0));
    gesture2.addPoint(cv::Point(18, 0));
    gesture2.addPoint(cv::Point(20, 0));

    gestures.push_back(gesture1);
    gestures.push_back(gesture2);
}

std::string GestureClassifier::classify(std::vector<cv::Point> gesturePath) {   
    for (int i = 0; i < gestures.size(); i++) {                // Budu projizdet vsechny vytvorene gesta
    std::cout << "Pokus o klasifikaci klasifikatorem: " << gestures.at(i).getName() << "\n";
        if (findGestureInPath(gestures.at(i), gesturePath)) {  
            return gestures.at(i).getName();                   // Pokud nejake klasifikuju, vratim jeho jmeno
        }
    }

    return "None";                                             // Pokud se zadne neklasifikuje, vrati se "None"
};

bool GestureClassifier::findGestureInPath(Gesture gesture, std::vector<cv::Point> gesturePath) {
    for (cv::Point gesturePoint : gesture.gesturePoints) {     // Budu projizdet vsechny body gesta
        bool pointFound = false;
        int eraseTo = 0;
        
        for (int i = 0; i < gesturePath.size(); i++) {         // Pro kazdy bod gesta budu projizdet body posunu ruky
            
            cv::Point pathPoint = gesturePath.at(i);
            int distance = abs(gesturePoint.x - pathPoint.x) + abs(gesturePoint.y - pathPoint.y);
            if (distance <= toleration) {
                pointFound = true;                             // Pokud najdu shodu v toleranci, zaznacim si to a vyjdu z cyklu
                eraseTo = i + 1;
                //std::cout << gesturePath.at(i) << "=?" << gesturePoint << "\n";
                //std::cout << "Match v cyklu: "  << i << ", pocet vymazanych pointu: " << eraseTo << "\n";
                break;
            }
        }

        if (pointFound)
            // Vymazu z cesty jiz projite body. 
            // PS: (findGestureInPath metoda ma kopii toho vektoru pohybu, takze v dalsim volani bude zase cely) 
            // PS2: nepremyslej nad tim zapisem, proste just C++ things
            gesturePath.erase(gesturePath.begin(), gesturePath.begin() + eraseTo);
        else                                                   
            return false;                                      // Pokud jsem projel cely vektro pohybu a nic ne nenaslo, gesto tam nebude                               
    }

    return true;                                               // Pokud to dojde az sem, znamena to, ze se uspesne zpracovali vsechny body z gesta
}
