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

    gestures.push_back(gesture1);
}

std::string GestureClassifier::classify(std::vector<cv::Point> gesturePath) {
    int distance = 0;           //promynna pro pocitani vzdalenosti mezi pointy
    bool isClassified = false;  //promenna pro ulozeni informace zda je klasifikovano    
    int foundCount = 0;         //promena pro ulozeni poctu nalezeni bodu    
    bool onePointFound = false; //promena pro vnitrni logiku, uklada informaci o nalezeni bodu
    int mp = 0;                 //promena ktera uchovava pozici bodu na vektoru klasifikovaneho gesta, aby znovu nebyly pouzity predchozi a nebo pouzite body
    int c = 9;
    
    // for (int i = 0; i < classifierVector.size(); i++){ //index pro vektory, ktere predstavuji klasifikatory
    //     foundCount = 0;
    //     mp = 0;                          
    //     for (int j = 0; j < classifierVector.at(i).size(); j++){  
    //         for (int m = mp; m < gesturePath.size(); m++){      
    //             onePointFound = false;    
    //             distance = abs(classifierVector.at(i).at(j).x - gesturePath.at(m).x); // pocitani distance, jednoduche scitani manhatanovske vzdalenosti
    //             distance += abs(classifierVector.at(i).at(j).y - gesturePath.at(m).y);
    //             if (distance <= tolerationConst){
    //                 mp = m; //aby se jiz stejny nebo predesle body z gesta nepouzili na matchnuti bodu nasledujicich z klasifikatoru
    //                 foundCount++;
    //                 onePointFound = true;
    //             }
    //             distance = 0;
    //             if(onePointFound){break;} //bod byl nalezen nema smysl dale prohledavat gesto na stejny bod, posouvame na dalsi
    //         }
    //         if (!onePointFound){break;} // point nebyl v klasifikatoru nalezen, takze nema smysl hledat dale
    //         if (foundCount >= classifierVector.at(i).size()-1){isClassified = true;}   // jestlize pocet shod byl nalezen tak je gesto klasifikovano
    //         if (isClassified){break;}
    //     }
    //     if (isClassified){
    //         int c = i;
    //         break;
    //     }
    // }
    // if (isClassified){ return c;}
    // else{
    //         return c;
    //     } //vraci cislo klasifikatoru, ktery byl matchnut, jestlize nebyl zadny matchnut vraci 9

};