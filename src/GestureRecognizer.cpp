#include "GestureRecognizer.hpp"

GestureRecognizer::GestureRecognizer() {
//    Gesture openGesture = Gesture("open");
//    openGesture.addPoint(cv::Point(0, 0));
//    openGesture.addPoint(cv::Point(10, 10));
//    openGesture.addPoint(cv::Point(20, 20));
//    openGesture.addPoint(cv::Point(30, 35));
//    openGesture.addPoint(cv::Point(35, 45));
        std::vector<cv::Point> a1;
        std::vector<cv::Point> a2;
        std::vector<cv::Point> a3;
        gesturePath.push_back(a1);
        gesturePath.push_back(a2);
        gesturePath.push_back(a3);
}

void GestureRecognizer::process(cv::Mat image) {
    this->image = image;

    preprocessImage();
    morphologyOperations();
    findAndFilterContours();
    
    
    cv::imshow("Preprocessed Image", preprocessedImage);
}

void GestureRecognizer::preprocessImage() {
    cv::cvtColor(image, preprocessedImage, cv::COLOR_BGR2HSV);
    cv::inRange(preprocessedImage, cv::Scalar(0, 42, 100), cv::Scalar(50, 255, 255), preprocessedImage);
}

void GestureRecognizer::morphologyOperations() {
    cv::erode(preprocessedImage, preprocessedImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));
    cv::dilate(preprocessedImage, preprocessedImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)));
}

void GestureRecognizer::findAndFilterContours() {
    cv::findContours(preprocessedImage, contours, hiearchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point> vectorOfContours;
    for(int i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> polygon;
        cv:approxPolyDP(contours.at(i), polygon, 5, true);
        
        double area = cv::contourArea(polygon);
        
        if (area > 3500 && area < 9000){
            cv::Rect rect = cv::boundingRect(polygon);
            float x = rect.tl().x + rect.width/2;
            float y = rect.tl().y + rect.height/2;
            cv::Point point = cv::Point(x,y);
            cv::circle(image, point, 20, cv::Scalar(0,255,0));
            gesturePath.at(i).push_back(point);
            std::cout << gesturePath.at(i) << std::endl;
        }
    }
    gesturePath.push_back(vectorOfContours);
}

void GestureRecognizer::normalizeMovementPath() {
    int x_max = 0, x_min = 640, y_max = 0, y_min = 480;
    for (int j = 0; j < gesturePath.size(); j++){
        for (int i = 0; i < gesturePath.at(j).size(); i++) {
            cv::Point p = gesturePath.at(j).at(i);
            if (p.x > x_max)
                x_max = p.x;
            if (p.x < x_min)
                x_min = p.x;
            if (p.y > y_max)
                y_max = p.y;
            if (p.y < y_min)
                y_min = p.y;
                
        }
    }



    int x_size = abs(x_max - x_min);
    float x_factor = 20.0 / x_size;
    
    std::cout << "Faktor: " << x_factor << " x_size: " << x_size << std::endl;

    for (int j = 0; j < gesturePath.size(); j++){
        std::cout << "okokok" << std::endl;
        for (int i = 0; i < gesturePath.at(j).size(); i++) {
            cv::Point p = gesturePath.at(j).at(i);
            // x faktor je ziskany koeficient ktery se ziska ze vztahu aktualniho max a min x a zvolene velikosti (100)
            p.x = (p.x - x_min) * x_factor;
            p.y = (p.y - y_min) * x_factor;
            normalizedGesturePath.at(j).push_back(p);
        //  std::cout << p.x << ", " << p.y << std::endl;
        }
    }
}

void GestureRecognizer::filterMotionLessObjects(){
    // int x_max = 0, x_min = 640, y_max = 0, y_min = 480;
    // for(int i = 0; normalizedGesturePath.size(); i++){
    //     for(int j = 0; normalizedGesturePath.at(i).size()){
    //     cv::Point p = gesturePath.at(j);
    //     if (p.x > x_max)
    //         x_max = p.x;
    //     if (p.x < x_min)
    //         x_min = p.x;
    //     if (p.y > y_max)
    //         y_max = p.y;
    //     if (p.y < y_min)
    //         y_min = p.y;
    //     }
    //     int distance = x_max - x_min + y_max - y_min;
    //     if distance <= minMotionDistance{


    //         }
    // }
}


void GestureRecognizer::recognize() {
    GestureClassifier classifier = GestureClassifier();
    

    // normalizeMovementPath();
    // std::string gestureName = classifier.classify(normalizedGesturePath);
    // std::cout << "Jmeno gesta: " << gestureName << "\n";

    for (cv::Point p : gesturePath.at(0)) {
        cv::circle(image, p, 5, cv::Scalar(255, 0, 0));
    }  
    for (cv::Point p : gesturePath.at(1)) {
        cv::circle(image, p, 5, cv::Scalar(0, 255, 0));
    }  
    for (cv::Point p : gesturePath.at(2)) {
        cv::circle(image, p, 5, cv::Scalar(0, 0, 255));
    }
    cv::imshow("hue", image);
}
