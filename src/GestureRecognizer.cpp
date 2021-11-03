#include "GestureRecognizer.hpp"

GestureRecognizer::GestureRecognizer() {

}

void GestureRecognizer::process(cv::Mat image) {
    this->image = image;
    image.copyTo(groupsImage);

    preprocessImage();
    morphologyOperations();
    findAndFilterContours();
    
    cv::imshow("preprocess", preprocessedImage);
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

    for(int i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> polygon;
        cv:approxPolyDP(contours.at(i), polygon, 5, true);
        
        double area = cv::contourArea(polygon);
        
        if (area > 2500 && area < 8000){
            cv::Point handPoint = createHandPoint(polygon);
            groupPoints(handPoint);
        }
    }

}

cv::Point GestureRecognizer::createHandPoint(std::vector<cv::Point> polygon) {
    cv::Rect rect = cv::boundingRect(polygon);
    float x = rect.tl().x + rect.width / 2;
    float y = rect.tl().y + rect.height / 2;
    return cv::Point(x, y);
}

void GestureRecognizer::drawGroups() {
    cv::Scalar groupColor[4] = {cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(0, 0, 0)};

    for (int i = 0; i < handsPath.size(); i++) {
        auto group = handsPath.at(i);
        for (cv::Point point : group) {
            cv::circle(groupsImage, point, 5, groupColor[i]);
        }
    }

    cv::imshow("groupsImage", groupsImage);
}

void GestureRecognizer::groupPoints(cv::Point handPoint) {
    if (handsPath.empty()) {
        std::vector<cv::Point> firstVector;
        firstVector.push_back(handPoint);
        handsPath.push_back(firstVector);
        return;
    }

    bool grouped = false;
    for (int i = 0; i < handsPath.size(); i++) {
        cv::Point lastPoint = handsPath.at(i).back();
        int distance = abs(handPoint.x - lastPoint.x) + abs(handPoint.y - lastPoint.y);

        if (distance <= groupPointTreshold) {
            handsPath.at(i).push_back(handPoint);
            grouped = true;
            break;
        }

    }

    if (!grouped) {
        auto lastPoints = getLastPointsFromHandsPath();

        for (auto lastPoint : lastPoints) {
            int distance = abs(handPoint.x - lastPoint.x) + abs(handPoint.y - lastPoint.y);
            
            if (distance <= ungroupPointTreshold) {
                std::vector<cv::Point> newGroup;
                newGroup.push_back(handPoint);
                handsPath.push_back(newGroup);
            }
        }
    }

    drawGroups();
}

std::vector<cv::Point> GestureRecognizer::getLastPointsFromHandsPath() {
    std::vector<cv::Point> lastPoints;

    for (std::vector<cv::Point> handPath : handsPath) {
        lastPoints.push_back(handPath.back());
    }

    return lastPoints;
}

void GestureRecognizer::normalizeMovementPath() {
    int x_max = 0, x_min = 640, y_max = 0, y_min = 480;
    for (int j = 0; j < handsPath.size(); j++){
        for (int i = 0; i < handsPath.at(j).size(); i++) {
            cv::Point p = handsPath.at(j).at(i);
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

    for (int j = 0; j < handsPath.size(); j++){
        std::vector<cv::Point> normalizedPath;

        for (int i = 0; i < handsPath.at(j).size(); i++) {
            cv::Point p = handsPath.at(j).at(i);
            p.x = (p.x - x_min) * x_factor;
            p.y = (p.y - y_min) * x_factor;
            normalizedPath.push_back(p);
        }

        normalizedHandsPath.push_back(normalizedPath);
    }
}

bool isMotionless(std::vector<cv::Point> gesture) {
    int minMotionDistance = 30;
    int x_max = 0, x_min = 640, y_max = 0, y_min = 480;

    for(int i = 0; i < gesture.size(); i++) {
        cv::Point p = gesture.at(i);
        if (p.x > x_max)
            x_max = p.x;
        if (p.x < x_min)
            x_min = p.x;
        if (p.y > y_max)
            y_max = p.y;
        if (p.y < y_min)
            y_min = p.y;
        }
    
    int distance = x_max - x_min + y_max - y_min;
    
    bool rvalue = (distance <= minMotionDistance);
    return rvalue;
}

void GestureRecognizer::filterMotionlessObjects() {
    remove_if(handsPath.begin(), handsPath.end(), isMotionless);

    for(int i = 0; i < handsPath.size(); i++){
        std::cout << handsPath[i] << std::endl;
    }
}

std::string GestureRecognizer::recognize() {
    GestureClassifier classifier = GestureClassifier();
    
    filterMotionlessObjects();

    image.copyTo(groupsImage);
    drawGroups();
    
    normalizeMovementPath();

    if(normalizedHandsPath.size() > 1) { 
        return classifier.classify(normalizedHandsPath.at(0), normalizedHandsPath.at(1));
    }
    else {
        std::vector<cv::Point> emptyVector;
       return classifier.classify(normalizedHandsPath.at(0), emptyVector);
    };
}
