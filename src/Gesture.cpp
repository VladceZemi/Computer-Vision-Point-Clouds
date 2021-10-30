#include "Gesture.hpp"

Gesture::Gesture(std::string name) {
    this->name = name;
}

std::string Gesture::getName() {
    return name;
}

void Gesture::addPoint(cv::Point point){
    gesturePoints.push_back(point);
};