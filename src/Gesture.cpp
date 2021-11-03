#include "Gesture.hpp"

Gesture::Gesture(std::string name) {
    this->name = name;
}

std::string Gesture::getName() {
    return name;
}

void Gesture::addPoint(cv::Point point) {
    gesturePoints.push_back(point);
}

void Gesture::addPointSec(cv::Point point) {
    gesturePointsSec.push_back(point);
}

bool Gesture::isTwoHanded() {
    return !gesturePointsSec.empty();
}
