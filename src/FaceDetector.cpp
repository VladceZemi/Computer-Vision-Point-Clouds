#include "FaceDetector.hpp"

FaceDetector::FaceDetector(){
    classifier.load("data/haarcascade_frontalface_alt.xml");
    
    
}
std::vector<cv::Rect> FaceDetector::getFaces(cv::Mat image){
    cv::Mat grayscaleImage;
    cv::cvtColor(image, grayscaleImage, cv::COLOR_BGRA2GRAY);
    std::vector<cv::Rect> faces;
    classifier.detectMultiScale(grayscaleImage, faces);
    return faces;
    
}
