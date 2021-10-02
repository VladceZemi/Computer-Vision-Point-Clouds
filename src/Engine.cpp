#include "Engine.hpp"


Engine::Engine(){
    videoCapture.open(0);
    windowSize = cv::Size(640,480);
}

void Engine::run(){
    FaceDetector faceDetector = FaceDetector();
    ColorTracker colorTracker = ColorTracker();
    ObjectTracker objectTracker = ObjectTracker();

    for(;;) {
        captureCameraFrame();
    //    std::vector<cv::Rect> faces = faceDetector.getFaces(cameraFrame);
    //    std::cout<< faces.size()<< std::endl;

    //    drawFaces(faces);
        
    //    cv::Point colorPosition = colorTracker.getColorPosition(cameraFrame);
    //    cv::circle(cameraFrame, colorPosition, 20, cv::Scalar(0, 0, 255));

        
        if (pointsToTrack.size() == 0) {
            pointsToTrack = objectTracker.getFeatures(currentFrame);
        }
        else {
            pointsToTrack = objectTracker.trackObject(previousFrame, currentFrame, pointsToTrack);
            objectTracker.drawPoints(cameraFrame, pointsToTrack, cv::Scalar(0, 255, 0));
        }
        
        cv::imshow("Camera Frame", cameraFrame);

        char c = cv::waitKey(1);
        if (c == 27){
            break;
        }
    }
}


void Engine::drawFaces(std::vector<cv::Rect> faceRectangles){
    for (int i = 0; i < faceRectangles.size(); i++){
        cv::rectangle(cameraFrame, faceRectangles.at(i).tl(), faceRectangles.at(i).br(), cv::Scalar(255, 0, 0),5);
    }
}

void Engine::captureCameraFrame(){
    videoCapture >> cameraFrame;
    cv::flip(cameraFrame, cameraFrame, 1);
    cv::resize(cameraFrame, cameraFrame, windowSize);

    currentFrame.copyTo(previousFrame);
    cameraFrame.copyTo(currentFrame);
}


