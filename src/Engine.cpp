#include "Engine.hpp"

Engine::Engine(){
    // videos.push_back("videos/leva_nahoru.mp4");
    // videos.push_back("videos/prava_nahoru.mp4");
    videos.push_back("videos/doleva.mp4");
    // videos.push_back("videos/roztazeni_orez.mp4");
    //videos.push_back("videos/leva_dolu.mp4");
    
    windowSize = cv::Size(640, 480);
    isRunning = true;
}

void Engine::run(){
    for (int i = 0; i < videos.size(); i++) {
        gestureRecognizer = GestureRecognizer();
        processVideo(videos.at(i));
        if (!isRunning) {
            break;
        }
    }
}

void Engine::processVideo(std::string videoName) {
    videoCapture.open(videoName);
    videoCapture >> frame;
    
    while (!frame.empty() && isRunning) {
        processCameraFrame(frame);
        
        int delay = 1;
        char c = cv::waitKey(delay);
        if (c == 27) {
            isRunning = false;
            break;
        }
        
        videoCapture >> frame;
    }
    std::string gestureName = gestureRecognizer.recognize();
}

void Engine::processCameraFrame(cv::Mat frame){
    cv::resize(frame, frame, windowSize);
    gestureRecognizer.process(frame);
}
