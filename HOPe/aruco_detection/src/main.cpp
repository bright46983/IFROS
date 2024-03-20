#include <string>
#include <iostream>
#include <opencv2/aruco.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>



int main(int argc, char *argv[]) {

    // Setup aruco detector object
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    // Video streaming
    cv::VideoCapture webCam(0);       // VideoCapture object declaration. Usually 0 is the integrated, 2 is the first external USB one
    cv::Mat inputImage;
    char charCheckForESCKey{0};

    if (webCam.isOpened() == false){   // Check if the VideoCapture object has been correctly associated to the webcam
    std::cerr << "error: Webcam could not be connected." << std::endl;
    return -1;
     }
    while (charCheckForESCKey != 27 && webCam.isOpened()){ 
        bool frameSuccess = webCam.read(inputImage);           // get next frame from input stream

        if (!frameSuccess || inputImage.empty()){              // if the frame was not read or read wrongly
        std::cerr << "error: Frame could not be read." << std::endl;
        break;
        }

        // start detection process
        detector.detectMarkers(inputImage, markerCorners, markerIds, rejectedCandidates);

        // draw visualization to outputImaage
        cv::Mat outputImage = inputImage.clone();
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

        // stream output image
        cv::imshow("Aruco Detection", outputImage);

        charCheckForESCKey = cv::waitKey(1);  // gets the key pressed
    }

   

    return 0;
}
