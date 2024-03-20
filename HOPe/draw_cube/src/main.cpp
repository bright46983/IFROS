#include <string>
#include <iostream>
#include <opencv2/aruco.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>
#include "aruco_samples_utility.hpp"

void drawCube(cv::InputOutputArray image, cv::InputArray camera_matrix, cv::InputArray dist_coeffs, cv::InputArray rvec, cv::InputArray tvec,
    float length)
{
    float half_length = length / 2.0;

    // Project cube points
    std::vector<cv::Point3f> axis_points;
    axis_points.push_back(cv::Point3f(half_length, half_length, length));
    axis_points.push_back(cv::Point3f(half_length, -half_length, length));
    axis_points.push_back(cv::Point3f(-half_length, -half_length, length));
    axis_points.push_back(cv::Point3f(-half_length, half_length, length));
    axis_points.push_back(cv::Point3f(half_length, half_length, 0));
    axis_points.push_back(cv::Point3f(half_length, -half_length, 0));
    axis_points.push_back(cv::Point3f(-half_length, -half_length, 0));
    axis_points.push_back(cv::Point3f(-half_length, half_length, 0));

    std::vector<cv::Point2f> image_points;
    cv::projectPoints(
        axis_points, rvec, tvec, camera_matrix, dist_coeffs, image_points
    );

    // Draw cube edges lines
    cv::line(image, image_points[0], image_points[1], cv::Scalar(255, 0, 0), 3);
    cv::line(image, image_points[0], image_points[3], cv::Scalar(255, 0, 0), 3);
    cv::line(image, image_points[0], image_points[4], cv::Scalar(255, 0, 0), 3);
    cv::line(image, image_points[1], image_points[2], cv::Scalar(255, 0, 0), 3);
    cv::line(image, image_points[1], image_points[5], cv::Scalar(255, 0, 0), 3);
    cv::line(image, image_points[2], image_points[3], cv::Scalar(255, 0, 0), 3);
    cv::line(image, image_points[2], image_points[6], cv::Scalar(255, 0, 0), 3);
    cv::line(image, image_points[3], image_points[7], cv::Scalar(255, 0, 0), 3);
    cv::line(image, image_points[4], image_points[5], cv::Scalar(255, 0, 0), 3);
    cv::line(image, image_points[4], image_points[7], cv::Scalar(255, 0, 0), 3);
    cv::line(image, image_points[5], image_points[6], cv::Scalar(255, 0, 0), 3);
    cv::line(image, image_points[6], image_points[7], cv::Scalar(255, 0, 0), 3);
}

int main(int argc, char *argv[]) {
   // getting commandline arguments
    std::string dictName = argv[1];
    int markerId = std::stoi(argv[2]);
    float markerSize = std::stof(argv[3]); // in meters

    std::string cameraParamsFilename = "/home/tanakrit-ubuntu/IFROS/HOPe/pose_estimation/param/intrinsics.txt";

    int dictId;
    if (dictName == "DICT_4X4_50") dictId = cv::aruco::DICT_4X4_50;
    else if (dictName == "DICT_4X4_100") dictId = cv::aruco::DICT_4X4_100;
    else if (dictName == "DICT_4X4_250") dictId = cv::aruco::DICT_4X4_250;
    else if (dictName == "DICT_4X4_1000") dictId = cv::aruco::DICT_4X4_1000;
    else if (dictName == "DICT_5X5_50") dictId = cv::aruco::DICT_5X5_50;
    else if (dictName == "DICT_5X5_100") dictId = cv::aruco::DICT_5X5_100;
    else if (dictName == "DICT_5X5_250") dictId = cv::aruco::DICT_5X5_250;
    else if (dictName == "DICT_5X5_1000") dictId = cv::aruco::DICT_5X5_1000;
    else if (dictName == "DICT_6X6_50") dictId = cv::aruco::DICT_6X6_50;
    else if (dictName == "DICT_6X6_100") dictId = cv::aruco::DICT_6X6_100;
    else if (dictName == "DICT_6X6_250") dictId = cv::aruco::DICT_6X6_250;
    else if (dictName == "DICT_6X6_1000") dictId = cv::aruco::DICT_6X6_1000;
    else if (dictName == "DICT_7X7_50") dictId = cv::aruco::DICT_7X7_50;
    else if (dictName == "DICT_7X7_100") dictId = cv::aruco::DICT_7X7_100;
    else if (dictName == "DICT_7X7_250") dictId = cv::aruco::DICT_7X7_250;
    else if (dictName == "DICT_7X7_1000") dictId = cv::aruco::DICT_7X7_1000;
    else {
        std::cerr << "Unknown dictionary name: " << dictName << std::endl;
        return 1;
    }

    // Set up camera parameters
    cv::Mat cameraMatrix, distCoeffs;
    readCameraParameters(cameraParamsFilename, cameraMatrix, distCoeffs); // This function is implemented in aruco_samples_utility.hpp
    // Set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerSize/2.f, markerSize/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerSize/2.f, markerSize/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerSize/2.f, -markerSize/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerSize/2.f, -markerSize/2.f, 0);


   // Setup aruco detector object
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dictId);
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
        std::cout << "Marker " << markerSize << std::endl;

        // draw visualization to outputImaage
        cv::Mat outputImage = inputImage.clone();
         // If at least one marker detected
        if (markerIds.size() > 0) {
            int nMarkers = markerCorners.size();
            std::vector<cv::Vec3d> rvecs, tvecs;
            // Calculate pose for each marker
            for (int i = 0; i < nMarkers; i++) {
                if (markerIds.at(i) == markerId){
                    cv::aruco::estimatePoseSingleMarkers(markerCorners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs);
                }
                
                }
            // Draw axis for each marker and  position
            for(unsigned int i = 0; i < markerIds.size(); i++) {
                if (markerIds.at(i) == markerId){
                drawCube(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 1);
                }
            }
        }

        // stream output image
        cv::imshow("Aruco Detection", outputImage);

        charCheckForESCKey = cv::waitKey(1);  // gets the key pressed
    }

    return 0;
}

