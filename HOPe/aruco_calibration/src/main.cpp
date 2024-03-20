#include <string>
#include <iostream>
#include <opencv2/aruco.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/calib3d.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>
#include "aruco_samples_utility.hpp"


int main(int argc, char *argv[]) {
    
    // getting commandline arguments
    std::string dictName = argv[1];
    std::string detectorParamFile = argv[2];
    int numRows = std::stoi(argv[3]);
    int numCols = std::stoi(argv[4]);
    int markerSize = std::stoi(argv[5]);
    int spaceSize = std::stoi(argv[6]);
    std::string outputFile = argv[7];

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

    // handle yaml
    cv::FileStorage fs(detectorParamFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open YAML file: " << detectorParamFile << std::endl;
        return -1;
    }
    cv::FileNode arucoParamsNode = fs.root();
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    detectorParams.readDetectorParameters(arucoParamsNode) ;    
    fs.release();

    // Setup aruco detector object
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dictId);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::GridBoard gridboard(cv::Size(numCols, numRows), markerSize, spaceSize, dictionary);    

    

    // // Video streaming
    cv::VideoCapture webCam(0);       // VideoCapture object declaration. Usually 0 is the integrated, 2 is the first external USB one
    cv::Mat inputImage;
    char charCheckKey{0};

    // data for calibration
    std::vector< std::vector< std::vector< cv::Point2f > > > corners;
    std::vector< std::vector< int > > ids;
    cv::Size imgSize;


    if (webCam.isOpened() == false){   // Check if the VideoCapture object has been correctly associated to the webcam
    std::cerr << "error: Webcam could not be connected." << std::endl;
    return -1;
     }
    while (charCheckKey != 27 && webCam.isOpened()){ 
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


        // capture image
        charCheckKey = cv::waitKey(1);
        if (charCheckKey == 'c'){
            std::cout << "Capturing image" << std::endl;
            corners.push_back(markerCorners);
            ids.push_back(markerIds);
            imgSize = inputImage.size();
        }
    }

    if(ids.size() < 15) {
        std::cerr << "Not enough images for calibration - need at least 15 frames" << std::endl;
        return 0;
    }
    std::cout << "Start Calibration Process ... " << std::endl;
    cv::Mat cameraMatrix, distCoeffs;

    // Prepare data for calibration
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Mat> processedObjectPoints, processedImagePoints;
    size_t nFrames = corners.size();

    for(size_t frame = 0; frame < nFrames; frame++) {
        cv::Mat currentImgPoints, currentObjPoints;

        gridboard.matchImagePoints(
            corners[frame], ids[frame],
            currentObjPoints, currentImgPoints
        );

        if(currentImgPoints.total() > 0 && currentObjPoints.total() > 0) {
            processedImagePoints.push_back(currentImgPoints);
            processedObjectPoints.push_back(currentObjPoints);
        }
    }

    // calibrate camera
    double repError = cv::calibrateCamera(processedObjectPoints, processedImagePoints,
                                           imgSize, cameraMatrix, distCoeffs, cv::noArray(),cv::noArray(),cv::noArray(),cv::noArray(),cv::noArray(),0);

    

    bool saveOk = saveCameraParams(
        outputFile, imgSize, 1, 0, cameraMatrix,
        distCoeffs, repError
    );

    std::cout << "Calibration file is written to" << outputFile << std::endl;
    return 0;
}
