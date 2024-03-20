#include <string>
#include <iostream>
#include <opencv2/aruco.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>



int main(int argc, char *argv[]) {

    std::string dictName = argv[1];
    int markerId = std::stoi(argv[2]);
    int markerSize = std::stoi(argv[3]);
    std::string arucoName = argv[4];
    
    //std::string directory = "~/IFROS/HOPe/generate_marker/img/";
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

    cv::Mat markerImage;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dictId);
    cv::aruco::generateImageMarker(dictionary, markerId, markerSize, markerImage, 1);
    //std::string arucoPath = directory + arucoName;
    cv::imwrite(arucoName, markerImage);


    return 0;
}
