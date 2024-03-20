#include <string>
#include <iostream>
#include <opencv2/aruco.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>



int main(int argc, char *argv[]) {

    int numRows = std::stoi(argv[1]);
    int numCols = std::stoi(argv[2]);
    std::string dictName = argv[3];
    int markerSize = std::stoi(argv[4]);
    int spaceSize = std::stoi(argv[5]);
    std::string arucoName = argv[6];
    
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
    
 


    
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dictId);
    cv::aruco::GridBoard gridBoard(cv::Size(numCols, numRows), markerSize, spaceSize, dictionary);  

    cv::Mat boardImage;
    cv::Size imageSize;
    int margin = spaceSize/2;
    imageSize.width = numCols * (markerSize + spaceSize) -  spaceSize + 2 * margin;
    imageSize.height = numRows * (markerSize + spaceSize) - spaceSize + 2 * margin;

    gridBoard.generateImage(imageSize, boardImage,margin);
  
    cv::imwrite(arucoName, boardImage);


    return 0;
}
