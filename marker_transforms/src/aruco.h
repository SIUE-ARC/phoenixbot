/*
 * aruco.h
 *
 * Created on April 4, 2017
 *      Author: Ryan Owens
 */
#ifndef INCLUDE_ARC_ARUCO_HPP_
#define INCLUDE_ARC_ARUCO_HPP_

#include <algorithm>
#include <fstream>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <unistd.h>

class Aruco {
public:
Aruco(int dicionaryId);
~Aruco();
bool openCamera();
bool arucoMarkerInView(int arucoId, cv::Mat *frame = nullptr);
bool setDictionary(int dictionaryId);
std::vector<int> arucoMarkersInView(cv::Mat *frame = nullptr);
std::vector<std::vector<cv::Point2f> >
arucoMarkerCorners(cv::Mat *frame = nullptr);
std::vector<double> getPose(int arucoId, cv::Mat *frame = nullptr);
void setChessBoardSize(float sizeInMeters);
void setArucoMarkerSize(float sizeInMeters);
bool calibrate(std::vector<cv::Mat> images);
bool setCameraCalibration(std::string filename);

private:
bool flipVertical = false;
float chessBoardSquareSize = 0.0235f;   // Meters
float arucoSquareSize = 0.093f;         // Meters
int numImagesForCalibration = 15;
cv::Size chessBoardDimensions = cv::Size(6, 9);
std::vector<cv::Mat> images;
std::vector<std::vector<cv::Point2f> > foundChessboardCorners;
cv::Mat cameraMatrix, distortionCoefficients;
int dictionaryId;
int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH |
                      cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;

std::string currentCalibrationFile = "";
std::string customDictionaryFile =
        "kipr_dictionary.yml";          // TODO need to get the proper path
std::string calibrationFilePath = "";   // TODO neet to get the proper path
cv::Ptr<cv::aruco::Dictionary> dictionary;
cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
cv::VideoCapture camera;

const std::string WHITE_CAMERA_FILE = "white_2016.yml";
const std::string BLACK_CAMERA_FILE = "black_2017.yml";

cv::Mat getFrame();
bool saveCalibration();
void getImagesFromCamera();
bool getCustomDictionary();
bool readCameraCalibration(std::string filename);
void calculateChessBoardPosition(std::vector<cv::Point3f> &vec);
bool vectorContains(std::vector<int> vec, int val);
void calculateChessBoardCornersFromImages(
        std::vector<std::vector<cv::Point2f> > &foundCorners);
};

#endif
