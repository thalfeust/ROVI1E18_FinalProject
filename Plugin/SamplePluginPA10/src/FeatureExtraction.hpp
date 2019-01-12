#ifndef FEATUREEXTRACTION_HPP
#define FEATUREEXTRACTION_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <dirent.h> // for linux systems
#include <sys/stat.h> // for linux systems
#include <sstream> //For writing multiple images
#include <math.h>
#include <exception>

#define DETECTOR_SURF 1
#define DETECTOR_SIFT 2

class ColorSegmentation {
public:

double area_threshold;
double compactness_limit;
int dilation_size;
int erosion_size;
cv::Size gaussianSize;
cv::Scalar red_lowerBoundary, red_upperBoundary;
cv::Scalar blue_lowerBoundary, blue_upperBoundary;

void plop();
void set( double, double, int, int, cv::Size, cv::Scalar, cv::Scalar, cv::Scalar, cv::Scalar);
cv::Mat FindCircles( const cv::Mat&, std::vector<cv::Point>&, bool);
cv::Mat dilate( const cv::Mat&);
cv::Mat erode( const cv::Mat&);
cv::Mat gaussianBlur( const cv::Mat&);
cv::Mat segmentation( const cv::Mat src, std::string mode);
cv::Mat drawResult( const cv::Mat& src, std::vector<cv::Point> center);
std::vector<cv::Point> algoFind3Points( const std::vector<cv::Point> center);
cv::Mat tick( cv::Mat, std::vector<cv::Point>&, bool);
};

class PointsFeatures {
public:

int detectorMode;
std::vector<cv::Point2f> bb_originalObject;

PointsFeatures( int, cv::Mat);
std::vector<cv::Point2f> keypoint_coordinates(const std::vector<cv::KeyPoint>&);
std::vector<std::vector<cv::DMatch> > detect_match( cv::Mat, cv::Mat, std::vector<cv::KeyPoint>*, std::vector<cv::KeyPoint>*);
std::vector<cv::DMatch> matchQuality( std::vector<std::vector<cv::DMatch> >, std::vector<cv::KeyPoint>*, std::vector<cv::KeyPoint>*);
std::vector<cv::Point> extractPoints( std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>, std::vector<char>*, std::vector<cv::Point2f>*, std::size_t, bool);
void drawComparaison( cv::Mat*, const cv::Mat, const cv::Mat, std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>, std::vector<cv::DMatch>, std::vector<cv::Point>, std::vector<char>, std::vector<cv::Point2f>);
void draw_bb(cv::Mat& img,
             const std::vector<cv::Point2f>& bb,
             const cv::Point2f& offset);
};

class FeaturesDections {
public:
std::vector<cv::String> loadFiles( cv::String);
};

#endif
