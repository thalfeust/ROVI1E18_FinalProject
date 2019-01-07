/*
   RoVi1

   Todo

   -Color segmentation needs to be checked, it fails on a few of the hard markers.
   -Region of interest?

   Done
   -Seperate the image into two colors, one for red, one for blue
   -Instead of RGB, use HSV color sergmentation
   -Circles hough transform works on easy sequence but not hard. Tilt of the maker is too much
   Need to try the contour method
   -Get centroid of contorus


   Version: $$version$$
 */

#include <iostream>

#include "FeatureExtraction.hpp"

using namespace cv;
using namespace std;

int ddepth = CV_16S;

int ct = 0;

int erosion_size = 1;
int dilation_size = 5;

int main(int argc, char* argv[])
{
								//For performance eval
								int64 t0 = cv::getTickCount();
								cv::CommandLineParser parser(argc, argv,"{help||print this message}" "{@path| ../data/marker_color/ | path}");

								if (parser.has("help")) {
																parser.printMessage();
																return 0;
								}

								// Get the path of the folder
								cv::String folder = parser.get<String>("@path");
								folder += "*.png";

								FeaturesDections features;
								ColorSegmentation extraction_CS;
								extraction_CS.set( 2000.0, 0.78, dilation_size, erosion_size, cv::Size(9, 9), cv::Scalar(0, 151, 0), cv::Scalar(010, 255, 255), cv::Scalar(90, 80, 10), cv::Scalar(180, 200, 150));

								cv::namedWindow(folder, cv::WINDOW_NORMAL);

								// load the names of the files
								std::vector<cv::String> fileNames = features.loadFiles(folder);

								for(size_t i = 0; i < fileNames.size(); ++i)
								{
																// Preparation of the output
																cv::Mat matArrayOutputLine1[2];
																cv::Mat matArrayOutputLine2[2];
																cv::Mat matArrayOutputLine3[2];

																//Load the image
																Mat img = imread(fileNames[i]);
																if(!img.data) {
																								cerr << "Problem loading image!!!" << endl;
																								return 0;
																}

																// Output
																matArrayOutputLine1[0] = img;
																//extraction_CS.tick( &img);
																// Isolate Circles by color---------------------

																cv::Mat red_hsv_range = extraction_CS.segmentation( img, "red");
																cv::Mat blue_hsv_range = extraction_CS.segmentation( img, "blue");

																// output
																matArrayOutputLine2[0] = red_hsv_range;
																matArrayOutputLine3[0] = blue_hsv_range;

																// Dilate to get rid of isoled pixels and some noise
																red_hsv_range = extraction_CS.dilate( red_hsv_range);
																blue_hsv_range = extraction_CS.dilate( blue_hsv_range);

																// Erode
																red_hsv_range = extraction_CS.erode( red_hsv_range);
																blue_hsv_range = extraction_CS.erode( blue_hsv_range);

																// Blur image
																red_hsv_range = extraction_CS.gaussianBlur( red_hsv_range);
																blue_hsv_range = extraction_CS.gaussianBlur( blue_hsv_range);

																// Storage of the centers
																std::vector<cv::Point> center;

																// Find the red circle
																matArrayOutputLine2[1] = extraction_CS.FindCircles( red_hsv_range, center, true);

																// Find blue circle
																matArrayOutputLine3[1] = extraction_CS.FindCircles( blue_hsv_range, center, true);

																std::cout << "Number of centers : " << center.size() << std::endl;

																matArrayOutputLine1[1] = img.clone();
																if( center.size()>=4) {

																								std::vector<cv::Point> keypoints = extraction_CS.algoFind3Points( center);
																								std::cout << " Key : " << keypoints[0] <<" "<< keypoints[1] << "\n";

																								if( keypoints.size()>=3) {
																																matArrayOutputLine1[1] =  extraction_CS.drawResult( matArrayOutputLine1[1], keypoints);
																								}
																}
																// Ouput the pictures
																/* DOC : https://docs.opencv.org/3.4.0/d2/de8/group__core__array.html#gaf9771c991763233866bf76b5b5d1776f */

																cv::cvtColor( matArrayOutputLine2[0], matArrayOutputLine2[0], cv::COLOR_GRAY2BGR);
																cv::cvtColor( matArrayOutputLine3[0], matArrayOutputLine3[0], cv::COLOR_GRAY2BGR);

																cv::Mat outL1;
																cv::hconcat( matArrayOutputLine1, 2, outL1);

																cv::Mat outL2;
																cv::hconcat( matArrayOutputLine2, 2, outL2);

																cv::Mat outL3;
																cv::hconcat( matArrayOutputLine3, 2, outL3);

																/* DOC : https://docs.opencv.org/3.4.0/d2/de8/group__core__array.html#ga744f53b69f6e4f12156cdde4e76aed27 */
																cv::Mat output;
																cv::Mat outC[] = { outL1, outL2, outL3};
																cv::vconcat( outC, 3, output);

																cv::imshow(folder, output);

																//For performance eval
																int64 t1 = cv::getTickCount();
																double run_time = (t1-t0)/cv::getTickFrequency();
																std::cout << "Time to run" << run_time << std::endl;

																while (waitKey() != 27)
																								; // (do nothing)
								}


								std::cout << "END" << std::endl;
}
