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
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <dirent.h> // for linux systems
#include <sys/stat.h> // for linux systems
#include <sstream> //For writing multiple images
#include <iostream>

using namespace cv;
using namespace std;

int ddepth = CV_16S;

int ct = 0;

int erosion_size = 1;
int dilation_size = 5;

cv::Mat FindCircles( const cv::Mat& circles_hsv_image, std::vector<Point> &center)
{
	
	cv::Mat drawing = cv::Mat::zeros( circles_hsv_image.size(), CV_8UC3 );
	std::cout << drawing.type() << "rharha\n" ;
	//Find countours-------------------------------------------------------------
	//Alternative to HoughCircles, should work better on hard sequences.
	//Using code from Lab week 9 (guest lecture)

	//Invert the colors so that the contour will work
	cv::bitwise_not ( circles_hsv_image, circles_hsv_image );

	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;
	/// Find contours
	cv::findContours( circles_hsv_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	// Start random number generator with a known seed, for color labeling
	RNG rng(12345);

	/// Draw contours for every oject in the image
	for( int i = 0; i< contours.size(); i++ )
	{
		//std::vector<Point> center;
		float radius;
		// Calculate perimeter length
		double perimeter = arcLength(contours[i], 1);
		double area = contourArea(contours[i], true);
		//This bit is a "trick" to get rid of small acrifacts, no the ideal way to do it.
		double area_threshold = 2000;
		if (area < area_threshold)
		{	perimeter = 10000;
			area = 0;}

		double compactness = (4 * 3.141592 * area) / (perimeter * perimeter);
		printf("perimeter: %8.3f  area: %8.3f   compactness: %8.3f\n", perimeter, area, compactness);


		if(compactness < 0.78)
		{
			// Draw contour for things smaller than
			Scalar color = cv::Scalar( 0, 0, 255);
			cv::drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point());
		}
		else
		{
			// Circle found, draw contour
			Scalar color = cv::Scalar( 0, 255, 0);
			cv::drawContours( drawing, contours, i, color, 4, 8, hierarchy, 0, Point());

			//Calculate the centers
			//Getting this to work was waaaay harder than it should have been.
			//https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/bounding_rects_circles/bounding_rects_circles.html
			//minEnclosingCircle(contours[i], center, radius);
			//Printin the centers
			//std::cout << "Circle Center: " << center << std::endl;

			// Get the moments of the contour
			cv::Moments mu = moments( contours[i], false );

			///  Get the mass centers:
  		Point mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
			std::cout << "Circle Center from Moments: " << mc << std::endl;
			center.push_back( mc);
		}
	}
	return drawing;
}

std::vector<Point> algoFind3Points( const std::vector<Point> center) {

	std::vector<cv::Point> toReturn;

	if (center.size() < 4) {
		return toReturn;
	}

	// We assume the first center must be the center of the red cercle
	cv::Point red = center[0];
	toReturn.push_back( red);

	double d_b1 = cv::norm(red-center[1]);
	double d_b2 = cv::norm(red-center[2]);
	double d_b3 = cv::norm(red-center[3]);

	std::vector<Point> b;
	cv::Point opposite;

	if( d_b1 > d_b2 && d_b1 > d_b3) {
		opposite = center[1];
		b.push_back(center[2]);
		b.push_back(center[3]);
	}else if( d_b2 > d_b3 && d_b2 > d_b1) {
		opposite = center[2];
		b.push_back(center[1]);
		b.push_back(center[3]);
	}else if( d_b3 > d_b2 && d_b3 > d_b1) {
		opposite = center[3];
		b.push_back(center[1]);
		b.push_back(center[2]);
	}else {
		// failled
		return toReturn;
	}

	if( red.y < opposite.y) {
		if( red.x <= opposite.x) {
			// Red on TOP LEFT wre want the right cercle
			if( b[0].x > b[1].x) {
				toReturn.push_back(b[0]);
			}else {
				toReturn.push_back(b[1]);
			}
		}else {
			// Red on TOP RIGHT wre want the right cercle
			if( b[0].x > b[1].x) {
				toReturn.push_back(b[0]);
			}else {
				toReturn.push_back(b[1]);
			}
		}
	}else {
		if( red.x <= opposite.x) {
			// Red on BOTTOM LEFT wre want the left cercle
			if( b[0].x < b[1].x) {
				toReturn.push_back(b[0]);
			}else {
				toReturn.push_back(b[1]);
			}
		}else {
			// Red on BOTTOM RIGHT wre want the left cercle
			if( b[0].x < b[1].x) {
				toReturn.push_back(b[0]);
			}else {
				toReturn.push_back(b[1]);
			}
		}
	}

	toReturn.push_back(opposite);
	return toReturn;
}

int main(int argc, char* argv[])
{
	//For performance eval
	int64 t0 = cv::getTickCount();
	cv::CommandLineParser parser(argc, argv,
			"{help     |                  | print this message}"
			"{@path  | ../data/marker_color/ | path}"
	);

	if (parser.has("help")) {
			parser.printMessage();
			return 0;
	}

	// Get the path of the folder
	std::string folder = parser.get<std::string>("@path");
	folder += "*.png";

	cv::namedWindow(folder, cv::WINDOW_NORMAL);
	//Load multiple images
	/* DOC : http://answers.opencv.org/question/58078/reading-a-sequence-of-files-within-a-folder/ */
	// notice here that we are using the Opencv's embedded "String" class
	vector<String> filenames;
  glob(folder, filenames);

	for(size_t i = 0; i < filenames.size(); ++i)
  {
		// Preparation of the output
		cv::Mat matArrayOutputLine1[2];
		cv::Mat matArrayOutputLine2[2];
		cv::Mat matArrayOutputLine3[2];

		//Load the image
    Mat img = imread(filenames[i]);
		if(!img.data) {
			cerr << "Problem loading image!!!" << endl;
			return 0;
		}

		// Output
		matArrayOutputLine1[0] = img;

		// Isolate Circles by color------------------------------------------------

		/* DOC : https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/ */
		// Convert input image to HSV
		cv::Mat hsv_image;
		cv::cvtColor(img, hsv_image, cv::COLOR_BGR2HSV);

		//Threshold the HSV image, keep only the red pixels
		cv::Mat red_hsv_range;
		cv::inRange(hsv_image, cv::Scalar(0, 151, 0), cv::Scalar(010, 255, 255), red_hsv_range);

		// output
		matArrayOutputLine2[0] = red_hsv_range;

		//Threshold the HSV image, keep only the blue pixels
		cv::Mat blue_hsv_range;
		cv::inRange(hsv_image, cv::Scalar(90, 80, 10), cv::Scalar(180, 200, 150), blue_hsv_range);

		// output
		matArrayOutputLine3[0] = blue_hsv_range;

		// Connected component labeling could be another option to look into
		cv::Mat elementD = getStructuringElement( MORPH_RECT,
                     Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                     Point( dilation_size, dilation_size ) );

		// Dialate to get rid of isoled pixels and some noise
		dilate( red_hsv_range, red_hsv_range, elementD);
		dilate( blue_hsv_range, blue_hsv_range, elementD);

		// Erode
		cv::Mat elementE = getStructuringElement( MORPH_RECT,
                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       Point( erosion_size, erosion_size ) );
		cv::erode( red_hsv_range, red_hsv_range, elementE);
		cv::erode( blue_hsv_range, blue_hsv_range, elementE);

		// Blur image
		cv::GaussianBlur( red_hsv_range, red_hsv_range, cv::Size(9, 9), 2, 2);
		cv::GaussianBlur( blue_hsv_range, blue_hsv_range, cv::Size(9, 9), 2, 2);

		// Storage of the centers
		std::vector<Point> center;

		// Find the red circle
		matArrayOutputLine2[1] = FindCircles( red_hsv_range, center);

		// Find blue circle
		matArrayOutputLine3[1] = FindCircles( blue_hsv_range, center);

		std::cout << "Number of centers : " << center.size() << std::endl;

		matArrayOutputLine1[1] = img.clone();
		if( center.size()>=4) {

			std::vector<Point> keypoints = algoFind3Points( center);
			std::cout  << " Key : " << keypoints[0] <<" "<< keypoints[1] << "\n";

			if( keypoints.size()>=3) {
				circle( matArrayOutputLine1[1], keypoints[0], 10, cv::Scalar( 0, 255, 0), -1);
				cv::putText( matArrayOutputLine1[1], "1", keypoints[0], cv::FONT_HERSHEY_PLAIN, 4,  cv::Scalar( 0, 255, 0), 4);
				circle( matArrayOutputLine1[1], keypoints[1], 10, cv::Scalar( 0, 255, 0), -1);
				cv::putText( matArrayOutputLine1[1], "2", keypoints[1], cv::FONT_HERSHEY_PLAIN, 4,  cv::Scalar( 0, 255, 0), 4);
				circle( matArrayOutputLine1[1], keypoints[2], 10, cv::Scalar( 0, 255, 0), -1);
				cv::putText( matArrayOutputLine1[1], "3", keypoints[2], cv::FONT_HERSHEY_PLAIN, 4,  cv::Scalar( 0, 255, 0), 4);
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
