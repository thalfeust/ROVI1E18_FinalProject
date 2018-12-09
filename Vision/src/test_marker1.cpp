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

int erosion_size = 5;
int dilation_size = 5;

//std::vector<Point> FindCircles(Mat circles_hsv_image)
//{
	//std::vector<Point> center;
cv::Mat FindCircles( const cv::Mat& circles_hsv_image)
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
		//For the center bit
		Point2f center;
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

		if(compactness < 0.8)
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
			minEnclosingCircle(contours[i], center, radius);
			//Printin the centers
			std::cout << "Circle Center: " << center << std::endl;
		}
	}
	return drawing;
}

int main(int argc, char* argv[])
{

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
		cv::inRange(hsv_image, cv::Scalar(0, 151, 0), cv::Scalar(023, 255, 255), red_hsv_range);

		// output
		matArrayOutputLine2[0] = red_hsv_range;

		//Threshold the HSV image, keep only the blue pixels
		cv::Mat blue_hsv_range;
		cv::inRange(hsv_image, cv::Scalar(50, 100, 10), cv::Scalar(180, 200, 150), blue_hsv_range);

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

		// Find the red circle
		matArrayOutputLine2[1] = FindCircles( red_hsv_range);

		// Find blue circle
		matArrayOutputLine3[1] = FindCircles( blue_hsv_range);

		matArrayOutputLine1[1] = img;

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

		while (waitKey() != 27)
			; // (do nothing)
	}
	std::cout << "END" << std::endl;
}
