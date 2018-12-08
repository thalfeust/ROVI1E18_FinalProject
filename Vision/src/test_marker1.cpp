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





//std::vector<Point> FindCircles(Mat circles_hue_image)
//{
	//std::vector<Point> center;

void FindCircles(const cv::Mat& circles_hue_image)
{
	//Find countours--------------------------------------------------------------------
	//Alternative to HoughCircles, should work better on hard sequences.
	//Using code from Lab week 9 (guest lecture)

	//Invert the colors so that the contour will work
	cv::bitwise_not ( circles_hue_image, circles_hue_image );
	//imshow( "Inverse", circles_hue_image );

	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;
	/// Find contours
	findContours( circles_hue_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	// Start random number generator with a known seed, for color labeling
	RNG rng(12345);

	/// Draw contours for every oject in the image
	Mat drawing = Mat::zeros( circles_hue_image.size(), CV_8UC3 );
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
			//Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			//drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point() );
		}
		else
		{
			// Circle found, draw contour
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			drawContours( drawing, contours, i, color, 4, 8, hierarchy, 0, Point() );

			//Calculate the centers
			//Getting this to work was waaaay harder than it should have been.
			//https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/bounding_rects_circles/bounding_rects_circles.html
			minEnclosingCircle(contours[i], center, radius);
			//Printin the centers
			std::cout << "Circle Center: " << center << std::endl;

			imshow( "Contours", drawing );
			//Center_Red[i] = center;

		}



	}


	while (waitKey() != 27)
        ; // (do nothing)


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

	//Load multiple images
	/* DOC : http://answers.opencv.org/question/58078/reading-a-sequence-of-files-within-a-folder/ */
	// notice here that we are using the Opencv's embedded "String" class
	vector<String> filenames;
    //String folder = "../data/marker_color/*.png"; // again we are using the Opencv's embedded "String" class
    glob(folder, filenames); // new function that does the job ;-)

    for(size_t i = 0; i < filenames.size(); ++i)
    {
		//Load the image
        Mat img = imread(filenames[i]);
		cv::imshow("Original Image", img);
		//Convert to grey
		//cv::Mat gray;
		//cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

        if(!img.data)
            cerr << "Problem loading image!!!" << endl;

        /* do whatever you want with your images here */

// Isolate Circles by color----------------------------------------------------------------------

		//https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
		// Convert input image to HSV
		cv::Mat hsv_image;
		cv::cvtColor(img, hsv_image, cv::COLOR_BGR2HSV);

		//Threshold the HSV image, keep only the red pixels
		cv::Mat red_hue_range;
		cv::inRange(hsv_image, cv::Scalar(0, 151, 0), cv::Scalar(023, 255, 255), red_hue_range);
		//cv::imshow("HSV Image", red_hue_range);

		//Threshold the HSV image, keep only the blue pixels
		cv::Mat blue_hue_range;
		cv::inRange(hsv_image, cv::Scalar(50, 100, 10), cv::Scalar(180, 200, 150), blue_hue_range);

		// Combine the above two images
		cv::Mat circles_hue_image;
		cv::addWeighted(red_hue_range, 1.0, blue_hue_range, 1.0, 0.0, circles_hue_image);

		//connected component labeling could be another option to look into
		//Dialate to get rid of isoled pixels and some noise
		cv::Mat elementD = getStructuringElement( MORPH_RECT,
                     Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                     Point( dilation_size, dilation_size ) );
		dilate( circles_hue_image, circles_hue_image, elementD );
		//cv::imshow("Dialated", circles_hue_image);

		//Erode
		cv::Mat elementE = getStructuringElement( MORPH_RECT,
                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       Point( erosion_size, erosion_size ) );
		cv::erode( circles_hue_image, circles_hue_image, elementE );
		//cv::imshow("Eroded", circles_hue_image);

		//Blur image
		cv::GaussianBlur(circles_hue_image, circles_hue_image, cv::Size(9, 9), 2, 2);
		//cv::imshow("Isolated circles blurred", circles_hue_image);


//Find the red circle----------------------------------------------------------------------
	cv::imshow("HSV Red Image", red_hue_range);
	FindCircles(red_hue_range);
	//cv::Mat red_point = FindCircles(red_hue_range);
	//std::vector<Point> Center_Red;
	//Center_Red = FindCircles(red_hue_range);
	//std::cout << "Red Circle Center: " << Center_Red << std::endl;
//Find blue circle ----------------------------------------------------------------------
	cv::imshow("HSV Blue Image", blue_hue_range);
	FindCircles(blue_hue_range);
	//FindCircles(circles_hue_image);


	//End----------------
    }





}
