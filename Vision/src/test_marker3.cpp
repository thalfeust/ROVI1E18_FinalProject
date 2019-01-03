/*
   code from:RoVi1 Planar matching

   1. Filter out bad matches.
   2. Find the homography between the books in the two images.
   3. Draw a box around the the book in the scene image.
   4. Also try detecting the book in the `book_scene2.jpg` image.

   For help and inspiration, see

   - [Features2D + Homography to find a known object](http://docs.opencv.org/master/d7/dff/tutorial_feature_homography.html)
   - [Feature Matching](http://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html)
   <<->>


   Todo

   do an analyse, not match
   cause it is interesting point, few variences

   -detect the corners

   Done
   -Choose bettween sift and surf
   going with sift as it is more resistant
   -flanbassed - why it is better
   Same performance as bruteforce but faster
   -Region of inetrest in the corner
   Manually define it


   Version: I have no idea at this point
 */

#include "FeatureExtraction.hpp"

// set the parameters
#define DETECTOR DETECTOR_SIFT
//#define DETECTOR DETECTOR_SURF

using namespace cv;
using namespace cv::xfeatures2d;

int main(int argc, char* argv[])
{
        //For performance eval
        int64 t0 = cv::getTickCount();

        cv::CommandLineParser parser(argc, argv,
                                     "{help     |                      | print this message}"
                                     "{@image1  | ../data/marker_4.png | image1 path}"
                                     "{@path    | ../data/marker_corny/| path of sources}"
                                     );

        if (parser.has("help")) {
                parser.printMessage();
                return 0;
        }

        // Load images
        std::string filepath1 = parser.get<std::string>("@image1");
        cv::Mat img1 = cv::imread(filepath1);

        //Throw error if image is empty or does not load.
        if (img1.empty()) {
                std::cout << "Input image not found Image 1\n";
                return 1;
        }

        FeaturesDections features;

        // Get the path of the folder
        std::string folder = parser.get<std::string>("@path");
        folder += "*.png";

        cv::namedWindow(folder, cv::WINDOW_NORMAL);

        // set the class
        PointsFeatures extraction_PF( DETECTOR, img1);

        // load the names of the files
        std::vector<cv::String> fileNames = features.loadFiles(folder);


        for(size_t i = 0; i < fileNames.size(); ++i)
        {

                Mat img2 = imread(fileNames[i]);

                std::vector<cv::KeyPoint> keypointsMarker, keypointsSrc;
                // Find 2 nearest correspondences for each descriptor
                std::vector<std::vector<cv::DMatch> > initial_matches = extraction_PF.detect_match( img1, img2, &keypointsMarker, &keypointsSrc);

                std::cout << "Number of initial matches: " << initial_matches.size() << std::endl;

                // 1. Judge match quality based on Lowe's ratio criterion
                std::vector<cv::DMatch> matches = extraction_PF.matchQuality( initial_matches, &keypointsMarker, &keypointsSrc);

                // We only use the good matches
                std::size_t num_matches = matches.size();
                std::cout << "Number of good matches: " << num_matches << std::endl;

                if (num_matches < 4) {
                        std::cout << "Too few matches!" << std::endl;
                        while (waitKey() != 27)
                                ; // (do nothing)
                        break;
                }

                std::vector<char> inlier_mask;
                // boundaryBox
                std::vector<cv::Point2f> bb;
                std::vector<Point> trackPoints = extraction_PF.extractPoints( keypointsMarker, keypointsSrc, &inlier_mask, &bb, num_matches, true);

                if( trackPoints.empty()) {
                        std::cout << "H matrix could not be estimated!" << std::endl;
                        while (waitKey() != 27)
                                ;         // (do nothing)
                        break;
                }

                // Visualize the result
                cv::Mat img_out;

                extraction_PF.drawComparaison( &img_out, img1, img2, keypointsMarker, keypointsSrc, matches, trackPoints, inlier_mask, bb);

                cv::imshow( folder, img_out);

                while (waitKey() != 27)
                        ; // (do nothing)
        }



//ROI--------------------------------------------
        //Select and cut the region of interest
        //http://opencv-help.blogspot.com/2013/02/how-to-extract-subimage-from-image-in.html
        cv::Mat subImage(img1, cv::Rect(0, 0, 100, 100));
        cv::imshow("ROI", subImage);

// Detect keypoints on ROI
        //Based on https://docs.opencv.org/3.0-beta/doc/tutorials/features2d/feature_detection/feature_detection.html
        int minHessian = 400;

        Ptr<SURF> detectors = SURF::create( minHessian );
        std::vector<KeyPoint> keypoints_1;
        detectors->detect( subImage, keypoints_1 );

        //Draw keypoints
        Mat img_keypoints_1;
        drawKeypoints( subImage, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

        //Display detected (drawn) keypoints
        imshow("Keypoints 1", img_keypoints_1 );

        //For performance eval
        int64 t1 = cv::getTickCount();
        double run_time = (t1-t0)/cv::getTickFrequency();
        std::cout << "Time to run" << run_time << std::endl;


        while (cv::waitKey() != 27)
                ;

        return 0;
}
