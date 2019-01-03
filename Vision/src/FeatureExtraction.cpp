#include "FeatureExtraction.hpp"

ColorSegmentation::ColorSegmentation( double threshold, double compactness, int dilation, int erosion, cv::Size gaussian, cv::Scalar red_lB, cv::Scalar red_uB, cv::Scalar blue_lB, cv::Scalar blue_uB) {

        area_threshold = threshold;
        compactness_limit = compactness;
        dilation_size = dilation;
        erosion_size = erosion;
        gaussianSize = gaussian;

        red_lowerBoundary = red_lB;
        red_upperBoundary = red_uB;
        blue_lowerBoundary = blue_lB;
        blue_upperBoundary = blue_uB;
}

cv::Mat ColorSegmentation::FindCircles( const cv::Mat& circles_hsv_image, std::vector<cv::Point> &center, bool print)
{

        cv::Mat drawing = cv::Mat::zeros( circles_hsv_image.size(), CV_8UC3 );

        //Find countours--------------------------------------------------------
        //Alternative to HoughCircles, should work better on hard sequences.
        //Using code from Lab week 9 (guest lecture)

        //Invert the colors so that the contour will work
        cv::bitwise_not ( circles_hsv_image, circles_hsv_image );

        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        /// Find contours
        cv::findContours( circles_hsv_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        /// Draw contours for every oject in the image
        for( unsigned int i = 0; i< contours.size(); i++ )
        {
                // Calculate perimeter length
                double perimeter = cv::arcLength(contours[i], 1);
                double area = cv::contourArea(contours[i], true);

                //This bit is a "trick" to get rid of small acrifacts, no the ideal way to do it.
                if (area < area_threshold)
                { perimeter = 10000;
                  area = 0;}

                double compactness = (4 * M_PI * area) / (perimeter * perimeter);

                if( print) {
                        std::printf("perimeter: %8.3f  area: %8.3f   compactness: %8.3f\n", perimeter, area, compactness);
                }

                // Comparison of the compactness with the limit given by our tests
                if(compactness < compactness_limit)
                {
                        // Draw contour for things smaller than
                        cv::Scalar color = cv::Scalar( 0, 0, 255);
                        cv::drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point());
                }
                else
                {
                        // Circle found, draw contour
                        cv::Scalar color = cv::Scalar( 0, 255, 0);
                        cv::drawContours( drawing, contours, i, color, 4, 8, hierarchy, 0, cv::Point());

                        //Calculate the centers
                        //Getting this to work was waaaay harder than it should have been.[DOC-1]
                        //minEnclosingCircle(contours[i], center, radius);
                        //Printin the centers
                        //std::cout << "Circle Center: " << center << std::endl;

                        // Get the moments of the contour
                        cv::Moments mu = cv::moments( contours[i], false );

                        ///  Get the mass centers:
                        cv::Point mc = cv::Point2f( mu.m10/mu.m00, mu.m01/mu.m00 );
                        std::cout << "Circle Center from Moments: " << mc << std::endl;
                        center.push_back( mc);
                }
        }
        return drawing;
}

cv::Mat ColorSegmentation::dilate( const cv::Mat& src) {

        cv::Mat toReturn;

        // Connected component labeling could be another option to look into
        cv::Mat elementD = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ), cv::Point( dilation_size, dilation_size));

        // Dilate to get rid of isoled pixels and some noise
        cv::dilate( src, toReturn, elementD);

        return toReturn;
}

cv::Mat ColorSegmentation::erode( const cv::Mat& src) {

        cv::Mat toReturn;

        cv::Mat elementE = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );

        cv::erode( src, toReturn, elementE);

        return toReturn;
}

cv::Mat ColorSegmentation::gaussianBlur( const cv::Mat& src) {

        cv::Mat toReturn;

        cv::GaussianBlur( src, toReturn, gaussianSize, 2, 2);

        return toReturn;
}

cv::Mat ColorSegmentation::segmentation( const cv::Mat& src, std::string mode) {

        if( mode.compare("blue")!=0 && mode.compare("red")!=0) {
                throw std::exception();
        }

        // Convert input image to HSV [DOC-3]
        cv::Mat hsv_image;
        cv::cvtColor(src, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat toReturn;

        if( mode.compare("red")==0) {
                //Threshold the HSV image, keep only the red pixels
                cv::inRange(hsv_image, red_lowerBoundary, red_upperBoundary, toReturn);
        }else {
                //Threshold the HSV image, keep only the blue pixels
                cv::inRange(hsv_image, blue_lowerBoundary, blue_upperBoundary, toReturn);
        }

        return toReturn;
}

cv::Mat ColorSegmentation::drawResult( const cv::Mat& src, std::vector<cv::Point> center) {

        cv::Mat toReturn = src.clone();

        circle( toReturn, center[0], 10, cv::Scalar( 0, 255, 0), -1);
        cv::putText( toReturn, "1", center[0], cv::FONT_HERSHEY_PLAIN, 4,  cv::Scalar( 0, 255, 0), 4);
        circle( toReturn, center[1], 10, cv::Scalar( 0, 255, 0), -1);
        cv::putText( toReturn, "2", center[1], cv::FONT_HERSHEY_PLAIN, 4,  cv::Scalar( 0, 255, 0), 4);
        circle( toReturn, center[2], 10, cv::Scalar( 0, 255, 0), -1);
        cv::putText( toReturn, "3", center[2], cv::FONT_HERSHEY_PLAIN, 4,  cv::Scalar( 0, 255, 0), 4);

        return toReturn;
}


std::vector<cv::Point> ColorSegmentation::algoFind3Points( const std::vector<cv::Point> center) {

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

        std::vector<cv::Point> b;
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

PointsFeatures::PointsFeatures( int detector, cv::Mat img) {

        if( detector!=DETECTOR_SURF && detector!=DETECTOR_SIFT) {
                throw std::exception();
        }
        detectorMode = detector;

        bb_originalObject = std::vector<cv::Point2f>{
                cv::Point2f(0, 0),
                cv::Point2f(img.cols, 0),
                cv::Point2f(img.cols, img.rows),
                cv::Point2f(0, img.rows)
        };
}

// Extract the coordinates of the keypoints
std::vector<cv::Point2f> PointsFeatures::keypoint_coordinates(const std::vector<cv::KeyPoint>& keypoints)
{
        std::vector<cv::Point2f> coords;
        coords.reserve(keypoints.size());

        for (const auto& kp : keypoints) {
                coords.push_back(kp.pt);
        }

        return coords;
}

std::vector<std::vector<cv::DMatch> > PointsFeatures::detect_match( const cv::Mat marker, const cv::Mat src, std::vector<cv::KeyPoint>* keypointsMarker, std::vector<cv::KeyPoint>* keypointsSrc) {

        // Construct detector
        cv::Ptr<cv::Feature2D> detector;

        //Sift or surf [DOC-4] & [DOC-5]
        if( detectorMode == DETECTOR_SURF) {
                detector = cv::xfeatures2d::SURF::create();
        }else if( detectorMode==DETECTOR_SIFT) {
                detector = cv::xfeatures2d::SIFT::create();
        }

        // Detect keypoints and compute descriptors
        cv::Mat descriptors1, descriptors2;
        detector->detectAndCompute( marker, cv::noArray(), *keypointsMarker, descriptors1);
        detector->detectAndCompute( src, cv::noArray(), *keypointsSrc, descriptors2);

        // Construct matcher
        cv::Ptr<cv::DescriptorMatcher> matcher;

        //Creating BFMatcher Flann Based
        matcher = cv::FlannBasedMatcher::create();

        // Find 2 nearest correspondences for each descriptor
        std::vector<std::vector<cv::DMatch> > initial_matches;
        matcher->knnMatch(descriptors1, descriptors2, initial_matches, 2);

        return initial_matches;
}

std::vector<cv::DMatch> PointsFeatures::matchQuality( std::vector<std::vector<cv::DMatch> > initial_matches, std::vector<cv::KeyPoint>* keypointsMarker, std::vector<cv::KeyPoint>* keypointsSrc) {

        // 1. Judge match quality based on Lowe's ratio criterion (from SIFT paper):
        // The ratio of the distance between the best and second-best match must be
        // less than 0.8
        std::vector<cv::DMatch> matches;
        std::vector<cv::KeyPoint> matched1;
        std::vector<cv::KeyPoint> matched2;
        int idx = 0;

        for (const auto& match : initial_matches) {
                if (match[0].distance < 0.8f * match[1].distance) {
                        matches.push_back(cv::DMatch(idx, idx, match[0].distance));
                        matched2.push_back((*keypointsSrc)[match[0].trainIdx]);
                        matched1.push_back((*keypointsMarker)[match[0].queryIdx]);
                        idx++;
                }
        }

        *keypointsMarker = matched1;
        *keypointsSrc = matched2;
        return matches;
}

std::vector<cv::Point> PointsFeatures::extractPoints( std::vector<cv::KeyPoint> keypointsMarker, std::vector<cv::KeyPoint> keypointsSrc, std::vector<char>* inlier_mask, std::vector<cv::Point2f>* bb, std::size_t num_matches, bool print) {

        // to return
        std::vector<cv::Point> trackPoints;

        // 2. Calculate perspective transformation
        cv::Mat H = cv::findHomography( keypoint_coordinates(keypointsMarker), keypoint_coordinates(keypointsSrc), cv::LMEDS, 0, *inlier_mask);

        if (H.empty()) {
                return trackPoints;
        }

        if( print) {
                // Print stats
                int num_inliers = cv::countNonZero(*inlier_mask);
                std::cout << "Number of inliers: " << num_inliers << std::endl;

                // Ratio of inliers vs original number of keypoint matches considered
                double ratio = double(num_inliers) / num_matches;
                std::cout << "Inlier ratio: " << ratio << std::endl;
        }

        // 3. Transform the bounding box points using the transformation matrix H
        cv::perspectiveTransform(bb_originalObject, (*bb), H);

        trackPoints.push_back((*bb)[0]);
        trackPoints.push_back((*bb)[1]);
        trackPoints.push_back((*bb)[2]);

        if( print) {
                std::cout << (*bb) <<"\n";
        }

        return trackPoints;
}

void PointsFeatures::drawComparaison( cv::Mat* img_out, const cv::Mat imgMarker, const cv::Mat imgSrc, std::vector<cv::KeyPoint> keypointsMarker, std::vector<cv::KeyPoint> keypointsSrc, std::vector<cv::DMatch> matches, std::vector<cv::Point> trackPoints, std::vector<char> inlier_mask, std::vector<cv::Point2f> bb) {

        cv::drawMatches(imgMarker, keypointsMarker,
                        imgSrc, keypointsSrc,
                        matches, (*img_out),
                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                        inlier_mask); // Draw only inliers
        draw_bb((*img_out), bb_originalObject, cv::Point2f(imgMarker.cols, 0));
        draw_bb((*img_out), bb, cv::Point2f(imgMarker.cols, 0)); // Offset by imgMarker.cols in the x direction

        if( trackPoints.size()>=3) {
                circle((*img_out), cv::Point (trackPoints[0].x + imgMarker.cols,trackPoints[0].y), 10, cv::Scalar( 0, 255, 0), -1);
                cv::putText((*img_out), "1", cv::Point (trackPoints[0].x + imgMarker.cols,trackPoints[0].y), cv::FONT_HERSHEY_PLAIN, 4,  cv::Scalar( 0, 255, 0), 4);
                circle((*img_out), cv::Point (trackPoints[1].x + imgMarker.cols,trackPoints[1].y), 10, cv::Scalar( 0, 255, 0), -1);
                cv::putText((*img_out), "2", cv::Point (trackPoints[1].x + imgMarker.cols,trackPoints[1].y), cv::FONT_HERSHEY_PLAIN, 4,  cv::Scalar( 0, 255, 0), 4);
                circle((*img_out), cv::Point (trackPoints[2].x + imgMarker.cols,trackPoints[2].y), 10, cv::Scalar( 0, 255, 0), -1);
                cv::putText((*img_out), "3", cv::Point (trackPoints[2].x + imgMarker.cols,trackPoints[2].y), cv::FONT_HERSHEY_PLAIN, 4,  cv::Scalar( 0, 255, 0), 4);
        }

}

// Draws bounding box defined by the points in 'bb'
void PointsFeatures::draw_bb(cv::Mat& img,
                             const std::vector<cv::Point2f>& bb,
                             const cv::Point2f& offset = cv::Point2f(0, 0))
{
        for (size_t i = 0; i < bb.size() - 1; i++) {
                cv::line(img, bb[i] + offset, bb[i + 1] + offset, cv::Scalar(0, 0, 255), 3);
        }

        cv::line(img, bb[bb.size() - 1] + offset, bb[0] + offset, cv::Scalar(0, 0, 255), 3);
}

std::vector<cv::String> FeaturesDections::loadFiles( cv::String folderName) {

        //Load multiple images [DOC-2]
        // notice here that we are using the Opencv's embedded "String" class
        std::vector<cv::String> filenames;
        cv::glob( folderName, filenames);

        return filenames;
}

// [DOC-1] :  https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/bounding_rects_circles/bounding_rects_circles.html
// [DOC-2] : http://answers.opencv.org/question/58078/reading-a-sequence-of-files-within-a-folder/
// [DOC-3] : https://docs.opencv.org/trunk/d2/de8/group__core__array.html#ga48af0ab51e36436c5d04340e036ce981
// [DOC-4] : https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_sift_intro/py_sift_intro.html
// [DOC-5] : https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_surf_intro/py_surf_intro.html
