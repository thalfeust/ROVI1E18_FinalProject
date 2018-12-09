/*
RoVi1
Planar matching

<<exercise-description>>
You must judge the quality of each matched descriptor pair, and then use the best
to find the perspective transformation between the two input images
`book_cover.jpg` and `book_scene.jpg`

1. Filter out bad matches.
2. Find the homography between the books in the two images.
3. Draw a box around the the book in the scene image.
4. Also try detecting the book in the `book_scene2.jpg` image.

For help and inspiration, see

- [Features2D + Homography to find a known object](http://docs.opencv.org/master/d7/dff/tutorial_feature_homography.html)
- [Feature Matching](http://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html)
<<->>



Todo
region of inetrest in the corner
manual fix it

do an analyse, not match

cause it is interesting point, few variences 





choose bettween sift and surf
google


flanbassed - goofle why it is better








Version: 6e92b7d
*/

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <iostream>

// Extract the coordinates of the keypoints
std::vector<cv::Point2f> keypoint_coordinates(const std::vector<cv::KeyPoint>& keypoints)
{
    std::vector<cv::Point2f> coords;
    coords.reserve(keypoints.size());

    for (const auto& kp : keypoints) {
        coords.push_back(kp.pt);
    }

    return coords;
}

// Draws bounding box defined by the points in 'bb'
void draw_bb(cv::Mat& img,
             const std::vector<cv::Point2f>& bb,
             const cv::Point2f& offset = cv::Point2f(0, 0))
{
    for (size_t i = 0; i < bb.size() - 1; i++) {
        cv::line(img, bb[i] + offset, bb[i + 1] + offset, cv::Scalar(0, 0, 255), 3);
    }

    cv::line(img, bb[bb.size() - 1] + offset, bb[0] + offset, cv::Scalar(0, 0, 255), 3);
}





int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(argc, argv,
        "{help     |                  | print this message}"
        "{features | sift             | feature type}"
        "{matcher  | bruteforce       | matcher type}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Load images
    //std::string filepath1 = parser.get<std::string>("@image1");
    //std::string filepath2 = parser.get<std::string>("@image2");
    //cv::Mat img1 = cv::imread(filepath1);
    //cv::Mat img2 = cv::imread(filepath2);
	cv::Mat img1 = cv::imread("../data/marker_4.png", 1);
	cv::Mat img2 = cv::imread("../data/marker_corny/marker_corny_01.png", 1);

    if (img1.empty()) {
        //std::cout << "Input image 1 not found at '" << filepath1 << "'\n";
        return 1;
    }

    if (img2.empty()) {
        //std::cout << "Input image 2 not found at '" << filepath2 << "'\n";
        return 1;
    }

    // Construct detector
    cv::Ptr<cv::Feature2D> detector;
    std::string features_type = parser.get<std::string>("features");

    if (features_type == "sift") {
        detector = cv::xfeatures2d::SIFT::create();
    } else if (features_type == "surf") {
        detector = cv::xfeatures2d::SURF::create();
    } else {
        std::cout << "Unknown feature type '" << features_type << "'\n";
        return 1;
    }

    std::cout << "Feature type: " << features_type << std::endl;

    // Detect keypoints and compute descriptors
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    detector->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

    // Construct matcher
    cv::Ptr<cv::DescriptorMatcher> matcher;
    std::string matcher_type = parser.get<std::string>("matcher");
	

    if (matcher_type == "bruteforce") {
        // Creating BFMatcher with the 'crossCheck' param set to true provides
        // an alternative to Lowe's ratio criterion.
        matcher = cv::BFMatcher::create();
    } else if (matcher_type == "flannbased") {
        matcher = cv::FlannBasedMatcher::create();
    } else {
        std::cout << "Unknown matcher type '" << matcher_type << "'\n";
        return 1;
    }

    std::cout << "Matcher type: " << matcher_type << std::endl;

    // Find 2 nearest correspondences for each descriptor
    std::vector<std::vector<cv::DMatch>> initial_matches;
    matcher->knnMatch(descriptors1, descriptors2, initial_matches, 2);
    std::cout << "Number of initial matches: " << initial_matches.size() << std::endl;

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
            matched1.push_back(keypoints1[match[0].queryIdx]);
            matched2.push_back(keypoints2[match[0].trainIdx]);
            idx++;
        }
    }

    // We only use the good matches
    auto num_matches = matches.size();
    std::cout << "Number of good matches: " << num_matches << std::endl;

    if (num_matches < 4) {
        std::cout << "Too few matches!" << std::endl;
        return 0;
    }

    // 2. Calculate perspective transformation
    std::vector<char> inlier_mask;
    cv::Mat H = cv::findHomography(keypoint_coordinates(matched1),
                                   keypoint_coordinates(matched2),
                                   cv::LMEDS,
                                   0,
                                   inlier_mask);

    if (H.empty()) {
        std::cout << "H matrix could not be estimated!" << std::endl;
        return 0;
    }

    int num_inliers = cv::countNonZero(inlier_mask);
    std::cout << "Number of inliers: " << num_inliers << std::endl;

    // Ratio of inliers vs original number of keypoint matches considered
    double ratio = double(num_inliers) / num_matches;
    std::cout << "Inlier ratio: " << ratio << std::endl;

    // Bounding box of original object (the book takes up the whole image area)
    std::vector<cv::Point2f> bb1{
        cv::Point2f(0, 0),
        cv::Point2f(img1.cols, 0),
        cv::Point2f(img1.cols, img1.rows),
        cv::Point2f(0, img1.rows)
    };

    // 3. Transform the bounding box points using the transformation matrix H
    std::vector<cv::Point2f> bb2;
    cv::perspectiveTransform(bb1, bb2, H);

    // Visualize the result
    cv::Mat img_out;
    cv::drawMatches(img1, matched1,
                    img2, matched2,
                    matches, img_out,
                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                    inlier_mask); // Draw only inliers
    draw_bb(img_out, bb1);
    draw_bb(img_out, bb2, cv::Point2f(img1.cols, 0)); // Offset by img1.cols in the x direction
    cv::imshow("Matches", img_out);

    while (cv::waitKey() != 27)
        ;

    return 0;
}

	
	
	
	
	
	
	
	