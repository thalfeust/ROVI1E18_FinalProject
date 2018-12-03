#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core.hpp>

#include "FeaturesDetection.h"

#include <iostream>

using namespace std;

int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(argc, argv,
        "{help      |                       | print this message}"
        "{@path     | ../data/marker_corny/  | image path}"
        "{@image    | ../data/marker_corny/marker_corny_01.png | image path}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Load images [Based on the exercise corrections]
    std::string filepath = parser.get<std::string>("@image");
    cv::Mat src = cv::imread(filepath, cv::IMREAD_COLOR);

    // verification
    if ( src.empty())
    {
      std::cout << "Input image not found at '" << filepath << "'\n";
      return 1;
    }

    FeaturesDetection detection;

    std::vector<cv::KeyPoint> keyPoints;
    //detection.extract( src, keyPoints, FeaturesDetection::SURF, true);
    detection.extract( src, keyPoints, FeaturesDetection::SIFT, true);

    while (cv::waitKey() != 27)
    ;
    return 0;
}
