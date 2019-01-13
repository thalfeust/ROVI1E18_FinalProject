#include <fstream>
#include <iostream>
#include <rw/rw.hpp>
#include <string>
#include <stdlib.h>

#include "Tracking.hpp"

using namespace std;

int main( int argc, char *argv[]) {

        std::string motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionSlow.txt";
        int pointNumber = 1;
        int printOutputMode = 0;
        bool velocity = false;
        if( argc>1) {
                if( strcmp( argv[1], "Slow")==0) {
                        motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionSlow.txt";
                }else if( strcmp( argv[1], "Medium")==0) {
                        motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionMedium.txt";
                }else if( strcmp( argv[1], "Fast")==0) {
                        motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionFast.txt";
                }

                if( argc>2) {
                        if( strcmp( argv[2], "1")==0) {
                                pointNumber = 1;
                        }else if( strcmp( argv[2], "3")==0) {
                                pointNumber = 3;
                        }

                        if( argc>3) {
                                if( strcmp( argv[3], "Q")==0) {
                                        printOutputMode = 1;
                                }else if( strcmp( argv[3], "tool")==0) {
                                        printOutputMode = 2;
                                }else if( strcmp( argv[3], "error")==0) {
                                        printOutputMode = 3;
                                }

                                if( argc>4) {
                                        if( strcmp( argv[4], "velocity")==0) {
                                                std::cout << "Velocity : ON\n";
                                                velocity = true;
                                        }
                                }
                        }
                }
        }

        std::cout << "File path : " << motions_path << "\nPress a key + enter to continu ..." << std::endl;
        char tamp;
        std::cin >> tamp;

        Tracking tracker;

        // Paths to workcell, name of robot, path for the motion
        const std::string workcell_path = "../../Workcells/PA10WorkCell/ScenePA10RoVi1.wc.xml";
        const std::string device_name = "PA10";

        // Load workcell
        rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(workcell_path);

        // Update the robot

        // Get device and check if it has been loaded correctly
        tracker.set( wc, pointNumber);
        tracker.device->setQ( tracker.qInit, tracker.state);

        tracker.getTransformMotions( motions_path);

        // Get the motions
        /*std::vector<rw::math::Transform3D<double>> TransformMotions = getTransformMotions( motions_path);*/

        std::cout << "\n\n\n";

        if( pointNumber==1) {
                std::cout << "UVref : " << tracker.UVref << "\nPress a key + enter to continu ..." << std::endl;
                std::cin >> tamp;
        }else {
                std::cout << "UVref x3 : " << tracker.UVref1 << " " << tracker.UVref2 << " " << tracker.UVref3 << "\nPress a key + enter to continu ..." << std::endl;
                std::cin >> tamp;
        }

        if( printOutputMode==3) {
                tracker.testError_from_deltaT( pointNumber);
        }else {
                tracker.superLoop( velocity, pointNumber, printOutputMode);
        }

        std::cout << "END" << std::endl;
        return 0;
}
