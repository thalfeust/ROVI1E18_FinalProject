#include <fstream>
#include <iostream>
#include <rw/rw.hpp>
#include <string>
#include <stdlib.h>

#include "Tracking.hpp"

using namespace std;

int main( int argc, char *argv[]) {

        std::string motions_path = "";
        if( argc>1) {
                if( strcmp( argv[1], "Slow")==0) {
                        motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionSlow.txt";
                }else if( strcmp( argv[1], "Medium")==0) {
                        motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionMedium.txt";
                }else if( strcmp( argv[1], "Fast")==0) {
                        motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionFast.txt";
                }else {
                        motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionSlow.txt";
                }
        }else {
                motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionSlow.txt";
        }

        int pointNumber = 3;

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

        tracker.superLoop( false, pointNumber);
        //tracker.testError_from_deltaT();

        std::cout << "END" << std::endl;
        return 0;

}
