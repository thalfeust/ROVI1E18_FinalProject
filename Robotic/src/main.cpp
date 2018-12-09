#include <fstream>
#include <iostream>
#include <rw/rw.hpp>
#include <string>
#include <stdlib.h>

#include "Tracking.h"

int main() {

  Tracking tracker;

  // Paths to workcell, name of robot, path for the motion
  const std::string workcell_path = "../../Workcells/PA10WorkCell/ScenePA10RoVi1.wc.xml";
  const std::string device_name = "PA10";
  //const std::string motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionSlow.txt";
  //const std::string motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionMedium.txt";
  const std::string motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionFast.txt";

  // Load workcell
  rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(workcell_path);

  // Get device and check if it has been loaded correctly
  tracker.device = wc->findDevice(device_name);
  if(tracker.device == nullptr) {
      RW_THROW("Device " << device_name << " was not found!");
  }

  // Find Camera
  tracker.cam_frame = wc->findFrame("Camera");
  if(tracker.cam_frame == nullptr) {
      RW_THROW("Camera frame not found!");
  }

  // Find Marker as movable to set the news positions
  tracker.marker_frame = dynamic_cast<rw::kinematics::MovableFrame*>(wc->findFrame("Marker"));
  if(tracker.marker_frame == nullptr) {
      RW_THROW("Marker frame not found!");
  }

  // Get an initial configuration, q
  tracker.state = wc->getDefaultState();
  rw::math::Q qq(7,0,-0.65,0,1.80,0,0.42,0);
  tracker.q = qq;
  tracker.qInit = qq;
  tracker.device->setQ( tracker.q, tracker.state);
  tracker.deltaT = 1;
  tracker.f = 823;
  tracker.z = 0.5;

  tracker.getTransformMotions( motions_path);

  // Get the motions
  /*std::vector<rw::math::Transform3D<double>> TransformMotions = getTransformMotions( motions_path);*/

  std::cout << "\n\n\n";

  // Get Pos of the marker inside Fcam to compute UVref
  rw::math::Transform3D<double> markerTcam = tracker.cam_frame->fTf( tracker.marker_frame, tracker.state);
  rw::math::Vector3D<double> P = markerTcam.P();
  rw::math::Vector2D<double> uvref( tracker.f*P(0)/P(2), tracker.f*P(1)/P(2));
  tracker.UVref = uvref;

  std::cout << "UVref : " << tracker.UVref << std::endl;

  //tracker.superLoop(false);
  tracker.testError_from_deltaT();

  std::cout << "END" << std::endl;
  return 0;

}
