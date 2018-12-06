#include <fstream>
#include <iostream>
#include <rw/rw.hpp>
#include <string>
#include <stdlib.h>

std::vector<rw::math::Transform3D<double>> getTransformMotions( std::string path) {

  // Open the file
  std::ifstream myfile;
  myfile.open (path, std::ios::in);

  // Initialization of the output
  std::vector<rw::math::Transform3D<double>> TransformMotions;

  if (myfile.is_open()) { /* ok, proceed with output */

    std::string line;

    // parser
    while ( getline( myfile, line)) {

      std::vector<std::string> tokens;

      std::istringstream iss(line);
      std::string token;
      while( std::getline(iss, token, '\t')) {
          tokens.push_back( token);
      }

      // Creaion of P and R from T by the syntax X Y Z Roll Pitch Yaw
      rw::math::Vector3D<double> P( atof(tokens[0].c_str()), atof(tokens[1].c_str()), atof(tokens[2].c_str()));
      rw::math::RPY<double> R( atof(tokens[3].c_str()), atof(tokens[4].c_str()), atof(tokens[5].c_str()));

      // Get P and T
      rw::math::Transform3D<double> T( P, R.toRotation3D());
      TransformMotions.push_back(T);
    }

    myfile.close();
  }else {
    RW_THROW("Motions file not found!");
  }

  // Print the results
  for(unsigned int j = 0; j < TransformMotions.size(); ++j) {
    std::cout << "Ts : " << j << " \n" << TransformMotions[j] << std::endl;
  }

  return TransformMotions;
}

rw::math::Vector2D<double> get_du_dv( rw::math::Vector2D<double> UVref, std::vector<rw::math::Transform3D<double>> TransformMotions, int index, rw::kinematics::Frame* cam_frame, rw::kinematics::MovableFrame* marker_frame, rw::kinematics::State state, float f) {

  rw::math::Transform3D<double> wTmarker = TransformMotions[index];

  marker_frame->setTransform( wTmarker, state);

  // Get new Pos of the marker inside Fcam
  rw::math::Transform3D<double> markerTcam = cam_frame->fTf( marker_frame, state);

  rw::math::Vector3D<double> P = markerTcam.P();
  rw::math::Vector2D<double> UV( f*P(0)/P(2), f*P(1)/P(2));
  rw::math::Vector2D<double> dUV( UVref(0)-UV(0), UVref(1)-UV(1));

  std::cout << "UV" << UV << std::endl;

  return dUV;
  //rw::math::Transform3D<double> wTb = device.worldTbase();
  //rw::math::Transform3D<double> wTb = device.baseTMarker();
}

int main() {

  // Paths to workcell, name of robot, path for the motion
  const std::string workcell_path = "../../Workcells/PA10WorkCell/ScenePA10RoVi1.wc.xml";
  const std::string device_name = "PA10";
  const std::string motions_path = "../../Plugin/SamplePluginPA10/motions/MarkerMotionSlow.txt";

  // Load workcell
  rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(workcell_path);

  // Get device and check if it has been loaded correctly
  rw::models::Device::Ptr device = wc->findDevice(device_name);
  if(device == nullptr) {
      RW_THROW("Device " << device_name << " was not found!");
  }

  // Find Camera
  rw::kinematics::Frame* cam_frame = wc->findFrame("Camera");
  if(cam_frame == nullptr) {
      RW_THROW("Camera frame not found!");
  }

  // Find Marker as movable to set the news positions
  rw::kinematics::MovableFrame* marker_frame = dynamic_cast<rw::kinematics::MovableFrame*>(wc->findFrame("Marker"));
  if(cam_frame == nullptr) {
      RW_THROW("Marker frame not found!");
  }

  // Get an initial configuration, q
  rw::kinematics::State state = wc->getDefaultState();
  rw::math::Q q(7,0,-0.65,0,1.80,0,0.42,0);
  device->setQ(q, state);
  float deltaT = 1;
  float f = 823;
  float z = 0.5;

  // Get the motions
  std::vector<rw::math::Transform3D<double>> TransformMotions = getTransformMotions( motions_path);

  std::cout << "\n\n\n";

  // Get Pos of the marker inside Fcam
  rw::math::Transform3D<double> markerTcam = cam_frame->fTf( marker_frame, state);

  rw::math::Vector3D<double> P = markerTcam.P();
  rw::math::Vector2D<double> UVref( f*P(0)/P(2), f*P(1)/P(2));

  std::cout << "UVref" << UVref << std::endl;

  rw::math::Vector2D<double> dUV = get_du_dv( UVref, TransformMotions, 0, cam_frame, marker_frame, state, f);

  std::cout << "dUV" << dUV << std::endl;
}
