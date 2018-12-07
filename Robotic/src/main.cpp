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

  std::cout << "Motions \n" << wTmarker << std::endl;

  marker_frame->setTransform( wTmarker, state);

  // Get new Pos of the marker inside Fcam
  rw::math::Transform3D<double> markerTcam = cam_frame->fTf( marker_frame, state);

  rw::math::Vector3D<double> P = markerTcam.P();
  rw::math::Vector2D<double> UV( f*P(0)/P(2), f*P(1)/P(2));
  rw::math::Vector2D<double> dUV( UVref(0)-UV(0), UVref(1)-UV(1));

  std::cout << "UV" << UV << std::endl;

  return dUV;
}

rw::math::Jacobian get_Jimage( float f, rw::math::Vector2D<double> UV, float z) {
  rw::math::Jacobian Jimage(2);
  Jimage(0,0) = -f/z;
  Jimage(1,0) = 0;
  Jimage(2,0) = UV[0]/z;
  Jimage(3,0) = UV[0]*UV[1]/f;
  Jimage(4,0) = -1 * ((f*f+UV[0]*UV[0])/f);
  Jimage(5,0) = UV[1];
  Jimage(0,1) = 0;
  Jimage(1,1) = -f/z;
  Jimage(2,1) = UV[1]/z;
  Jimage(3,1) = ((f*f+UV[1]*UV[1])/f);
  Jimage(4,1) = -(UV[0]*UV[1])/f;
  Jimage(5,1) = -UV[0];

  return Jimage;
}

rw::math::Jacobian get_Sq( rw::models::Device::Ptr device, rw::kinematics::State state, rw::kinematics::Frame* cam_frame) {

  rw::math::Jacobian Sq(6);
  rw::math::Rotation3D<> R = device->baseTframe(cam_frame, state).R();

  Sq(0,0) = R(0,0);
  Sq(1,0) = R(0,1);
  Sq(2,0) = R(0,2);
  Sq(0,1) = R(1,0);
  Sq(1,1) = R(1,1);
  Sq(2,1) = R(1,2);
  Sq(0,2) = R(2,0);
  Sq(1,2) = R(2,1);
  Sq(2,2) = R(2,2);

  Sq(3,0) = 0;
  Sq(4,0) = 0;
  Sq(5,0) = 0;
  Sq(3,1) = 0;
  Sq(4,1) = 0;
  Sq(5,1) = 0;
  Sq(3,2) = 0;
  Sq(4,2) = 0;
  Sq(5,2) = 0;

  Sq(0,3) = 0;
  Sq(1,3) = 0;
  Sq(2,3) = 0;
  Sq(0,4) = 0;
  Sq(1,4) = 0;
  Sq(2,4) = 0;
  Sq(0,5) = 0;
  Sq(1,5) = 0;
  Sq(2,5) = 0;

  Sq(3,3) = R(0,0);
  Sq(4,3) = R(0,1);
  Sq(5,3) = R(0,2);
  Sq(3,4) = R(1,0);
  Sq(4,4) = R(1,1);
  Sq(5,4) = R(1,2);
  Sq(3,5) = R(2,0);
  Sq(4,5) = R(2,1);
  Sq(5,5) = R(2,2);

  return Sq;
}

rw::math::Jacobian get_Zimage( rw::models::Device::Ptr device, rw::kinematics::State state, rw::kinematics::Frame* cam_frame, float f, rw::math::Vector2D<double> UV, float z) {

  // get the Jacobian
  rw::math::Jacobian J = device->baseJframe(cam_frame, state);

  // get the matrix Sq from baseRcam
  rw::math::Jacobian Sq = get_Sq( device, state, cam_frame);

  // get Jimage from UV before displacement
  rw::math::Jacobian Jimage = get_Jimage( f, UV, z);
  rw::math::Jacobian Jimage_transpose(Jimage.e().transpose());

  // return Zimage
  return Jimage_transpose * Sq * J;
}

// based on the ex4_2 correction
rw::math::Q algorithm1(const rw::models::Device::Ptr device, rw::kinematics::State state, const rw::kinematics::Frame* cam_frame, rw::math::Jacobian Zimage, rw::math::Q q, rw::math::Vector2D<double> dUV) {

	  rw::math::Jacobian Zimage_tranpose(Zimage.e().transpose());
	  rw::math::Jacobian Zimage_inv((Zimage*Zimage_tranpose).e().inverse());

    rw::math::Jacobian duv(2,1);
    duv(0,0) = dUV(0);
    duv(1,0) = dUV(1);

    std::cout << "Zimage_inv from Algo1 :\n" << Zimage_inv << std::endl;

    // based on the equation (4.33)
    rw::math::Jacobian y = Zimage_inv*duv;
  	rw::math::Jacobian deltaQ = Zimage_tranpose*y;
    std::cout << "dQ from Algo1 : \n" << deltaQ << std::endl;

    rw::math::Q dq(7, deltaQ(0,0), deltaQ(0,1), deltaQ(0,2), deltaQ(0,3), deltaQ(0,4), deltaQ(0,5), deltaQ(0,6));
    return dq;
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

  std::cout << "UVref : " << UVref << std::endl;

  rw::math::Vector2D<double> dUV = get_du_dv( UVref, TransformMotions, 460, cam_frame, marker_frame, state, f);

  std::cout << "dUV : " << dUV << std::endl;

  rw::math::Jacobian Zimage = get_Zimage( device, state, cam_frame, f, UVref, z);

  std::cout << "Zimage :\n" << Zimage << std::endl;

  rw::math::Q dq = algorithm1( device, state, cam_frame, Zimage, q, dUV);

  q += dq;

  std::cout << "New q :\n" << q << std::endl;

  std::cout << "END" << std::endl;
  return 0;

}
