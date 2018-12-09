#include <fstream>
#include <iostream>
#include <rw/rw.hpp>
#include <string>
#include <stdlib.h>

struct dq_from_dUV_computation {
  std::vector<rw::math::Q> dq; // every dq used to track the marker
  int iterations; // number of iterations
} ;

class Tracking{
public:
  // Every movements
  std::vector<rw::math::Transform3D<double>> TransformMotions;
  rw::models::Device::Ptr device;
  rw::kinematics::State state;
  rw::kinematics::Frame* cam_frame;
  rw::kinematics::MovableFrame* marker_frame;
  rw::math::Q q;
  rw::math::Vector2D<double> UVref;

  float f;
  float z;
  float deltaT;

  void getTransformMotions( std::string path);
  void superLoop( bool optionStoreTest);
  dq_from_dUV_computation algorithm1( int index);
  rw::math::Jacobian get_Zimage();
  rw::math::Jacobian get_Sq();
  rw::math::Jacobian get_Jimage();
  rw::math::Vector2D<double> get_du_dv(int index);
};
