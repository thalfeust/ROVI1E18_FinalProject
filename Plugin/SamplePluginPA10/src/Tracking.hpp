#ifndef TRACKING_HPP
#define TRACKING_HPP

#include <fstream>
#include <iostream>
#include <rw/models.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/math.hpp>
#include <rw/math/Vector.hpp>
#include <string>
#include <stdlib.h>

struct dq_from_dUV_computation {
        std::vector<rw::math::Q> dq; // every q used to track the marker
        std::vector<double> error; // error in posion for each steps
        double maxError;
        int iterations; // number of iterations
};

class Tracking {
public:
// Every movements
std::vector<rw::math::Transform3D<double> > TransformMotions;
rw::models::Device::Ptr device;
rw::kinematics::State state;
rw::kinematics::Frame* cam_frame;
rw::kinematics::Frame* tool_frame;
rw::kinematics::MovableFrame* marker_frame;
rw::math::Q q;
rw::math::Q qInit;
rw::math::Vector2D<double> UVref;
// tracking 3 points
rw::math::Vector3D<double> pt1;
rw::math::Vector3D<double> pt2;
rw::math::Vector3D<double> pt3;
rw::math::Vector2D<double> UVref1;
rw::math::Vector2D<double> UVref2;
rw::math::Vector2D<double> UVref3;

double errorDUV;
bool isReachable;

float f;
float z;
float deltaT;

void getTransformMotions( std::string path);
void testError_from_deltaT( int pointNumber);
void superLoop( bool velocity, int pointNumber, int printOutputMode);
dq_from_dUV_computation algorithm1_1point( int index, bool velocity);
dq_from_dUV_computation algorithm1_3point( bool velocity, bool fromVision);
rw::math::Jacobian get_Zimage( int pointNumber);
rw::math::Jacobian get_Sq();
rw::math::Jacobian get_Jimage( int pointNumber);
rw::math::Vector2D<double> get_du_dv_1point();
rw::math::VelocityScrew6D <double> get_du_dv_3point( bool fromVision);
void update_Marker( int index);
void set( rw::models::WorkCell::Ptr wc, int mode);
void set_1point( rw::models::WorkCell::Ptr wc);
void set_3points( rw::models::WorkCell::Ptr wc);
void set_3pointsFromVision( rw::models::WorkCell::Ptr wc);
void tick( int index, bool velocity, int pointNumber, bool fromVision);
bool tickFromVision( int index, bool velocity, int pointNumber, bool fromVision);
std::string print( rw::models::WorkCell::Ptr wc, int index);
};

#endif
