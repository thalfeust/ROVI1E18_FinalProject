#include "Tracking.h"

void Tracking::getTransformMotions( std::string path) {
  // Open the file
  std::ifstream myfile;
  myfile.open (path, std::ios::in);

  // Initialization of the output
  std::vector<rw::math::Transform3D<double>> TransformMotionsCurrent;

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
      TransformMotionsCurrent.push_back(T);
    }

    myfile.close();
  }else {
    RW_THROW("Motions file not found!");
  }

  // Print the results
  for(unsigned int j = 0; j < TransformMotionsCurrent.size(); ++j) {
    std::cout << "Ts : " << j << " \n" << TransformMotionsCurrent[j] << std::endl;
  }

  // update the object
  TransformMotions = TransformMotionsCurrent;
}

void Tracking::superLoop( bool optionStoreTest) {

    // Get the velocity limit
    rw::math::Q qVelocityLimit = device->getVelocityLimits();

    // For each steps the new q is store
    rw::math::Q qStorage[TransformMotions.size()];
    int accessible[TransformMotions.size()]; // 0 : impossible, 1 : not reachable, 2 : yes

    for (unsigned int i=0; i<TransformMotions.size(); i++) {

      dq_from_dUV_computation result = algorithm1( i);

      if(result.dq.size()>0) {
        for (unsigned j=result.dq.size()-1; j>=0; j--) {
          bool reachable = true;
          for (unsigned w=0; w<q.size(); w++) {
            if( result.dq[j][w] / deltaT > qVelocityLimit[w]) {
              reachable = false;
              break;
            }
          }
          if( reachable) {
            q += result.dq[j];
            if( j!=result.dq.size()-1) {
              accessible[i] = 1;
            }else {
              accessible[i] = 2;
            }
            break;
          }else {
            accessible[i] = 0;
          }
        }
      }else {
        accessible[i] = 2;
      }

      // Update the storages;
      qStorage[i] = q;

      // Update the workcell
      device->setQ( q, state);
    }

    // Print the results
    std::cout << "\n\n\nQ results : " << std::endl;
    for (unsigned int i=0; i<TransformMotions.size(); i++) {
      std::cout << accessible[i] << "\t" << qStorage[i] << std::endl;
    }
}

void Tracking::compute( int index) {

    // Get the velocity limit
    rw::math::Q qVelocityLimit = device->getVelocityLimits();

    marker_frame->setTransform( TransformMotions[index], state);

    dq_from_dUV_computation result = algorithm1( index);

      if(result.dq.size()>0) {
        for (unsigned j=result.dq.size()-1; j>=0; j--) {
          bool reachable = true;
          for (unsigned w=0; w<q.size(); w++) {
            if( result.dq[j][w] / deltaT > qVelocityLimit[w]) {
              reachable = false;
              break;
            }
          }
          if( reachable) {
            q += result.dq[j];
          }
        }
      }

      // Update the workcell
      device->setQ( q, state);
  }

// based on the ex4_2 correction
dq_from_dUV_computation Tracking::algorithm1( int index) {

  // Initialization of the return variable
  dq_from_dUV_computation results;
  results.iterations = 0;

  rw::math::Q currentQ = q;

  // Comute du and dv
  rw::math::Vector2D<double> currentDUV = get_du_dv( index);

  const double epsilon = 1; // precision of 1 pixel

  // Loop until the precision is lower 1 pixel
  while( currentDUV.norm2() > epsilon) {

    // Get Zimage from the equation following the (eq.4.34)
    rw::math::Jacobian Zimage = get_Zimage( );

    // based on the way to solve the equation (eq.4.34)
    rw::math::Q dq( rw::math::LinearAlgebra::pseudoInverse(Zimage.e())*currentDUV.e());

    // Update the result of the function
    results.dq.push_back( dq);
    results.iterations++;

    // Update the state of the joints
    currentQ += dq;
    device->setQ( currentQ, state);

    //std::cout << "dQ from Algo1 : [" << results.iterations << "]\n" << dq << std::endl;

    // Compute the new du and dv
    currentDUV = get_du_dv( index);

    //std::cout << "currentDUV from Algo1 : \n" << currentDUV << std::endl;
  }
  return results;
}

rw::math::Jacobian Tracking::get_Zimage() {

  // get the Jacobian
  rw::math::Jacobian J = device->baseJframe( cam_frame, state);

  // get the matrix Sq from baseRcam
  rw::math::Jacobian Sq = get_Sq();

  // get Jimage from UV before displacement
  rw::math::Jacobian Jimage = get_Jimage();
  rw::math::Jacobian Jimage_transpose(Jimage.e().transpose());

  // return Zimage
  return Jimage_transpose * Sq * J;
}

rw::math::Jacobian Tracking::get_Sq() {

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

rw::math::Jacobian Tracking::get_Jimage() {

  rw::math::Jacobian Jimage(2);
  Jimage(0,0) = -f/z;
  Jimage(1,0) = 0;
  Jimage(2,0) = UVref[0]/z;
  Jimage(3,0) = UVref[0]*UVref[1]/f;
  Jimage(4,0) = -1 * ((f*f+UVref[0]*UVref[0])/f);
  Jimage(5,0) = UVref[1];
  Jimage(0,1) = 0;
  Jimage(1,1) = -f/z;
  Jimage(2,1) = UVref[1]/z;
  Jimage(3,1) = ((f*f+UVref[1]*UVref[1])/f);
  Jimage(4,1) = -(UVref[0]*UVref[1])/f;
  Jimage(5,1) = -UVref[0];

  return Jimage;
}

rw::math::Vector2D<double> Tracking::get_du_dv(int index) {

  rw::math::Transform3D<double> wTmarker = TransformMotions[index];

  marker_frame->setTransform( wTmarker, state);

  // Get new Pos of the marker inside Fcam
  rw::math::Transform3D<double> markerTcam = cam_frame->fTf( marker_frame, state);

  rw::math::Vector3D<double> P = markerTcam.P();
  rw::math::Vector2D<double> UV( f*P(0)/P(2), f*P(1)/P(2));
  rw::math::Vector2D<double> dUV( UVref(0)-UV(0), UVref(1)-UV(1));

  return dUV;
}