#include "Tracking.hpp"

/* Open the file by the given path in parameter and extract the motions to use.
 * The motions will be print on the screen iteratively.
 */
void Tracking::getTransformMotions( std::string path) {

        // Open the file
        std::ifstream myfile;
        myfile.open (path, std::ios::in);

        // Initialization of the output
        std::vector<rw::math::Transform3D<double> > TransformMotionsCurrent;

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

        // Print the extract motions
        for(unsigned int j = 0; j < TransformMotionsCurrent.size(); ++j) {
                std::cout << "Ts : " << j << " \n" << TransformMotionsCurrent[j] << std::endl;
        }

        // update the object
        TransformMotions = TransformMotionsCurrent;
}

/* Compute new Q for each motions of the marker using the pinhole model and the
 * inverse Kinematics.
 */
void Tracking::superLoop( bool optionStoreTest, int pointNumber) {

        // Verification
        if( pointNumber<1 and pointNumber>3 and pointNumber==2) {
                RW_THROW("Error mode");
        }

        // Get the velocity limit
        rw::math::Q qVelocityLimit = device->getVelocityLimits();

        // For each steps the new q is store
        rw::math::Q qStorage[TransformMotions.size()];
        int accessible[TransformMotions.size()]; // 0 : impossible, 1 : not reachable, 2 : yes

        q = device->getQ(state);
        for (unsigned int i=0; i<TransformMotions.size(); i++) {

                update_Marker( i);

                dq_from_dUV_computation result;
                if( pointNumber==1) {
                        result = algorithm1_1point( i, true);
                }else {
                        result = algorithm1_3point( i, true);
                }

                if(result.dq.size()>0) {
                        q += result.dq[result.dq.size()-1];
                }

                // Update the storages;
                qStorage[i] = q;

                // Update the workcell
                device->setQ( q, state);

                if( optionStoreTest) {
                        errorDUV.push_back( result.maxError);
                }

        }
        if( !optionStoreTest) {
                // Print the results
                std::cout << "\n\n\nQ results : " << std::endl;
                for (unsigned int i=0; i<TransformMotions.size(); i++) {
                        std::cout << i << " : " << qStorage[i] << "\nCam : " << cam_frame->wTf(state) << std::endl;
                }
        }
}

/* Compute dQ for a specific motion.
 * Algorithm adapted from the Newton-Raphson method.
 * based on the ex4_2 correction
 */
dq_from_dUV_computation Tracking::algorithm1_1point( int index, bool velocity) {

        // Initialization of the return variable
        dq_from_dUV_computation results;
        results.iterations = 0;

        rw::math::Q currentQ = device->getQ( state);
        rw::math::Q currentdQ;

        double maxError = 0;

        // get the velocity limits
        rw::math::Q qVelocityLimit = device->getVelocityLimits();

        // Comute du and dvobot
        //device->setQ( qInit, state);
        rw::math::Vector2D<double> currentDUV = get_du_dv_1point( index);


        const double epsilon = 1; // precision of 1 pixel

        // Loop until the precision is lower than 1 pixel
        while( currentDUV.norm2() > epsilon) {

                // Get Zimage from the equation following the (eq.4.34)
                rw::math::Jacobian Zimage = get_Zimage(1);

                // based on the way to solve the equation (eq.4.34)
                rw::math::Q dq( rw::math::LinearAlgebra::pseudoInverse(Zimage.e())*currentDUV.e());

                // Update the result of the function
                if(results.iterations > 1) {
                        currentdQ += dq;
                }else {
                        currentdQ = dq;
                }

                bool reachable = true;

                if( velocity) {
                        for (unsigned w=0; w<q.size(); w++) {
                                if( currentdQ[w] / deltaT > qVelocityLimit[w]) {
                                        reachable = false;
                                        break;
                                }
                        }
                }

                // The robot can't reach the position
                if( !reachable) {
                        break;
                }

                if(results.iterations >= 1) {
                        results.dq.push_back( dq+results.dq[ results.iterations-1]);
                }else {
                        results.dq.push_back( dq);
                }
                results.iterations++;

                // Update the state of the joints
                currentQ = device->getQ( state) + dq;
                device->setQ( currentQ, state);

                //std::cout << "dQ from Algo1 : [" << results.iterations << "]\n" << dq << std::endl;

                // Compute the new du and dv
                currentDUV = get_du_dv_1point( index);

                //update the ouput
                maxError = currentDUV.norm2();

                //std::cout << "currentDUV from Algo1 : \n" << currentDUV << std::endl;
        }
        results.maxError = maxError;

        return results;
}

/* Compute dQ for a specific motion.
 * Algorithm adapted from the Newton-Raphson method.
 * based on the ex4_2 correction
 */
dq_from_dUV_computation Tracking::algorithm1_3point( int index, bool velocity) {

        // Initialization of the return variable
        dq_from_dUV_computation results;
        results.iterations = 0;

        rw::math::Q currentQ = device->getQ( state);
        rw::math::Q currentdQ;

        double maxError = 0;

        // get the velocity limits
        rw::math::Q qVelocityLimit = device->getVelocityLimits();

        // Comute du and dvobot
        //device->setQ( qInit, state);
        rw::math::
        VelocityScrew6D<double> currentDUV = get_du_dv_3point( index);


        const double epsilon = 1; // precision of 1 pixel

        // Loop until the precision is lower than 1 pixel
        while( currentDUV.norm2() > epsilon) {

                // Get Zimage from the equation following the (eq.4.34)
                rw::math::Jacobian Zimage = get_Zimage(3);

                // based on the way to solve the equation (eq.4.34)
                rw::math::Q dq( rw::math::LinearAlgebra::pseudoInverse(Zimage.e())*currentDUV.e());

                // Update the result of the function
                if(results.iterations > 1) {
                        currentdQ += dq;
                }else {
                        currentdQ = dq;
                }

                bool reachable = true;

                if( velocity) {
                        for (unsigned w=0; w<q.size(); w++) {
                                if( currentdQ[w] / deltaT > qVelocityLimit[w]) {
                                        reachable = false;
                                        break;
                                }
                        }
                }

                // The robot can't reach the position
                if( !reachable) {
                        break;
                }

                if(results.iterations >= 1) {
                        results.dq.push_back( dq+results.dq[ results.iterations-1]);
                }else {
                        results.dq.push_back( dq);
                }
                results.iterations++;

                // Update the state of the joints
                currentQ = device->getQ( state) + dq;
                device->setQ( currentQ, state);

                //std::cout << "dQ from Algo1 : [" << results.iterations << "]\n" << dq << std::endl;

                // Compute the new du and dv
                currentDUV = get_du_dv_3point( index);

                //update the ouput
                maxError = currentDUV.norm2();

                //std::cout << "currentDUV from Algo1 : \n" << currentDUV << std::endl;
        }
        results.maxError = maxError;

        return results;
}

/* Compute the matrix Zimage from the matrixes Jimage, J and S(q). */
rw::math::Jacobian Tracking::get_Zimage( int pointNumber) {

        // get the Jacobian matrix
        rw::math::Jacobian J = device->baseJframe( cam_frame, state);

        // get the matrix Sq from baseRcam
        rw::math::Jacobian Sq = get_Sq();

        // get Jimage from UV before displacement
        rw::math::Jacobian Jimage = get_Jimage( pointNumber);
        rw::math::Jacobian Jimage_transpose(Jimage.e().transpose());

        // return Zimage
        return Jimage_transpose * Sq * J;
}

/* Compute the matrix S(q) from camRbase(q) matrix.
 * Equation based on the Robotic Notes (next to 4.32).
 */
rw::math::Jacobian Tracking::get_Sq() {

        rw::math::Jacobian Sq(6);
        // Compute camRbase
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

/* Compute the Jimage matrix from the elements (u,v), z and f.
 */
rw::math::Jacobian Tracking::get_Jimage( int pointNumber) {

        if( pointNumber<1 or pointNumber==2 or pointNumber>3) {
                RW_THROW("Error number of points");
        }

        if( pointNumber==1) {
                std::cout << "Jimage1\n";
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
        }else {
                std::cout << "Jimage3\n";
                rw::math::Jacobian Jimage(6);
                Jimage(0,0) = -f/z;
                Jimage(1,0) = 0;
                Jimage(2,0) = UVref1[0]/z;
                Jimage(3,0) = UVref1[0]*UVref1[1]/f;
                Jimage(4,0) = -1 * ((f*f+UVref1[0]*UVref1[0])/f);
                Jimage(5,0) = UVref1[1];
                Jimage(0,1) = 0;
                Jimage(1,1) = -f/z;
                Jimage(2,1) = UVref1[1]/z;
                Jimage(3,1) = ((f*f+UVref1[1]*UVref1[1])/f);
                Jimage(4,1) = -(UVref1[0]*UVref1[1])/f;
                Jimage(5,1) = -UVref1[0];

                Jimage(0,2) = -f/z;
                Jimage(1,2) = 0;
                Jimage(2,2) = UVref2[0]/z;
                Jimage(3,2) = UVref2[0]*UVref2[1]/f;
                Jimage(4,2) = -1 * ((f*f+UVref2[0]*UVref2[0])/f);
                Jimage(5,2) = UVref2[1];
                Jimage(0,3) = 0;
                Jimage(1,3) = -f/z;
                Jimage(2,3) = UVref2[1]/z;
                Jimage(3,3) = ((f*f+UVref2[1]*UVref2[1])/f);
                Jimage(4,3) = -(UVref2[0]*UVref2[1])/f;
                Jimage(5,3) = -UVref2[0];

                Jimage(0,4) = -f/z;
                Jimage(1,4) = 0;
                Jimage(2,4) = UVref3[0]/z;
                Jimage(3,4) = UVref3[0]*UVref3[1]/f;
                Jimage(4,4) = -1 * ((f*f+UVref3[0]*UVref3[0])/f);
                Jimage(5,4) = UVref3[1];
                Jimage(0,5) = 0;
                Jimage(1,5) = -f/z;
                Jimage(2,5) = UVref3[1]/z;
                Jimage(3,5) = ((f*f+UVref3[1]*UVref3[1])/f);
                Jimage(4,5) = -(UVref3[0]*UVref3[1])/f;
                Jimage(5,5) = -UVref3[0];
                return Jimage;
        }
}

/* This function is used to get the value of a displacement (du,dv) from the
 * coordinate (x,y,z) of a point on the picture in the coordinate frame of the
 * camera. The calculation is based on the pin hole model.
 */
rw::math::Vector2D<double> Tracking::get_du_dv_1point(int index) {
        std::cout << "DUV1\n";
        // Get new Pos of the marker inside Fcam
        rw::math::Transform3D<double> markerTcam = cam_frame->fTf( marker_frame, state);

        rw::math::Vector3D<double> P = markerTcam.P();
        rw::math::Vector2D<double> UV( f*P(0)/P(2), f*P(1)/P(2));
        rw::math::Vector2D<double> dUV( UVref(0)-UV(0), UVref(1)-UV(1));

        return dUV;
}

rw::math::VelocityScrew6D <double> Tracking::get_du_dv_3point(int index) {
        std::cout << "DUV3\n";
        // Get new Pos of the marker inside Fcam
        rw::math::Transform3D<double> markerTcam = cam_frame->fTf( marker_frame, state);

        rw::math::Vector3D<double> P;

        P = markerTcam*pt1;
        rw::math::Vector2D<double> UV1( f*(P(0)/P(2)), f*(P(1)/P(2)));

        P = markerTcam*pt2;
        rw::math::Vector2D<double> UV2( f*(P(0)/P(2)), f*(P(1)/P(2)));

        P = markerTcam*pt3;
        rw::math::Vector2D<double> UV3( f*(P(0)/P(2)), f*(P(1)/P(2)));

        rw::math::VelocityScrew6D<double> dUV( UVref1(0)-UV1(0), UVref1(1)-UV1(1), UVref2(0)-UV2(0), UVref2(1)-UV2(1), UVref3(0)-UV3(0), UVref3(1)-UV3(1));

        return dUV;
}

/* Update the marker position and rotation in the workcell using a
 * motion.
 */
void Tracking::update_Marker( int index) {
        rw::math::Transform3D<double> wTmarker = TransformMotions[index];

        marker_frame->setTransform( wTmarker, state);
}

/* Set a Tracking object with the element of the Workcell wc.
 */
void Tracking::set( rw::models::WorkCell::Ptr wc, int mode) {

        if( mode<1 and mode>3 and mode==2) {
                RW_THROW("Error mode");
        }

        const std::string device_name = "PA10";

        // Get device and check if it has been loaded correctly
        device = wc->findDevice(device_name);
        if(device == nullptr) {
                RW_THROW("Device " << device_name << " was not found!");
        }

        // Find Camera
        cam_frame = wc->findFrame("Camera");
        if(cam_frame == nullptr) {
                RW_THROW("Camera frame not found!");
        }

        state = wc->getDefaultState();

        marker_frame = dynamic_cast<rw::kinematics::MovableFrame*>(wc->findFrame("Marker"));
        if(marker_frame == nullptr) {
                RW_THROW("Marker frame not found!");
        }

        rw::math::Q qq(7,0,-0.65,0,1.80,0,0.42,0);
        qInit = qq;
        q = qq;
        deltaT = 1;
        f = 823;
        z = 0.5;

        if( mode==1) {
                set_1point( wc);
        }else {
                set_3points( wc);
        }
}

void Tracking::set_1point( rw::models::WorkCell::Ptr wc) {
        // Get Pos of the marker inside Fcam to compute UVref
        rw::math::Transform3D<double> markerTcam = cam_frame->fTf( marker_frame, state);
        rw::math::Vector3D<double> P = markerTcam.P();
        rw::math::Vector2D<double> uvref( f*P(0)/P(2), f*P(1)/P(2));
        UVref = uvref;
}

void Tracking::set_3points( rw::models::WorkCell::Ptr wc) {
        // Get Pos of the marker inside Fcam to compute UVref
        rw::math::Transform3D<double> markerTcam = cam_frame->fTf( marker_frame, state);
        rw::math::Vector3D<double> P = markerTcam.P();
        UVref = rw::math::Vector2D<double>( f*P(0)/P(2), f*P(1)/P(2));

        pt1 = rw::math::Vector3D<double>( 0.15, 0.15, 0.0);
        pt2 = rw::math::Vector3D<double>( -0.15, 0.15, 0.0);
        pt3 = rw::math::Vector3D<double>( 0.15, -0.15, 0.0);

        P = markerTcam*pt1;
        UVref1 = rw::math::Vector2D<double>( f*(P(0)/P(2)), f*(P(1)/P(2)));

        P = markerTcam*pt2;
        UVref2 = rw::math::Vector2D<double>( f*(P(0)/P(2)), f*(P(1)/P(2)));

        P = markerTcam*pt3;
        UVref3 = rw::math::Vector2D<double>( f*(P(0)/P(2)), f*(P(1)/P(2)));
}

/* Like the superLoop function but used for the GUI.
 */
void Tracking::tick( int index, bool velocity, int pointNumber) {

        // Verification
        if( pointNumber<1 or pointNumber>3 or pointNumber==2) {
                RW_THROW("Error mode");
        }

        q = device->getQ(state);

        dq_from_dUV_computation result;

        if( pointNumber==1) {
                result = algorithm1_1point( index, velocity);
        }else {
                result = algorithm1_3point( index, velocity);
        }

        if(result.dq.size()>0) {
                q += result.dq[result.dq.size()-1];
        }

        // Update the robot
        device->setQ( q, state);
}

/* Return a string with the informations about the algorithm ( index,
 * ...)
 */
std::string Tracking::print( rw::models::WorkCell::Ptr wc, int index) {
        char* str;
        std::string toReturn = "Index : ";
        std::sprintf(str,"%d",index);
        toReturn.append( str);
        toReturn.append("\nMarker : Rotation3D[");

        /*for( int i=0; i<3; i++) {
                for( int j=0; j<3; j++) {
                        std::sprintf(str,"%f",wc->findFrame("Marker")->getTransform( state).R().getCol(i)[j]);
                        toReturn.append(str);
                        if( i!=2 and j!=2) {toReturn.append(",");}
                }
           }*/

        toReturn.append("]");

        return toReturn;
}

void Tracking::testError_from_deltaT() {
        for( float i=0.05; i<=1; i+=0.05) {
                device->setQ( qInit, state);
                deltaT = i;
                errorDUV.clear();

                superLoop( true, 1);
                //std::cout << deltaT << " : [";
                std::cout << "[";
                for( int j=0; j<errorDUV.size(); j++) {
                        std::cout << errorDUV[j] << ",";
                }
                std::cout << ";\n";
        }
        std::cout << "]\n";
}
