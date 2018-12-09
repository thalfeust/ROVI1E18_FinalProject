#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>

#include <fstream>
#include <iostream>
#include <rw/rw.hpp>
#include <string>
#include <stdlib.h>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;

using namespace cv;

using namespace std::placeholders;

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
  void compute( int index);
  dq_from_dUV_computation algorithm1( int index);
  rw::math::Jacobian get_Zimage();
  rw::math::Jacobian get_Sq();
  rw::math::Jacobian get_Jimage();
  rw::math::Vector2D<double> get_du_dv(int index);
};

bool markerColor = true;
bool markerCorny = false;

// initialization
Tracking tracker;

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );
  connect(_checkBox_color  ,SIGNAL(stateChanged(int)), this, SLOT(btnPressed()) );
  connect(_checkBox_corny  ,SIGNAL(stateChanged(int)), this, SLOT(btnPressed()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}

SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize() {
	log().info() << "INITALIZE" << "\n";

	getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, _1), this);

	// Auto load workcell
	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/student/Documents/ROVI1E18_FinalProject/Workcells/PA10WorkCell/ScenePA10RoVi1.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);

	// Load Lena image
	Mat im, image;
	im = imread("/home/student/Documents/ROVI1E18_FinalProject/Plugin/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
	cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
	if(! image.data ) {
		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
	}
	QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin

  const std::string device_name = "PA10";
  tracker.device = _wc->findDevice(device_name);
  tracker.cam_frame = _wc->findFrame("Camera");

  // Get an initial configuration, q
  tracker.state = _wc->getDefaultState();
  rw::math::Q qq(7,0,-0.65,0,1.80,0,0.42,0);
  tracker.q = qq;
  tracker.device->setQ( tracker.q, tracker.state);
  tracker.deltaT = 1;
  tracker.f = 823;
  tracker.z = 0.5;

  rw::math::Transform3D<double> markerTcam = tracker.cam_frame->fTf( tracker.marker_frame, tracker.state);
  rw::math::Vector3D<double> P = markerTcam.P();
  rw::math::Vector2D<double> uvref( tracker.f*P(0)/P(2), tracker.f*P(1)/P(2));
  tracker.UVref = uvref;

}

void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
	// Add the texture render to this workcell if there is a frame for texture
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
	}
	// Add the background render to this workcell if there is a frame for texture
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
	}

	// Create a GLFrameGrabber if there is a camera frame with a Camera property set
	Frame* cameraFrame = _wc->findFrame("CameraSim");
	if (cameraFrame != NULL) {
		if (cameraFrame->getPropertyMap().has("Camera")) {
			// Read the dimensions and field of view
			double fovy;
			int width,height;
			std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
			std::istringstream iss (camParam, std::istringstream::in);
			iss >> fovy >> width >> height;
			// Create a frame grabber
			_framegrabber = new GLFrameGrabber(width,height,fovy);
			SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
			_framegrabber->init(gldrawer);
		}
	}
    }
}


void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}


void SamplePlugin::btnPressed() {
    QObject *obj = sender();
	if(obj==_btn0){
		log().info() << "Button 0\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
		image = ImageLoader::Factory::load("/home/student/Documents/ROVI1E18_FinalProject/Plugin/SamplePluginPA10/markers/Marker1.ppm");
		_textureRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
	} else if(obj==_btn1){
		log().info() << "Button 1\n";
		// Toggle the timer on and off
		if (!_timer->isActive())
		    _timer->start(1000); // run 1 Hz
		else
			_timer->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	} else if( obj==_checkBox_color) {
    if( _checkBox_corny->checkState()==Qt::Checked){
      log().info() << "markerColor\n";
      markerColor = true;
      markerCorny = false;
      _checkBox_corny->setCheckState(Qt::Unchecked);
    }else {
      markerColor = false;
    }
  } else if( obj==_checkBox_corny) {
    if( _checkBox_corny->checkState()==Qt::Checked){
      log().info() << "markerCorny\n";
      markerColor = false;
      markerCorny = true;
      _checkBox_color->setCheckState(Qt::Unchecked);
    }else {
      markerCorny = false;
    }
  }
}

void SamplePlugin::timer() {
	if (_framegrabber != NULL) {
		// Get the image as a RW image
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		_framegrabber->grab(cameraFrame, _state);
		const Image& image = _framegrabber->getImage();

		// Convert to OpenCV image
		Mat im = toOpenCVImage(image);
		Mat imflip;
		cv::flip(im, imflip, 0);

		// Show in QLabel
		QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
		QPixmap p = QPixmap::fromImage(img);
		unsigned int maxW = 400;
		unsigned int maxH = 800;
		_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
	}
}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}

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
