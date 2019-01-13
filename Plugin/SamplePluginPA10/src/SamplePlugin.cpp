#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>

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

// Parameters
std::string motionPath_Slow = "/home/student/Documents/ROVI1E18_FinalProject/Plugin/SamplePluginPA10/motions/MarkerMotionSlow.txt";
std::string motionPath_Medium = "/home/student/Documents/ROVI1E18_FinalProject/Plugin/SamplePluginPA10/motions/MarkerMotionMedium.txt";
std::string motionPath_Fast = "/home/student/Documents/ROVI1E18_FinalProject/Plugin/SamplePluginPA10/motions/MarkerMotionFast.txt";
std::string workcellPath = "/home/student/Documents/ROVI1E18_FinalProject/Workcells/PA10WorkCell/ScenePA10RoVi1.wc.xml";
std::string lenaPath = "/home/student/Documents/ROVI1E18_FinalProject/Plugin/SamplePluginPA10/src/lena.bmp";
std::string backgroundPath = "/home/student/Documents/ROVI1E18_FinalProject/Plugin/SamplePluginPA10/backgrounds/color1.ppm";
std::string colorPath = "/home/student/Documents/ROVI1E18_FinalProject/Plugin/SamplePluginPA10/markers/Marker1.ppm";

SamplePlugin::SamplePlugin() :
        RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png")) {
        setupUi(this);

        _timer = new QTimer(this);
        connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

        connect(radioButton_mode1,SIGNAL(pressed()), this, SLOT(btnPressed()) );
        connect(radioButton_mode2,SIGNAL(pressed()), this, SLOT(btnPressed()) );
        connect(pushButton_Load,SIGNAL(pressed()), this, SLOT(btnPressed()) );
        connect(pushButton_Play,SIGNAL(pressed()), this, SLOT(btnPressed()) );
        connect(pushButton_Break,SIGNAL(pressed()), this, SLOT(btnPressed()) );

        Image textureImage(300,300,Image::GRAY,Image::Depth8U);
        _textureRender = new RenderImage(textureImage);
        Image bgImage(0,0,Image::GRAY,Image::Depth8U);
        _bgRender = new RenderImage(bgImage,2.5/1000.0);
        _framegrabber = NULL;
}

SamplePlugin::~SamplePlugin() {
        delete _textureRender;
        delete _bgRender;
}

void SamplePlugin::initialize() {
        log().info() << "INITALIZE" << "\n";

        getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, _1), this);

        // Auto load workcell
        WorkCell::Ptr wc = WorkCellLoader::Factory::load(workcellPath);
        getRobWorkStudio()->setWorkCell(wc);

        // Load Lena image
        Mat im, image;
        im = imread( lenaPath, CV_LOAD_IMAGE_COLOR); // Read the file
        if(!im.data ) {
                RW_THROW("Could not open or find the image: please modify the file path in the source code!");
        }
        cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
        QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
        _label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}

void SamplePlugin::open(WorkCell* workcell) {
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
        //Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
        Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
        res.data = (uchar*)img.getImageData();
        return res;
}

void SamplePlugin::btnPressed() {
        QObject *obj = sender();

        if( obj==pushButton_Load) {
                log().info() << "Button Load\n";

                pushButton_Play->setEnabled(false);

                // Set a new texture (one pixel = 1 mm)
                Image::Ptr image;
                image = ImageLoader::Factory::load(backgroundPath);
                _bgRender->setImage(*image);

                //extraction_CS.plop();

                if( radioButton_mode1->isChecked()) {

                        if( rb_m1_Slow->isChecked()) {
                                label_mode->setText( "Mode Motions : Slow");
                                label_state->setText( "[load]");
                                tracker.getTransformMotions( motionPath_Slow);
                        }else if( rb_m1_Medium->isChecked()) {
                                label_mode->setText( "Mode Motions : Medium");
                                label_state->setText( "[load]");
                                tracker.getTransformMotions( motionPath_Medium);
                        }else if( rb_m1_Fast->isChecked()) {
                                label_mode->setText( "Mode Motions : Fast");
                                tracker.getTransformMotions( motionPath_Fast);
                        }

                        mode1 = true;

                        if( rb_1point->isChecked()) {

                                label_mode->setText( label_mode->text() + " 1 point");

                                mode1_1point = true;

                                // Set the Tracking variables
                                tracker.set( _wc, 1);
                        }else {

                                label_mode->setText( label_mode->text() + " 3 points");

                                mode1_1point = false;

                                // Set the Tracking variables
                                tracker.set( _wc, 3);
                        }

                        image = ImageLoader::Factory::load(colorPath);
                        _textureRender->setImage(*image);
                        // Update the maker frame
                        tracker.update_Marker( 0);

                }else { // Vision mode

                        // Set the Vision variables
                        extraction_CS.set( 2000.0, 0.78, 5, 1, cv::Size(9, 9), cv::Scalar(0, 0, 0), cv::Scalar(30, 255, 255), cv::Scalar(70, 0, 0), cv::Scalar(180, 255, 255));

                        mode1 = false;

                        // Set the Tracking variables
                        tracker.set( _wc, 3);

                        if( rb_m1_Slow->isChecked()) {
                                label_mode->setText( "Mode Vision : Slow");
                                label_state->setText( "[load]");
                                tracker.getTransformMotions( motionPath_Slow);
                        }else if( rb_m1_Medium->isChecked()) {
                                label_mode->setText( "Mode Vision : Medium");
                                label_state->setText( "[load]");
                                tracker.getTransformMotions( motionPath_Medium);
                        }else if( rb_m1_Fast->isChecked()) {
                                label_mode->setText( "Mode Vision : Fast");
                                tracker.getTransformMotions( motionPath_Fast);
                        }

                        label_mode->setText( label_mode->text() + " Color marker");
                        label_state->setText( "[load]");

                        image = ImageLoader::Factory::load("/home/student/Documents/ROVI1E18_FinalProject/Plugin/SamplePluginPA10/markers/Marker1.ppm");
                        _textureRender->setImage(*image);
                }

                // Update the robot
                tracker.device->setQ( tracker.qInit, tracker.state);
                getRobWorkStudio()->setState( tracker.state);
                getRobWorkStudio()->updateAndRepaint();
                printPicture( grabPicture());

                if( !mode1) {
                        std::cout << "set Mode 2\n";
                        cv::Mat img = grabPicture();

                        std::vector<cv::Point> center;
                        cv::Mat toPrint = extraction_CS.tick( img, center, checkBox_m2_openCV->isChecked());

                        if( rb_1point->isChecked()) {
                                mode1_1point = true;
                                tracker.set( _wc, 1);
                        }else {
                                mode1_1point = false;
                                tracker.set( _wc, 3);
                        }
                }

                pushButton_Play->setEnabled(true);
                pushButton_Play->setStyleSheet("background-color: green");

        }else if( obj==pushButton_Play) {

                log().info() << "Button Play / Stop\n";

                // Update the robot
                tracker.device->setQ( tracker.qInit, tracker.state);

                // Toggle the timer on and off
                if (!_timer->isActive()) {
                        indexTimer=0;
                        pushButton_Load->setEnabled(false);
                        pushButton_Break->setEnabled(true);
                        _timer->start( spinBox_deltaT->value());
                        pushButton_Play->setStyleSheet("background-color: red");
                        pushButton_Break->setStyleSheet("background-color: red");
                        label_state->setText( "[running...]");
                }else {
                        pushButton_Play->setStyleSheet("background-color: green");
                        pushButton_Break->setStyleSheet("background-color: white");
                        label_state->setText( "[stop]");
                        pushButton_Load->setEnabled(true);
                        pushButton_Break->setEnabled(false);
                        _timer->stop();
                }
        }else if( obj==pushButton_Break) {

                log().info() << "Button Break\n";

                // Toggle the timer on and off
                if (!_timer->isActive()) {
                        pushButton_Break->setStyleSheet("background-color: red");
                        label_state->setText( "[running...]");
                        _timer->start( spinBox_deltaT->value());
                }else {
                        pushButton_Break->setStyleSheet("background-color: green");
                        label_state->setText( "[break...]");
                        _timer->stop();
                }
        }
}

void SamplePlugin::timer() {
        if (_framegrabber != NULL) {
                log().info() << "Tick\n";

                // Update the maker frame
                tracker.update_Marker( indexTimer);

                if( mode1) {
                        if( mode1_1point) {
                                tracker.tick( indexTimer, false, 1, false);
                        }else {
                                tracker.tick( indexTimer, false, 3, false);
                        }
                }else {
                        std::cout << "Mode 2\n";

                        cv::Mat toPrint;
                        getRobWorkStudio()->setState( tracker.state);
                        getRobWorkStudio()->updateAndRepaint();

                        cv::Mat img = grabPicture();

                        // Extraction of the points from the picture
                        std::vector<cv::Point> center;
                        toPrint = extraction_CS.tick( img, center, checkBox_m2_openCV->isChecked());

                        // tracking
                        if( mode1_1point) {
                                tracker.tick( indexTimer, false, 1, false);
                        }else {
                                tracker.tick( indexTimer, false, 3, false);
                        }

                        printPicture( toPrint);
                }

                // Update the workcell
                //getRobWorkStudio()->setWorkCell(_wc);
                getRobWorkStudio()->setState( tracker.state);
                getRobWorkStudio()->updateAndRepaint();
                std::cout << "Index : " << indexTimer << "\nMarker : " << _wc->findFrame("Marker")->getTransform(tracker.state).R() << " " << _wc->findFrame("Marker")->getTransform(tracker.state).P() << "\n";

                if( mode1) {
                        printPicture( grabPicture());
                }

                indexTimer++;
                if ( indexTimer>=tracker.TransformMotions.size()) {
                        _timer->stop();
                }
        }
}

cv::Mat SamplePlugin::grabPicture() {
        // Get the image as a RW image
        Frame* cameraFrame = _wc->findFrame("CameraSim");
        _framegrabber->grab(cameraFrame, _state);
        const Image& image = _framegrabber->getImage();

        // Convert to OpenCV image
        Mat im = toOpenCVImage(image);
        Mat imflip;
        cv::flip(im, imflip, 0);

        return imflip;
}

void SamplePlugin::printPicture( cv::Mat img) {
        // Show in QLabel
        QImage img_out(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
        QPixmap p = QPixmap::fromImage(img_out);
        //unsigned int maxW = 400;
        //unsigned int maxH = 800;
        unsigned int maxW = 350;
        unsigned int maxH = 700;
        _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
}

void SamplePlugin::stateChangedListener(const State& state) {
        _state = state;
}
