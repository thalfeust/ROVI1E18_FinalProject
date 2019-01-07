#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// Qt
#include <QTimer>

#include "ui_SamplePlugin.h"

// Project class
#include "Tracking.hpp"
#include "FeatureExtraction.hpp"

class SamplePlugin : public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
SamplePlugin();
virtual ~SamplePlugin();

virtual void open(rw::models::WorkCell* workcell);

virtual void close();

virtual void initialize();

cv::Mat grabPicture();
void printPicture( cv::Mat img);

private slots:
void btnPressed();
void timer();

void stateChangedListener(const rw::kinematics::State& state);

private:
static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

QTimer* _timer;

rw::models::WorkCell::Ptr _wc;
rw::kinematics::State _state;
rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
rwlibs::simulation::GLFrameGrabber* _framegrabber;

// Global Variables
Tracking tracker;
int indexTimer = 0;
bool mode1 = true;
bool mode1_1point = true;
bool mode2_color = true;

ColorSegmentation extraction_CS;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
