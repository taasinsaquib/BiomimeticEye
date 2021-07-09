#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <osg/Timer>
#include <osgUtil/SceneView>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Quat>
#include <osg/Light>
#include <osg/LightSource>

//SNOPT
// char*		snCw=0;
// integer*	snIw=0;
// doublereal* snRw=0;

FILE* fileMotionLEye = 0;
FILE* fileMuscleMotionLEye = 0;
FILE* fileDimensionsPupil = 0;
FILE* fileDimensionsLens = 0;
FILE* fileRaytracePoints = 0;

FILE* fileDumbbell = 0;
FILE* fileBall = 0;
FILE* flarm = 0;
// std::vector<gMuscleList> exportMuscles;
bool exportMotion = false;
std::vector< std::vector<double> > reverseRayBuffer;
void exportReverseTraceBuffer();

osg::ref_ptr<osgUtil::SceneView> sceneView;
osg::ref_ptr<osg::Image> imageDump = NULL;

// window
int wndWidth = 1280;
int wndHeight = 720;
int lastx, lasty;
osg::Vec3 centerPos;
float viewRot, viewElev, viewRadius;
typedef enum {eRotate, eTrans, eZoom} _MouseMode;
_MouseMode mouseMode;

// timesteps
gVec3 targeting_location(0,0,0);

// options
bool bPlay = false;
bool dumpImages = false;

//frame
int dumpFrame = 0;
int frame=0;

// global variables
// FILE* fpT=0;
// FILE* fpPt=0;
// FILE* fpPh=0;
// FILE* fpQ = 0;

gReal curTime = 0;
char imageName[128];

int RUNNING_MODE = 0; //default = freefall

#define FREE_FALL 0
#define GENERATE_TRAINING_DATASET_PUPIL 1
#define GENERATE_TRAINING_DATASET_LENS 2
#define GENERATE_TRAINING_DATASET_FOVEATION 3
#define GENERATE_TRAINING_DATASET_MOTOR 4
#define EYE_CONTROL 5

gRigidBody* Manipulator=NULL;
osg::ref_ptr<osg::MatrixTransform> visManip = NULL;

/***************************************************************************/

osg::ref_ptr<osg::MatrixTransform> visualTarget;	// doll

bool show_reverse_trace_rays;

#endif