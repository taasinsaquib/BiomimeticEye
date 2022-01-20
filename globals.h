#ifndef GLOBALS_H
#define GLOBALS_H

#include <omp.h> 
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>  
#include <math.h>
#include <windows.h>
#include <GL/glut.h>
#include <osg/Timer>
#include <osgUtil/SceneView>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Quat>
#include <osg/Light>
#include <osg/LightSource>
#include "snopt/snopt_c.h"

#include "HuMuLEye.h"

//#include "HuMuSkel.h"
//#include "HuMuSkel.h"
//#include "HuMuSkelNeck.h"
//#include "HuMuSkelRestOfBody.h"

#include "gkd/gkd_general.h"
#include "gkd/gkd_multibody_system.h"
#include "gkd/gkd_system_vis.h"
#include "gkd/gkd_system_md.h"
#include "gkd/gkd_util.h"
#include "gkd/gkd_multibody_system_loader.h"
#include "gkd/gkd_osg_util.h"

#include "MuscleOptimizer/MuscleOptimizer.h"

#include "SdA\Sda_Decoder.h"

#include "RandomMotion.h"
#include <osg/MatrixTransform>

#include "Skinning.h"
#include <osg/PositionAttitudeTransform>
#include <osg/Material>

#include <osg/StateSet>
#include <osg/PrimitiveSet>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Array>
#include <osg/Geometry>
#include <osg/BlendFunc>
#include "dqconv.h"

#include <gsl/gsl_linalg.h>

#define PI 3.14159265

//extern gVec3 GRAVITY_VECTOR(0,0,0);//
extern gVec3 GRAVITY_VECTOR(0, 0, -9810.); // mm/sec^2, -Z						// I stop the Gravity fow now. It is too hard to compute muscle dynamicswith gravity
//extern gVec3 GRAVITY_VECTOR(0,0,0);


using namespace std;

int NumThreads;	// number of threads 

osg::ref_ptr<osgUtil::SceneView> sceneView;
osg::ref_ptr<osg::Group>		 sceneGroup;
osg::ref_ptr<osg::MatrixTransform>	 box;
osg::ref_ptr<osg::Image> imageDump = NULL;
osg::ref_ptr<osg::Image> imageDumpRetina = NULL;

// left eye
HuMuLEye* SysLEye = NULL;
gVisSystemMD* VisLEye = NULL;
osg::ref_ptr<osg::Group> leyeGroup;
gSystemSolver* SolverLeye = NULL;
gSystemMDCtrlVol* VoluntaryLEye = NULL;
gSystemMDCtrlReflex* ReflexLEye = NULL;
gSystemMD* SysMDLeye = NULL; //point to Sys

GM* PF_ub = NULL;	//upper body
GM* PF_ubLeye = NULL;	//upper body
GM* PF_ubNeck = NULL;	//upper body
GM* PF_ubRarm = NULL;	//upper body

//SNOPT
char* snCw = 0;
integer* snIw = 0;
doublereal* snRw = 0;
integer		snLenCw = 8 * 500 * 20;
integer		snLenIw = 30000 * 20;
integer		snLenRw = 140000 * 20;

FILE* fileMotionLEye = 0;
FILE* fileMuscleMotionLEye = 0;
FILE* fileDimensionsPupil = 0;
FILE* fileDimensionsLens = 0;
FILE* fileRaytracePoints = 0;

FILE* fileDumbbell = 0;
FILE* fileBall = 0;
FILE* flarm = 0;
std::vector<gMuscleList> exportMuscles;
bool exportMotion = false;
std::vector<std::vector<double>> reverseRayBuffer;


//PCA
GM* U_t = NULL;	//principal components
GV* mean_t = NULL; // mean

//mocap
bool useMocap = false;
gMultibodySystem* MocapSys = NULL;
gVisSystem* MocapVis = NULL;
gMotionDataAMC* AmcData = NULL;
gRotMat Lshoulder;
gRotMat Rshoulder;

bool showMocap = false;

////window
//this is for the final demo
//int wndWidth = 1280;//800;//1200;
//int wndHeight = 720;//600;//900;
//this is for testing and dev
//int wndWidth = 640;//800;//1200;
//int wndHeight = 360;//600;//900;
int wndWidth = 1280;
int wndHeight = 720;
int wndWidthRetina = 300;//800;//1200;
int wndHeightRetina = 300;//600;//900;
int lastx, lasty;
osg::Vec3 centerPos;
float viewRot, viewElev, viewRadius;
typedef enum { eRotate, eTrans, eZoom } _MouseMode;
_MouseMode mouseMode;

int num_pupil_rings = 1;
double vis_target_ec;

int num_lens_rings = 2;
double lens_arc_increment = 0.0;

double dlensmin = 0.9508;
double dlensmax = 3.8448;

float yplane = -500;

//timesteps
//gReal displayTimeStep = 1./30;
const gReal displayTimeStep = 1. / 100;
//const gReal simulationTimeStep = 0.001;				//FOR backward Euler method
//const gReal simulationTimeStep = 1./100;				//FOR backward Euler method we need this smaller time step when we have neck
const gReal simulationTimeStep = 1. / 1000;				//FOR backward Euler method we need this smaller time step when we have neck
//gReal simulationTimeStep =  0.02;				//FOR backward Euler method
gReal simulationTimeStepExplicitEuler = 1.14e-6;//FOR forward Euler method
gVec3 targeting_location(0, 0, 0);

//options
bool bPlay = false;
bool bDrop = false;
bool muscleTuning = false;
bool dumpImages = false;
bool recordMuscleAct = false;

//frame
int pert_round = 1;
int LARMCOUNT = 0;
int simCount = 0;
int dumpFrame = 0;
int dumpFrameRetina = 0;
int frame = 0;
int breatheFrame = 0;

//global variables
FILE* fpT = 0;
FILE* fpPt = 0;
FILE* fpPh = 0;
FILE* fpQ = 0;

GV* tmp = 0;
gReal curTime = 0;
char imageName[128];
char imageNameRetina[128];


int RUNNING_MODE = 0; //default = freefall

#define FREE_FALL 0
#define GENERATE_TRAINING_DATASET_PUPIL 1
#define GENERATE_TRAINING_DATASET_LENS 2
#define GENERATE_TRAINING_DATASET_FOVEATION 3
#define GENERATE_TRAINING_DATASET_MOTOR 4
#define EYE_CONTROL 5

#define NUM_MUSCLES_NECK 217
#define NUM_MUSCLES_LEYE 6
//#define ButterFly 13
//#define LEG_KICK_CANNON 13
//
gRigidBody* Manipulator = NULL;
osg::ref_ptr<osg::MatrixTransform> visManip = NULL;
osg::ref_ptr<osg::MatrixTransform> visManip2 = NULL;
osg::ref_ptr<osg::MatrixTransform> visManipStool = NULL;
osg::ref_ptr<osg::MatrixTransform> visManipCannon = NULL;
osg::ref_ptr<osg::MatrixTransform> visManipRacket = NULL;
osg::ref_ptr<osg::MatrixTransform> visManipBullet = NULL;
osg::ref_ptr<osg::MatrixTransform> visManipCowboyhat = NULL;
osg::ref_ptr<osg::MatrixTransform> visManipBulb = NULL;

osg::ref_ptr<osg::MatrixTransform> visManip3 = NULL;
osg::ref_ptr<osg::MatrixTransform> visManip4 = NULL;
osg::ref_ptr<osg::MatrixTransform> visManip5 = NULL;
FILE* fileStaticPose = NULL;
FILE* fileDynamicPose = NULL;
const char fileNameStaticPose[] = "staticPose.txt";
const char fileNameDynamicPose[] = "dynamicPose.txt";

gVec3 initial_pos_head, initial_pos_trunk, initial_pos_larm;//, initial_pos_rarm;

#define DEG_TO_RAD	1.74532925199432957692369076848e-2	//(M_PI/180.0)
#define	D2R			DEG_TO_RAD	
FILE* fbody = 0;//masaki
FILE* fbody_input = 0;//masaki


Sda_Decoder* sda;
ifstream myfile;
bool bTargetPosAchieved;


//those macros are for generating training set. 
#define X_NUM 5
#define Y_NUM 5
#define Z_NUM 5

#define X_MAX 740.0
#define Y_MAX 180.0
#define Z_MIN -70

#define X_MIN 700.0
#define Y_MIN 110.0
#define Z_MAX 20.0


float muscle_activation_data[3000][29];

long double muscle_activation_data_delta_leye[5000][6];
long double muscle_activation_data_leye[5000][9];
double resets_leye[5000];

double muscle_activation_data_delta_leye_pupil[1500];
double ballpos_leye_pupil[1500][4];
double random_lens_cilliary_act[1500];
double image_leye_pupil[40 * 360];
double muscle_activation_data_leye_pupil_start;

double muscle_activation_data_delta_leye_lens[1500];
double ballpos_leye_lens[1500][4];
double random_pupil_sphincter_act[1500];
double image_leye_lens[40 * 360];
double muscle_activation_data_leye_lens_start;

double image_leye_perception[40 * 360];
double etheta_ephi_perception[1500][2];
double random_pupil_sphincter_act_foveation[1500];
double random_lens_cilliary_act_foveation[1500];
double ballpos_leye_foveation[1500][4];

/***************************************************************************/

//#include "Keras\keras_model.h"
//#include <iterator>

using namespace keras;

KerasModel* m;
KerasModel* mLEye;

double meanValueLEye[NUM_MUSCLES_LEYE];
double stdValueLEye[NUM_MUSCLES_LEYE];

DataChunkFlat* sampleLEye;

RandomMotion randomMotion;
RandomMotion randomMotion2;
osg::ref_ptr<osg::MatrixTransform> visualTarget;	// doll
osg::ref_ptr<osg::MatrixTransform> visualTarget2;	// doll
osg::ref_ptr<osg::MatrixTransform> visualBounceBoard1;	// doll
osg::ref_ptr<osg::MatrixTransform> visualBounceBoard2;	// doll
osg::ref_ptr<osg::MatrixTransform> visualBounceBoard3;	// doll
osg::ref_ptr<osg::MatrixTransform> eyeBallLeft;	// doll
osg::ref_ptr<osg::MatrixTransform> eyeBallRight;	// doll
osg::ref_ptr<osg::MatrixTransform> headSkin;	// doll
osg::ref_ptr<osg::MatrixTransform> visualEyeGazeHitbox;

bool new_target_ball;

gVec3 prev_pos, after_pos;

gVec3* SkinVertices;
osg::PositionAttitudeTransform* MeshTransform;
osg::ref_ptr<osg::Node> HumanMesh;
osg::StateSet* stateset;
gSystemMeshLoader MeshObj;

osg::ref_ptr<osgUtil::SceneView> sceneViewLeft;
osg::ref_ptr<osgUtil::SceneView> sceneViewRight;
osg::ref_ptr<osgUtil::SceneView> sceneView_PR;
osg::ref_ptr<osgUtil::SceneView> sceneView_PRRight;
osg::ref_ptr<osg::Group>		 sceneGroupLeft;
osg::ref_ptr<osg::Group>		 sceneGroupRight;
osg::ref_ptr<osg::Group>		 sceneGroup_PR;
osg::ref_ptr<osg::Group>		 sceneGroup_PRRight;
osg::ref_ptr<osg::Group>		 sceneGroup_raytrace;

osg::ref_ptr<osgUtil::SceneView> sceneView_PRTest;
osg::ref_ptr<osg::Group>		 sceneGroup_PRTest;

osg::PositionAttitudeTransform* pat;
osg::PositionAttitudeTransform* pat2;
osg::PositionAttitudeTransform* pat2Right;
osg::PositionAttitudeTransform* pat_PR;
osg::PositionAttitudeTransform* pat_PR_active;
osg::PositionAttitudeTransform* pat_PR_activeTest;
osg::PositionAttitudeTransform* pat_PRRight;
osg::PositionAttitudeTransform* pat_PR_activeRight;
osg::PositionAttitudeTransform* pat_eye_line;
osg::PositionAttitudeTransform* pat_eye_line_t;
osg::PositionAttitudeTransform* pat_eye_lineRight;
osg::PositionAttitudeTransform* pat_raytrace_lines;

//#include "Keras\keras_model.h"

keras::KerasModel* mRayKeras;
keras::DataChunk* sampleRayKeras;
keras::KerasModel* mRayKerasRight;
keras::DataChunk* sampleRayKerasRight;

keras::KerasModel* mPupilKeras;
keras::DataChunk* samplePupilKeras;

keras::KerasModel* mLensKeras;
keras::DataChunk* sampleLensKeras;

/***************************************************************************/

//#define CONTROL_LEGS

//#define SHOW_CORNEA_SPHERES
#define SHOW_EYELINES

//#define ONLINE_PUPIL
//#define ONLINE_LENS
//#define ONLINE_FOVEATION
//#define ONLINE_MOTOR
//#define LENS_VIDEO

#define ONLINE_EXE
//#define DELTA_ONV
//string snnPrefix = "C:\\Users\\taasi\\Desktop\\RunSNN\\files";
//ofstream fonvOut(snnPrefix + "\\onvOut.csv", std::fstream::trunc);
//ifstream fanglesOut("test.csv", std::fstream::trunc);

#define RECORD_ANGLES
ofstream frecordAngles("./angleRecordings/actual_smooth_lateral.csv");
ofstream frecordAngles1("./angleRecordings/smooth_LCNSpikingHybrid_delta_100epoch_k25_L1.csv");

//#define SHOW_RAYS
bool show_forward_trace_rays;
bool show_reverse_trace_rays;
bool show_full_ball_trace;
double full_ball_alpha_channel;
double forward_rays_alpha_channel;
double reverse_rays_alpha_channel;
int photoreceptor_to_trace_rho;
int photoreceptor_to_trace_theta;
//#define SHOW_LENS_SPHERES

//#define OLD_RETINA

#ifdef OLD_RETINA
#define RAYTRACE_WIDTH 40
#define RAYTRACE_HEIGHT 90
#else
#define RAYTRACE_WIDTH 40
#define RAYTRACE_HEIGHT 360
#endif

#define MIN_PUPIL_SIZE 2.0
#define MAX_PUPIL_SIZE 3.75

#define MIN_LENS_WIDTH 1.0
#define MAX_LENS_WIDTH 4.84

//#define RECORD_PUPIL
//#define TEST_PUPIL
bool start_recording_pupil;
bool final_record_iteration_pupil;
bool oscillatePupilSphincterActivation;
bool pupilMovementDemoComplete;

//#define UNIFORM_LENS
//#define RECORD_LENS
//#define TEST_LENS
bool start_recording_lens;
bool record_lens_data;
bool final_record_iteration_lens;
double lens_error;
Vec3f_local stored_focal_point;
osg::Vec3 lfpd;
bool holdBallPosition;
bool targetMoveOverride;
bool oscillateLensCilliaryActivation;
bool lensMovementDemoComplete;

#define RECORD_FOVEATION
//#define TEST_FOVEATION
bool start_recording_foveation;

//#define TEST_MOTOR

double pupil_adjustment_rate = 0.04;
double pupil_size = 0;
double pupil_utility = 0.0;
double pupil_utility_max = 3.0 * RAYTRACE_WIDTH * RAYTRACE_HEIGHT;
double prior_pupil_size_for_training = 0.0;
int pupil_adjustment_direction = 0;
bool pupil_adjusted = false;

//int lens_adjustment_direction = 0;
double lens_adjustment = 0;
bool fatten_lens = true;
bool lens_adjusted = false;

//#define FULL_LENS_DATASET_GENERATION
//#define FULL_FOVEATION_DATASET_GENERATION

#if defined(FULL_LENS_DATASET_GENERATION) || defined(FULL_FOVEATION_DATASET_GENERATION)
string dirPrefix = "E:\\biomechanical_eye_training_dataset";
#else
string dirPrefix = ".";
#endif

ofstream finputpupil(dirPrefix + "\\pupilData\\image_x_act_iris_sphincter.csv", ios::app);
ofstream finputpupilact(dirPrefix + "\\pupilData\\iris_sphincter_act.csv", ios::app);
ofstream foutputpupil(dirPrefix + "\\pupilData\\output_iris_sphincter_delta_act.csv", ios::app);
ofstream foutputpupilballpos(dirPrefix + "\\pupilData\\ballpos.csv", ios::app);
ofstream foutputpupilrandomlensact(dirPrefix + "\\pupilData\\randomlensact.csv", ios::app);
ofstream foutputpupilutility(dirPrefix + "\\pupilData\\utility.csv", ios::app);
ofstream foutputpupilsize(dirPrefix + "\\pupilData\\size.csv", ios::app);

ofstream finputlens(dirPrefix + "\\lensData\\image_x_act_cilliary.csv", ios::app);
ofstream finputlensact(dirPrefix + "\\lensData\\cilliary_act.csv", ios::app);
ofstream foutputlens(dirPrefix + "\\lensData\\output_lens_cilliary_delta_act.csv", ios::app);
ofstream foutputlensballpos(dirPrefix + "\\lensData\\ballpos.csv", ios::app);
ofstream foutputpupilrandompupilact(dirPrefix + "\\lensData\\randompupilact.csv", ios::app);
ofstream foutputlenssize(dirPrefix + "\\lensData\\size.csv", ios::app);
ofstream finputlensmaybeballpos(dirPrefix + "\\lensData\\maybeballpos.csv", ios::app);

ofstream finputperception(dirPrefix + "\\perceptionData\\image_x.csv", ios::app);
ofstream foutputperception(dirPrefix + "\\perceptionData\\output_etheta_ephi.csv", ios::app);
ofstream foutputperceptionballpos(dirPrefix + "\\perceptionData\\ballpos.csv", ios::app);
ofstream foutputpupilrandompupilactperception(dirPrefix + "\\perceptionData\\randompupilact.csv", ios::app);
ofstream foutputpupilrandomlensactperception(dirPrefix + "\\perceptionData\\randomlensact.csv", ios::app);

ofstream fdistoutput("eyeDist.csv", std::ofstream::out);

#ifdef CONTROL_LEGS
//#define SPHERE_SIZE 290
#define SPHERE_SIZE 100
#else
//#define SPHERE_SIZE 15
#define SPHERE_SIZE 50
//#define SPHERE_SIZE 10
#endif
#define M_PI 3.141592653589793
#define INFINITY 1e8

double meanValueRight[2];
double stdValueRight[2];
double meanValueLeft[2];
double stdValueLeft[2];

double meanValuePupil[1];
double stdValuePupil[1];

double meanValueLens[1];
double stdValueLens[1];

double raytrace_grid_points[4091][3];

/***************************************************************************/

using namespace osg;

#define EYE_SIZE 16
#define EYE_DIFF_X 35 
#define EYE_DIFF_Y 95 
#define EYE_DIFF_Z 57

#ifndef sqr
#define	sqr(X)		(X)*(X)
#endif

bool leye_IK_solved;
bool leye_muscle_solved;
double eye_cxl, eye_cyl, eye_cxr, eye_cyr;
double idcxl, idcyl;

ofstream fleft;
ofstream fright;

int raytrace_counter = 0;
ofstream fraytrace;
int raytrace_counter_right = 0;
ofstream fraytrace_right;

Group* eyeLineGroup = NULL;			// root node of scenegra
Group* eyeLineGroupRight = NULL;			// root node of scenegraph
Group* raytraceLineGroup = NULL;			// root node of scenegra
Group* focalPointLineGroup = NULL;			// root node of scenegra

Group* activeCellGroup = NULL;			// root node of scenegraph
vector<int> active_idx_pupil;
vector<int> active_idx_lens;
vector<int> active_idx_foveation;
int windowPhotoReceptorLeft, windowPhotoReceptorLeftTest;

gReal* retina_distribution_noise;

float** scanpath;

int counter_total = 4;
int counter_start_L = 1;
double  tyl_temp, txl_temp;

Box_local eyeTraceHitBox;
bool firstTimeHit;

vector<Vec3f_local> active_idx_color_pupil;
vector<Vec3f_local> active_idx_color_lens;
vector<Vec3f_local> active_idx_color_foveation;

/***************************************************************************/

double lens_ec, lens_distance;

double prev_lx, prev_ly, prev_lz;
double prev_rx, prev_ry, prev_rz;

int lastStoredFrame = 0;
int preKinE = 0;
int theta = 360;
int phi = 0;
#define CONTINUOUS_CONTROL
bool bHit = false;
int countAfterHit = 0;

int countAfterActivation = 0;

#ifdef CONTROL_LEGS
int leftOrRight = 2;
#else
int leftOrRight = 0;
#endif

double angleInY = 30;
double sphereRadius = 700;
double angleInX = 30;
int deltaX = 10;
int deltaY = 10;
int deltaZ = 10;
int startX = 700;
int startY = 110;
int startZ = -70;
int endX = 740;
int endY = 180;
int endZ = 20;
int sizeX = (endX - startX) / deltaX;
int sizeY = (endY - startY) / deltaY;
int sizeZ = (endZ - startZ) / deltaZ;

int startPhi = 0;
int endPhi = 180;
int deltaPhi = 30;
int sizePhi = (endPhi - startPhi) / deltaPhi;
int startTheta = 0;
int endTheta = 360;
int deltaTheta = 30;
int sizeTheta = (endTheta - startTheta) / deltaTheta;
int x = startX;
int y = startY;
int z = startZ;
int angle1Z = 0;
int angle1Y = -20;
int angle1X = 0;
int angle2 = 1;
bool tooBig = false;

int rX = 0;
int rY = 10;
int rZ = 360;

float magnitudeTraining = 0;

#endif
