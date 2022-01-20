//following is the animation cases
//1. linear target following
//2. one point target reaching
//3. Instant target reaching
//4. Ball catching
//5. Release arm once and recover the original posture

//#include <opencv2/highgui/highgui.hpp>
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
extern gVec3 GRAVITY_VECTOR(0,0,-9810.); // mm/sec^2, -Z						// I stop the Gravity fow now. It is too hard to compute muscle dynamicswith gravity
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
char*		snCw=0;
integer*	snIw=0;
doublereal* snRw=0;
integer		snLenCw =  8*500*20;
integer		snLenIw =  30000*20;
integer		snLenRw = 140000*20;

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
void exportReverseTraceBuffer();

//PCA
GM* U_t = NULL;	//principal components
GV* mean_t = NULL; // mean

//mocap
bool useMocap =  false;
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
typedef enum {eRotate, eTrans, eZoom} _MouseMode;
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
const gReal displayTimeStep = 1./100;
//const gReal simulationTimeStep = 0.001;				//FOR backward Euler method
//const gReal simulationTimeStep = 1./100;				//FOR backward Euler method we need this smaller time step when we have neck
const gReal simulationTimeStep = 1./1000;				//FOR backward Euler method we need this smaller time step when we have neck
//gReal simulationTimeStep =  0.02;				//FOR backward Euler method
gReal simulationTimeStepExplicitEuler = 1.14e-6;//FOR forward Euler method
gVec3 targeting_location(0,0,0);

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
int frame=0;
int breatheFrame = 0;

//global variables
FILE* fpT=0;
FILE* fpPt=0;
FILE* fpPh=0;
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
gRigidBody* Manipulator=NULL;
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

void muscle_tuning();
void test_IK();
void doFreeFall(gReal timestep);
void doTorqueDrivenHomePosition();
void doKinematicMocapAnimation();
void animate2();
void compute_rotationalInertia(gReal density);
void compute_rotationalInertiaFromBoundingBox(gReal density);
void addRibSprings(gReal ks, gReal kd);
void report_mass();
void translate_jointAnglesFromMocap();
void apply_mocapJointAngles();
//void makeSittingInitialPose();
void doTorqueDrivenMocapTrack(gReal time, gReal interval, int incr);
void store_oldJointAnglesMocap();
void compute_velocityMocap(gReal);
void breathing(gReal scale, gReal time);
void setDesiredAccRib(gReal scale, gReal time);
void setDesiredAccHNTAL(gReal h, gReal gamma);
void doRandomPoseAnimationFromMocap();
void applyActivationDynamics(gReal t);
void generateTrainingData(gReal num);
void doPoseBasedMocapFollow2(gReal time,gReal coactivation, gReal interval, int incr, bool computeMuscleForce);
void doPoseBasedMocapFollow3(gReal time, gReal coact, gReal interval, int incr, bool computeMuscleForce);
void doPoseBasedTargetFollow(gReal time, gReal coact, gReal interval, int incr, bool computeMuscleForce, double* target);
void doPoseBasedTargetFollowFor
	(gReal time, gReal coact, gReal interval, int incr, double* target);
void doPoseBasedTargetInverseDynamicsControl(gReal time, gReal coact, gReal interval, int incr, double* target);
void scenarioCall();
void setJointStiffnessAndDamping();
void setIntialPoseGradually();
int adjust_muscle_weights();
int adjust_muscle_pcsa();
void finish();
void SymmetrizeWeightFile(const char* filename, double max_, bool selectMax);
void computeDesiredMuscleState(GV* srcAcc, gReal timestep);
void makeIndexHand();
void makeFist();
void makeFistRight();
void prepareRecord();
void setRandomTargetJointAngle();
int doRandomAnimation();
void sol_momentArmMatrixForBone_Sparse();
void next_stepNewmarkNumerical(gReal h);
int doRandomAnimationMD();
void init_feedback_gain_tuning();
void sol_feedback_gain_optimization();
void feedback_gain_update();
void makeInitialPoseGradually();
void multDiagMat(GM* dst, double* diag, const GM* src);
void update_PFS(void);
void update_PF_t();
void update_PF_h();
void update_PF_ub_Leye();
void reportMuscle();
void writeActivation(int frame,FILE* fp,gReal up);
void writeBallPoses(int frame, FILE* fp);
void printStiffnessMatrix(const char* filename);
void doCollectPoseData(gReal);
void poseDataAnalysis();
void testPrincipalComponentTrunk();
void doFeedbackMuscleControl(gReal time, gReal interval, int incr);
void setTrunkVertebraJointAngles(gReal pcx, gReal pcy, gReal pcz);
void test_moment_arm_matrix();
/****added by masaki*********************************************************/
void testRotateTrunk(); //added by masaki as testing function of control 120410
void testArm();//added by masaki as testing function of control 120411
void testPose();
void kinematicMotion();
void testArm2();
void doTargetMotionMuscleControl();
void setMyTargetJointAngle();
void setMyTargetJointAngleWithoutLarm();
double phi_vec_conversion(gVec3 v);
double theta_vec_conversion(gVec3 v);
void resetToInitialPoseLeye();

bool setTargetJointAngleLefArm(double *v);
bool setTargetJointAngleRightArm(double *v);
bool setTargetJointAngleLefLeg(double *v);
bool setTargetJointAngleRigLeg(double *v);

void doLeftArmTargetMuscleControl(gReal time, gReal coact, gReal interval, bool computeMuscleForce);
void calculateMuscleForce();
GV* calculateMuscleForce2();
gVec3 doFullBodyMuscleControlInveseDynamics(gReal coact, bool computeMuscleForce, gVec3 v);

gVec3 initial_pos_head, initial_pos_trunk, initial_pos_larm;//, initial_pos_rarm;
gVec3 get_currentLeftAmrPos();
gVec3 get_currentLeftAmrPos_no_print();
gVec3 computeTargetPos2(double* target, gVec3 cur_pos);
void generateTrainingDataFullBody();
void generateTrainingDataFullBody2();
void generateTrainingDataFullBody3();
void generateTrainingDataFullBody2015();
#define DEG_TO_RAD	1.74532925199432957692369076848e-2	//(M_PI/180.0)
#define	D2R			DEG_TO_RAD	
FILE* fbody = 0;//masaki
FILE* fbody_input = 0;//masaki
void doFullBodyMuscleControlInveseDynamicsForGeneratingTrainingSet(gReal coact, bool computeMuscleForce, gVec3 v);
void doFullBodyMuscleControlInveseDynamicsForGeneratingTrainingSet2(gReal coact, bool computeMuscleForce, gVec3 v, GV* values);
void doFullBodyMuscleControlInveseDynamicsForGeneratingTrainingSet3(gReal coact, bool computeMuscleForce, gVec3 v, GV* values);
const std::string currentDateTime();
void doFullBodyMuscleControlDNN(gVec3 v);
void doFullBodyMuscleControlDNN2(gVec3 v);
void testWithLearnedActivationSignals();
void doFullBodyMuscleControlInverseDynamics(gVec3 v);
gVec3 computeTargetUnitVec(double* target, gVec3 cur_pos);

static gReal randomNumber(gReal low, gReal high);

//this is new functions March 2015
void generateTrainingDatasetByPoseBasedTargetInverseDynamics();
void generateTrainingDatasetByPoseBasedTargetInverseDynamicsWithDisplay();
void generateTrainingDatasetPupil();
void generateTrainingDatasetLens();
void generateTrainingDatasetFoveation();
void generateTrainingDatasetMotor();
void generateTrainingDatasetMotor_fixaton_dataset();
void generateTrainingDatasetMotorWithDisplay();
void generateTrainingDatasetMotorWithDisplay_fixaton();
void testLoopKinematicAndDNNFixatonsDisplay();
void correctAndSetPosturekinematically();
void doFullBodyMuscleControlDNN3(gVec3 v);

//May 2016d
void doOneStepMuscleDrivenControl(gReal time, gReal coact, gReal interval, int incr, ofstream& foutput, ofstream& finput, bool generatingTrainingDataset);
void doOneStepMuscleDrivenControlGeneratingTrainingDataset(gReal time, gReal coact, gReal interval, int incr, ofstream& foutput, ofstream& finput, bool generatingTrainingDataset);
void doOneStepMuscleDrivenControlGeneratingTrainingDatasetLeye(gReal time, gReal coact, gReal interval, int incr, ofstream& foutput, ofstream& finput, bool generatingTrainingDataset);

Sda_Decoder	*sda;
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

void testSavedMuscleActivation();
float muscle_activation_data[3000][29];

long double muscle_activation_data_delta_leye[5000][6];
long double muscle_activation_data_leye[5000][9];
double resets_leye[5000];

double muscle_activation_data_delta_leye_pupil[1500];
double ballpos_leye_pupil[1500][4];
double random_lens_cilliary_act[1500];
double image_leye_pupil[40*360];
double muscle_activation_data_leye_pupil_start;

double muscle_activation_data_delta_leye_lens[1500];
double ballpos_leye_lens[1500][4];
double random_pupil_sphincter_act[1500];
double image_leye_lens[40*360];
double muscle_activation_data_leye_lens_start;

double image_leye_perception[40*360];
double etheta_ephi_perception[1500][2];
double random_pupil_sphincter_act_foveation[1500];
double random_lens_cilliary_act_foveation[1500];
double ballpos_leye_foveation[1500][4];

/***************************************************************************/

#include "Keras\keras_model.h"
#include <iterator>

using namespace keras;

void resetToInitialPose();

KerasModel *m;
KerasModel *mLEye;

void setRandomPupilAndLensLengths();

void doFullBodyMuscleControlKeras(gVec3 v);

void doForwardDynamicMuscleControlLeye(int muscle_index, double actLevel);
void doInverseDynamicMuscleControlLeye(gVec3 v);
void doInverseDynamicMuscleControlLeyeNew(gVec3 v);
void doInverseDynamicMuscleControlLeyeNew_fixaton(gVec3 v, float time);
void doInverseKinematicsControlLeyeNew_fixaton(gVec3 v, float time);
void doInverseDynamicMuscleControlLeyeAndOutputDataThatIsLessExtensiveThanTrainingDataButStillUsefulForPlotting(gVec3 v);
void doMuscleControlKerasLEye(gVec3 v);
void testGeneratedDatasetFromTrainingLeyeMotor();
void testGeneratedDatasetFromTrainingLeyePupil();
void testGeneratedDatasetFromTrainingLeyeLens();
void testGeneratedDatasetFromTrainingLeyeFoveation();

void doFullBodyMuscleControlKeras58(gVec3 v);

double meanValueLEye[NUM_MUSCLES_LEYE];
double stdValueLEye[NUM_MUSCLES_LEYE];

DataChunkFlat *sampleLEye;

void doOneStepPoseBasedTargetInverseDynamicsControlForGenerateTrainingDatasetIntergrateToNextTimestepChangingMagnitudeLeye(gReal coact, gReal interval, int incr, ofstream& foutput, ofstream& foutputDelta, ofstream& finput, bool generatingTrainingDataset, gVec3 target);

void computeThetaPhi();
gVec3 computeTargetUnitVecRegularized(double* target, gVec3 cur_pos);

void setLarmJointAngle();

void reflexTest(gVec3 v);

void computeDesiredMuscleState3(gVec3 vec, gVec3 cur, gReal timestep);

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

void create_visual_target(void);
void create_visual_target2(void);
void create_bounce_board(void);
void create_eye_ball_left(void);
void re_create_eye_ball_left(void);
void create_eye_ball_right(void);
void move_draw(gVec3 prev, gVec3 after);
void move_visual_target(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_eye_training(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_saccade_fixed(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_smooth_pursuit(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_smooth_lens_adjustment(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_smooth_pursuit_lateral(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_oscillate_straight(osg::MatrixTransform* trans_visual_target, double time);
bool move_visual_target_ball_thrown(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_ball_straight(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_pupil_training(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_foveation_training(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_lens_training(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_lens_training_new(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_lens_online_fixed_positions(osg::MatrixTransform* trans_visual_target, double time);
bool move_visual_target_fixed_position(osg::MatrixTransform* trans_visual_target, double time);

bool new_target_ball;

gVec3 getcurrent_ori();
gVec3 getcurrent_ori_leye();

void adjust_pupil_sphincter_muscles();
void compute_pupil_size();

void adjust_lens_cilliary_muscles();
void compute_lens_size();
void maybeRecordBallPos();
Vec3f_local compute_focal_point();

gVec3 prev_pos, after_pos;

void computeDesiredMuscleState3LEye(gVec3 vec, gVec3 cur, gReal timestep);
gVec3 *SkinVertices;
osg::PositionAttitudeTransform* MeshTransform;
osg::ref_ptr<osg::Node> HumanMesh;
osg::StateSet *stateset;
gSystemMeshLoader MeshObj;
void UpdateSimpleVertex(gReal elapsed_time);

void drawTargetBox();
void setCameraParameter();

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

#pragma comment(linker, "/STACK:1500000")
#pragma comment(linker, "/HEAP:1500000")

#include "Keras\keras_model.h"
keras::KerasModel *mRayKeras;
keras::DataChunk *sampleRayKeras;
keras::KerasModel *mRayKerasRight;
keras::DataChunk *sampleRayKerasRight;

keras::KerasModel *mPupilKeras;
keras::DataChunk *samplePupilKeras;

keras::KerasModel *mLensKeras;
keras::DataChunk *sampleLensKeras;

//#define CONTROL_LEGS

//#define SHOW_CORNEA_SPHERES
#define SHOW_EYELINES

//#define ONLINE_PUPIL
//#define ONLINE_LENS
//#define ONLINE_FOVEATION
#define ONLINE_MOTOR
//#define LENS_VIDEO

#define SHOW_RAYS
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

#define UNIFORM_LENS
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

//#define RECORD_FOVEATION
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

using namespace osg;
#include <iostream>
//using namespace cv;
//#include "Perception\target_detection.h"

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

#include <algorithm>    // std::min


#include <fstream>
int raytrace_counter = 0;
ofstream fraytrace;
int raytrace_counter_right = 0;
ofstream fraytrace_right;

Group					*eyeLineGroup = NULL;			// root node of scenegra
Group					*eyeLineGroupRight = NULL;			// root node of scenegraph
Group					*raytraceLineGroup = NULL;			// root node of scenegra
Group					*focalPointLineGroup = NULL;			// root node of scenegra

Group					*activeCellGroup = NULL;			// root node of scenegraph
vector<int> active_idx_pupil;
vector<int> active_idx_lens;
vector<int> active_idx_foveation;
int windowPhotoReceptorLeft, windowPhotoReceptorLeftTest;

static gReal randomRange(gReal low, gReal up);

gReal *retina_distribution_noise;

float** scanpath;
//#define PERLIN_TARGET_LOCATION

#ifdef PERLIN_TARGET_LOCATION

#include <Perlin\Perlin.h>
Perlin plarm;
Perlin prarm;
Perlin plleg;
Perlin prleg;

#endif

int counter_total = 4;
int counter_start_L = 1;
double  tyl_temp, txl_temp;

Box_local eyeTraceHitBox;
bool firstTimeHit;

vector<Vec3f_local> active_idx_color_pupil;
vector<Vec3f_local> active_idx_color_lens;
vector<Vec3f_local> active_idx_color_foveation;

//[comment]
// This variable controls the maximum recursion depth
//[/comment]
#define MAX_RAY_DEPTH 4

float mix(const float &a, const float &b, const float &mix)
{
    return b * mix + a * (1 - mix);
}

//[comment]
// This is the main trace function. It takes a ray as argument (defined by its origin
// and direction). We test if this ray intersects any of the geometry in the scene.
// If the ray intersects an object, we compute the intersection point, the normal
// at the intersection point, and shade this point using this information.
// Shading depends on the surface property (is it transparent, reflective, diffuse).
// The function returns a color for the ray. If the ray intersects an object that
// is the color of the object at the intersection point, otherwise it returns
// the background color.
//[/comment]
Vec3f_local trace(
    const Vec3f_local &rayorig,
    Vec3f_local &raydir,
    std::vector<Sphere_local> &Sphere_locals,
    const int &depth,
	int filterOption=0) // filter options: 0 = no filter, 1 = binary filter, 2 = inverse dot filter
{
	Vec3f_local tnear = NULL;
	vector<Vec3f_local> tnears;
    //const Sphere_local* Sl = NULL;
	Sphere_local Sltemp;
	vector<Sphere_local*> Sls;
    // find intersection of this ray with the Sphere_local in the scene
	int num_intersections = 0;
	bool atLeastOneHit = false;
    for (unsigned i = 0; i < Sphere_locals.size(); ++i) {
		Vec3f_local near_int, far_int;
		if (Sphere_locals[i].intersect(rayorig, raydir, near_int, far_int)) {
			//tnear = near_int;
			tnears.push_back(near_int);
			Sls.push_back(&Sphere_locals[i]);
			num_intersections++;
			atLeastOneHit = true;
        }
    }

	Vec3f_local bgCol(sceneView->getClearColor().x(), sceneView->getClearColor().y(), sceneView->getClearColor().z());
//	Vec3f_local bgCol(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec);
	Vec3f_local retVal;
	if (!atLeastOneHit) {
		switch(filterOption) {
			case 0:
				retVal = bgCol;
				break;
			default:
				retVal = Vec3f_local(0,0,0);
				break;
		}
		//return bgCol;
		return retVal;
	}

	Vec3f_local phit = 0;
    Sphere_local* Sl = NULL;
	if(num_intersections > 1) { // multiple spheres were hit- take only closest one
		double min_dist = 10000000000000;
		int closestIndex = 0;
		for(int i = 0; i < tnears.size(); i++) {
			Vec3f_local tn = tnears.at(i);
			double dist = (tnears.at(i) - rayorig).length();
			if(dist < min_dist) {
				closestIndex = i;
				min_dist = dist;
			}
		}
		phit = tnears.at(closestIndex);
		Sl = Sls.at(closestIndex);
	}
	else {
		phit = tnears.at(0);
		Sl = Sls.at(0);
	}
	
    Vec3f_local nhit = phit - Sl->center; // normal at the intersection point
    nhit.normalize(); // normalize normal direction

	float facingratio = -raydir.dot(nhit);
	/*Vec3f_local hitCol = Sl->emissionColor * facingratio;
	Vec3f_local surfaceColor = hitCol;*/

	switch(filterOption) {
		case 1:
			retVal = Vec3f_local(1.0,1.0,1.0);
			break;
		case 2:
			retVal = Vec3f_local(1.0,1.0,1.0) * 10.0 * exp(-facingratio*10.0);
			break;
		default:
			retVal = Sl->emissionColor * facingratio;
			break;
	}
    //return surfaceColor;
	return retVal;
}

//[comment]
// Main rendering function. We compute a camera ray for each pixel of the image
// trace it and return a color. If the ray hits a Sphere_local, we return the color of the
// Sphere_local at the intersection point, else we return the background color.
//[/comment]


//log polar mapping


void render(std::vector<Sphere_local> &Sphere_locals, Vec3f_local ray_orig, Vec3f_local ray_dir, Vec3f_local* returnedImage, int side)
{
    //unsigned width = wind[MAIN_WINDOW].width, height = wind[MAIN_WINDOW].height;
    unsigned width =RAYTRACE_WIDTH, height = RAYTRACE_HEIGHT;
    Vec3f_local *image = new Vec3f_local[width * height], *pixel = image;
    float invWidth = 1 / float(width), invHeight = 1 / float(height);
    //float fov = 30, aspectratio = width / float(height);

	
#ifdef CONTROL_LEGS
	
    float fov = 240, aspectratio = width / float(height);
#else
    float fov = 240, aspectratio = width / float(height);
#endif
    float angle = tan(M_PI * 0.5 * fov / 180.);

	//new Ray Trace grids

	/*
	1. get eye center.
	2. get eye tip (front)
	3. compute the vector. 
	4. get radius and use that to estimate the projection plane at the back of the eye
	5. use the projected point as the center of the raytracer grid. take constant to -x and +x, -y, +y. 
	6. rotate the raytracer projection point with the current eye orientation
	7. compute the direction of the ray by subtracting each raytracer grid pos from the eye tip pos 
	8. use it in *pixel = trace(ray_orig, raydir, Sphere_locals, 0);
	*/
	if(side==0)
	{

		osg::Vec3 osg_eye_center(ray_orig.x,ray_orig.y,ray_orig.z);

		double xangle, yangle;
		xangle = ray_dir.x;
		yangle = ray_dir.z;

		double phi = xangle;
		double theta   = yangle;

		
		osg::Vec3 eye_direction_vec(0,-1,0);
		osg::Quat q2(phi, osg::X_AXIS, 0, osg::Y_AXIS,theta, osg::Z_AXIS);
		osg::Vec3 osg_eye_temp2 = q2*(eye_direction_vec);

#ifdef SHOW_RAYS
		//draw ray trace lines
		osg::Vec3Array* vertices_eye = new osg::Vec3Array;
		osg::Vec4Array* colors_eye = new osg::Vec4Array;
		vertices_eye->setName("VertexRayTrace");
#endif

		osg::Vec3 rotated_eye_dir;

		double ray_width = RAYTRACE_WIDTH;
		double maxRadius = 2.0/3.0*std::exp(ray_width/5.0);
		//5
		
		//#pragma omp parallel for num_threads(8) 
		for (int rho = 0; rho < RAYTRACE_WIDTH; rho++) {
			for (unsigned thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT*4; thetaLogPolar+=4, ++pixel) {
				
				double logPolarX =2.0* std::exp(double(rho/5.0)) * cos(double(thetaLogPolar*DEG_TO_RAD));
				double logPolarY =2.0* std::exp(double(rho/5.0)) * sin(double(thetaLogPolar*DEG_TO_RAD));

				float yy = (1 * ((logPolarX) /maxRadius)) * fov*D2R;
				float xx = (1 * ((logPolarY) /maxRadius)) * fov*D2R;

				osg::Vec3 eye_direction_vec3(0,1,0);
				osg::Quat q3(xx, osg::X_AXIS, 0, osg::Y_AXIS,yy, osg::Z_AXIS);
				osg::Vec3 osg_eye_temp3 = q3*(eye_direction_vec3);

				osg_eye_temp3.normalize();
				osg_eye_temp2.normalize();
				
				osg::Vec3 raytrace_orig(osg_eye_center.x()+logPolarX/maxRadius*EYE_SIZE/2, osg_eye_center.y()+EYE_SIZE/2, osg_eye_center.z()+logPolarY/maxRadius*EYE_SIZE/2);
				Vec3f_local raytrace_orig_local(raytrace_orig.x(), raytrace_orig.y(), raytrace_orig.z());
				osg::Vec3 pin_hole_pos = osg_eye_center + osg::Vec3(0,-1,0)*EYE_SIZE/2;

				osg::Vec3 raydir = pin_hole_pos - raytrace_orig;
				raydir = q2*raydir;
				raydir.normalize();				
				Vec3f_local raydir_local(raydir.x(), raydir.y(),raydir.z());
				*pixel = trace(raytrace_orig_local, raydir_local, Sphere_locals, 0);

			}
		}
		
#ifdef SHOW_RAYS
		osg::Vec3 up(0,0,1);
		osg::Vec3 pCenter(SysLEye->pupil.center.x,SysLEye->pupil.center.y-2,SysLEye->pupil.center.z);
		double rad = SysLEye->pupil.radius;

		for(int i = 0; i < 4; i++) 
		{
			osg::Quat q(0, osg::X_AXIS, 45*i*gDTR, osg::Y_AXIS, 0, osg::Z_AXIS);
			osg::Vec3 fin_dir = q*(up);
			osg::Vec3 p1 = pCenter + fin_dir*rad;
			osg::Vec3 p2 = pCenter - fin_dir*rad;

			vertices_eye->push_back(p1);
			vertices_eye->push_back(p2);
			colors_eye->push_back(osg::Vec4(255/255.0,0/255.0,0/255.0,1));
			colors_eye->push_back(osg::Vec4(255/255.0,0/255.0,0/255.0,1));
		}

		vertices_eye->push_back(osg_eye_center+osg_eye_temp2*3000);
		vertices_eye->push_back(osg_eye_center-osg_eye_temp2*3000);
		colors_eye->push_back(osg::Vec4(206/255.0,206/255.0,206/255.0,1));
		colors_eye->push_back(osg::Vec4(206/255.0,206/255.0,206/255.0,1));


		//draw ray trace lines
		osg::ref_ptr<osg::Geometry> geom_active = new osg::Geometry;
		geom_active->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, vertices_eye->size()));
		geom_active->setUseDisplayList(false);

		geom_active->setVertexArray(vertices_eye);
		geom_active->setColorArray(colors_eye);
		geom_active->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

		osg::ref_ptr<osg::Geode> geode_active = new osg::Geode;
		geode_active->addDrawable(geom_active.get());

		osg::StateSet* state_active = geode_active->getOrCreateStateSet();
		state_active->setMode(GL_LIGHTING, osg::StateAttribute::ON);
		state_active->setMode(GL_BLEND, osg::StateAttribute::ON);
		state_active->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
		osg::LineWidth* lw_active = new osg::LineWidth;
		lw_active->setWidth(3);
		//osg::Point *point_size = new osg::Point(6.0f);
		state_active->setAttribute(lw_active, osg::StateAttribute::ON);
		osg::BlendFunc* blendfunc_active = new osg::BlendFunc();
		state_active->setAttributeAndModes(blendfunc_active, osg::StateAttribute::ON);
		pat_raytrace_lines = new osg::PositionAttitudeTransform();
		pat_raytrace_lines->addChild(geode_active);

		pat_raytrace_lines->setPosition(osg::Vec3d(0.0, 0, 0.0));

		if(raytraceLineGroup->getNumChildren()>0)
			raytraceLineGroup->removeChild(0,1);
		raytraceLineGroup->addChild(pat_raytrace_lines);
#endif
    }
	else{
		osg::Vec3 osg_eye_center(ray_orig.x,ray_orig.y,ray_orig.z);

	
		double cxr, cyr;
		
		double xangle, yangle;
		xangle = ray_dir.x;
		yangle = ray_dir.z;

		double theta = xangle;
		double phi   = yangle;
		
		
		//osg::Vec3 osg_eye_temp2(cos(theta)*sin(phi), -cos(theta)*cos(phi), sin(theta));
		osg::Vec3 eye_direction_vec(0,-1,0);
		osg::Quat q2(theta, osg::X_AXIS, 0, osg::Y_AXIS, phi, osg::Z_AXIS);
		osg::Vec3 osg_eye_temp2 = q2*(eye_direction_vec);

		osg::Vec3 rotated_eye_dir;

		//double pho = character[0]->head->eyes->get_eye_radius_Z();
		double ray_width = RAYTRACE_WIDTH;
		double maxRadius = std::exp(ray_width);
		//5
		
		//#pragma omp parallel for num_threads(8) 
		for (int rho = 0; rho < RAYTRACE_WIDTH; rho++) {
			for (unsigned thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT*4; thetaLogPolar+=4, ++pixel) {
						
				double logPolarX = std::exp(double(rho)) * cos(double(thetaLogPolar*DEG_TO_RAD));
				double logPolarY = std::exp(double(rho)) * sin(double(thetaLogPolar*DEG_TO_RAD));

				float yy = (1 * ((logPolarX) /maxRadius)) * fov*D2R;
				float xx = (1 * ((logPolarY) /maxRadius)) * fov*D2R;

				osg::Vec3 eye_direction_vec3(0,1,0);
				osg::Quat q3(theta+xx, osg::X_AXIS, 0, osg::Y_AXIS,phi+yy, osg::Z_AXIS);
				osg::Vec3 osg_eye_temp3 = q3*(eye_direction_vec3);

				osg_eye_temp3.normalize();
				osg_eye_temp2.normalize();
				osg::Vec3 raytrace_orig(osg_eye_center.x()+logPolarX/maxRadius*EYE_SIZE/2, osg_eye_center.y()+EYE_SIZE/2, osg_eye_center.z()+logPolarY/maxRadius*EYE_SIZE/2);
				osg::Vec3 pin_hole_pos = osg_eye_center + osg::Vec3(0,-1,0)*EYE_SIZE/2;

				
				Vec3f_local raytrace_orig_local(raytrace_orig.x(), raytrace_orig.y(), raytrace_orig.z());
				osg::Vec3 raydir = pin_hole_pos - raytrace_orig;
				raydir = q2*raydir;
				raydir.normalize();				
				Vec3f_local raydir_local(raydir.x(), raydir.y(),raydir.z());

				*pixel = trace(raytrace_orig_local, raydir_local, Sphere_locals, 0);
			}
		}
	}

	compute_pupil_size();
	compute_lens_size();
	SysLEye->adjust_pupil(pupil_size);
	SysLEye->adjust_cornea();
	
	for (unsigned i = 0; i < RAYTRACE_WIDTH * RAYTRACE_HEIGHT; i++) {
		returnedImage[i].x = min(float(1), image[i].x);
		returnedImage[i].y = min(float(1), image[i].y);
		returnedImage[i].z = min(float(1), image[i].z);
	}
    delete [] image;
}

void render_ultimate(std::vector<Sphere_local> &Sphere_locals, Vec3f_local ray_orig, Vec3f_local ray_dir, Vec3f_local* returnedImagePupil, Vec3f_local* returnedImageLens, Vec3f_local* returnedImageFoveation, int side)
{
    unsigned width =RAYTRACE_WIDTH, height = RAYTRACE_HEIGHT;

    Vec3f_local *imagePupil = new Vec3f_local[width * height], *pixelPupil = imagePupil;
	Vec3f_local *imageLens = new Vec3f_local[width * height], *pixelLens = imageLens;
	Vec3f_local *imageFoveation = new Vec3f_local[width * height], *pixelFoveation = imageFoveation;

    float invWidth = 1 / float(width), invHeight = 1 / float(height);
	
#ifdef CONTROL_LEGS
	
    float fov = 240, aspectratio = width / float(height);
#else
    //float fov = 240, aspectratio = width / float(height);
	float spread = 65, aspectratio = width / float(height);
#endif
    float angle = tan(M_PI * 0.5 * spread / 180.);

	pupil_utility = 0.0;

	//new Ray Trace grids

	/*
	1. get eye center.
	2. get eye tip (front)
	3. compute the vector. 
	4. get radius and use that to estimate the projection plane at the back of the eye
	5. use the projected point as the center of the raytracer grid. take constant to -x and +x, -y, +y. 
	6. rotate the raytracer projection point with the current eye orientation
	7. compute the direction of the ray by subtracting each raytracer grid pos from the eye tip pos 
	8. use it in *pixel = trace(ray_orig, raydir, Sphere_locals, 0);
	*/
	if(side==0)
	{
		osg::Vec3 osg_eye_center(ray_orig.x,ray_orig.y,ray_orig.z);
		
		gVec3 rot = getcurrent_ori_leye();
		osg::Quat eye_rot(rot.x()*gDTR, osg::X_AXIS, 0, osg::Y_AXIS, rot.z()*gDTR, osg::Z_AXIS);

		double xangle, yangle, zangle;
		xangle = ray_dir.x;
		xangle = ray_dir.y;
		zangle = ray_dir.z;

		double theta = xangle;
		double yrot = yangle;
		double phi = zangle;

		osg::Vec3 up(0,0,1);

		osg::Vec3 forward(0,1,0);
		osg::Vec3 eye_direction_vec = -forward;
		//osg::Quat q2(theta, osg::X_AXIS, yrot, osg::Y_AXIS,phi, osg::Z_AXIS);

		osg::Vec3 rotated_eye_dir = eye_rot*(eye_direction_vec);

#if defined(SHOW_RAYS)
		//draw ray trace lines
		osg::Vec3Array* vertices_eye = new osg::Vec3Array;
		std::vector<osg::Vec3> pup_points_eye;
		osg::Vec4Array* colors_eye = new osg::Vec4Array;
		vertices_eye->setName("VertexRayTrace");
#endif

		osg::Vec3 pupilRight = up ^ rotated_eye_dir;
		osg::Vec3 pupilUp = rotated_eye_dir ^ pupilRight; // cross product
		pupilUp.normalize();
		osg::Vec3 pupil_top_orig = osg::Vec3(SysLEye->pupil.center.x,SysLEye->pupil.center.y,SysLEye->pupil.center.z) + pupilUp*SysLEye->pupil.radius;
		osg::Vec3 pupil_forward_vec = eye_rot*forward;

		Vec3f_local pupil_top_orig_local(pupil_top_orig.x(),pupil_top_orig.y(),pupil_top_orig.z());
		Vec3f_local pupil_forward_vec_local(pupil_forward_vec.x(),pupil_forward_vec.y(),pupil_forward_vec.z());
		Vec3f_local near_int_lens;
		Vec3f_local far_int_lens;

		if(!SysLEye->lens.intersect_reverse(pupil_top_orig_local, pupil_forward_vec_local, Vec3f_local(0), far_int_lens, Vec3f_local(0))) { // if no intersection, pupil is too wide and lens is too fat, so just use lens radius instead
#ifdef UNIFORM_LENS
			pupil_top_orig = osg::Vec3(SysLEye->pupil.center.x,SysLEye->pupil.center.y,SysLEye->pupil.center.z) + pupilUp*SysLEye->lens.radius;
#else
			pupil_top_orig = osg::Vec3(SysLEye->pupil.center.x,SysLEye->pupil.center.y,SysLEye->pupil.center.z) + pupilUp*SysLEye->lens.smallRadius;
#endif
			pupil_top_orig_local = Vec3f_local(pupil_top_orig.x(),pupil_top_orig.y(),pupil_top_orig.z());
			SysLEye->lens.intersect_reverse(pupil_top_orig_local, pupil_forward_vec_local, Vec3f_local(0), far_int_lens, Vec3f_local(0));
		}

		Vec3f_local lens_sphere_v = (SysLEye->lens.sphere2.center + pupil_forward_vec_local*SysLEye->lens.sphere2.radius) - SysLEye->lens.sphere2.center;
		Vec3f_local lens_pupil_v = SysLEye->pupil.center - SysLEye->lens.sphere2.center;
		Vec3f_local hypotenuse_vec = far_int_lens - SysLEye->pupil.center;
		Vec3f_local lower_vec = lens_sphere_v - lens_pupil_v;

		double pupil_ang = acos(hypotenuse_vec.dot(lower_vec) / (lower_vec.length()*hypotenuse_vec.length()));
		double ang = asin(hypotenuse_vec.length() * sin(pupil_ang) / SysLEye->lens.sphere2.radius);

/*#ifdef SHOW_RAYS
		//draw ray trace lines
		osg::Vec3 far_int_v3(far_int_lens.x,far_int_lens.y,far_int_lens.z);
		osg::Vec3 center_vec_v3(SysLEye->pupil.center.x, SysLEye->pupil.center.y, SysLEye->pupil.center.z);
		osg::Vec3 lower_vec_v3(lower_vec.x,lower_vec.y,lower_vec.z);

		vertices_eye->push_back(pupil_top_orig);
		vertices_eye->push_back(far_int_v3);
		colors_eye->push_back(osg::Vec4(0/255.0,255/255.0,0/255.0,1));
		colors_eye->push_back(osg::Vec4(0/255.0,255/255.0,0/255.0,1));

		vertices_eye->push_back(center_vec_v3);
		vertices_eye->push_back(far_int_v3);
		colors_eye->push_back(osg::Vec4(255/255.0,0/255.0,255/255.0,1));
		colors_eye->push_back(osg::Vec4(255/255.0,0/255.0,255/255.0,1));

		vertices_eye->push_back(center_vec_v3);
		vertices_eye->push_back(center_vec_v3+lower_vec_v3);
		colors_eye->push_back(osg::Vec4(255/255.0,0/255.0,255/255.0,1));
		colors_eye->push_back(osg::Vec4(255/255.0,0/255.0,255/255.0,1));

#endif*/

		Vec3f_local lens_far_sphere_center = SysLEye->lens.sphere2.center;
		double lens_sphere_radius = SysLEye->lens.sphere2.radius;
		osg::Vec3 lens_far_center(lens_far_sphere_center.x,lens_far_sphere_center.y,lens_far_sphere_center.z);

		std::vector<osg::Vec3> lens_ring_points;
		int total_points = 0;
		
		//double dphi = 1.0 * gDTR; // TWEAK THIS
		double dphi = lens_arc_increment / SysLEye->lens.sphere2.radius; // CONSTANT ARC LENGTH
		for(double rotation_angle = 0.0; rotation_angle < ang; rotation_angle += dphi)
		{
			osg::Quat qRup(rotation_angle, osg::X_AXIS, 0, osg::Y_AXIS, 0, osg::Z_AXIS);
			osg::Vec3 result_vec = qRup*eye_rot*forward;

			if(rotation_angle == 0.0) {
				osg::Vec3 lens_point = lens_far_center + result_vec*lens_sphere_radius;
				lens_ring_points.push_back(lens_point);
				total_points++;
/*#ifdef SHOW_RAYS
				vertices_eye->push_back(lens_point-result_vec*0.1);
				vertices_eye->push_back(lens_point+result_vec*0.1);
				colors_eye->push_back(osg::Vec4(15/255.0,206/255.0,20/255.0,1));
				colors_eye->push_back(osg::Vec4(15/255.0,206/255.0,20/255.0,1));
#endif*/
			}
			else {
				int num_points = 20;
				double point_increment = 360.0 / (double) num_points;
				total_points += num_points;

				for (unsigned ltheta = 0; ltheta < num_points; ltheta++) {
					osg::Quat qRing(0, osg::X_AXIS, point_increment*ltheta*gDTR, osg::Y_AXIS, 0, osg::Z_AXIS);
					osg::Vec3 fin_dir = qRup*qRing*eye_rot*forward; // for some reason this rotates in the OPPOSITE order of what I expect
					fin_dir.normalize();
					osg::Vec3 lens_point = lens_far_center + fin_dir*lens_sphere_radius;
					lens_ring_points.push_back(lens_point);
/*#ifdef SHOW_RAYS
					vertices_eye->push_back(lens_point-fin_dir*0.1);
					vertices_eye->push_back(lens_point+fin_dir*0.1);
					colors_eye->push_back(osg::Vec4(15/255.0,206/255.0,20/255.0,1));
					colors_eye->push_back(osg::Vec4(15/255.0,206/255.0,20/255.0,1));
#endif*/
				}
			}
		}

		double ray_width = RAYTRACE_WIDTH;
		//double divisor = 5.0;
		double divisor = 10.0;
		double maxRadius = 2.0/3.0*std::exp(ray_width/divisor);
		double gaussian_mean = 0;
		double gaussian_spread = (double) RAYTRACE_WIDTH; // Peripheral falls 1 stdev / spread weight of center
		//5
		//#pragma omp parallel for num_threads(8)
		reverseRayBuffer.clear();
		for (int rho = 0; rho < RAYTRACE_WIDTH; rho++) {
			// Change_by_honglin 3: the increment is 1 instead of 4
			for (unsigned int thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT*1; thetaLogPolar+=1, ++pixelPupil, ++pixelLens, ++pixelFoveation) {
				// Change_by_honglin 4: generate new noise.txt and change the increment from 4 to 1
				double logPolarX = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0/3.0* std::exp(double(rho/divisor)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * cos(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
				double logPolarY  = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0/3.0* std::exp(double(rho/divisor)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * sin(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));

				float yy = (1 * ((logPolarX) /maxRadius)) * spread*D2R;
				float xx = (1 * ((logPolarY) /maxRadius)) * spread*D2R;

				//osg::Vec3 eye_direction_vec3(0,1,0);
				osg::Vec3 eye_direction_vec3 = -rotated_eye_dir;
				osg::Quat q3(xx, osg::X_AXIS, 0, osg::Y_AXIS,yy, osg::Z_AXIS);
				//osg::Vec3 osg_eye_temp3 = q3*(eye_direction_vec3);
				osg::Vec3 osg_eye_temp3 = eye_rot*q3*forward;

				osg_eye_temp3.normalize();
				rotated_eye_dir.normalize();
				
				//osg::Vec3 raytrace_orig(osg_eye_center.x()+logPolarX/maxRadius*EYE_SIZE/2, osg_eye_center.y()+EYE_SIZE/2, osg_eye_center.z()+logPolarY/maxRadius*EYE_SIZE/2);
				osg::Vec3 raytrace_orig = osg_eye_center + osg_eye_temp3*EYE_SIZE;
				Vec3f_local raytrace_orig_local(raytrace_orig.x(), raytrace_orig.y(), raytrace_orig.z());

#ifdef SHOW_RAYS
				//draw ray trace lines
				vertices_eye->push_back(raytrace_orig-osg_eye_temp3*0.1);
				vertices_eye->push_back(raytrace_orig+osg_eye_temp3*0.1);
				colors_eye->push_back(osg::Vec4(0/255.0,0/255.0,0/255.0,1));
				colors_eye->push_back(osg::Vec4(0/255.0,0/255.0,0/255.0,1));
#endif

				Vec3f_local cornea_near,cornea_far,cornea_refracted,pupil_near,pupil_far,pupil_refracted,lens_near,lens_far,lens_refracted;
				Vec3f_local raydir_local_sum_pupil(0);
				Vec3f_local raydir_local_sum_lens(0);
				Vec3f_local raydir_local_sum_foveation(0);

				//int total_points = 0;
				//for(std::vector<osg::Vec3>::iterator it = lens_ring_points.begin(); it != lens_ring_points.end(); ++it) {
				for(int i = 0; i < lens_ring_points.size(); i++) {
					//osg::Vec3 raydir = (*it) - raytrace_orig;
					osg::Vec3 raydir = lens_ring_points.at(i) - raytrace_orig;
					raydir.normalize();
					eye_direction_vec.normalize();
					Vec3f_local raydir_local(raydir.x(), raydir.y(),raydir.z());
					//Vec3f_local eye_rot_vec(osg_eye_temp2.x(),osg_eye_temp2.y(),osg_eye_temp2.z());
					Vec3f_local eye_rot_vec(eye_direction_vec.x(),eye_direction_vec.y(),eye_direction_vec.z());
					
					if(SysLEye->lens.intersect(raytrace_orig_local,raydir_local,lens_near,lens_far,lens_refracted)) {

						if(SysLEye->pupil.intersect(lens_far, lens_refracted, pupil_near, pupil_far, pupil_refracted)) {
							if(SysLEye->cornea.intersect(pupil_near,pupil_refracted,cornea_near,cornea_far,cornea_refracted)) {
								cornea_refracted.normalize();
								raydir_local_sum_pupil += trace(cornea_far, cornea_refracted, Sphere_locals, 0, 0);
								raydir_local_sum_lens += trace(cornea_far, cornea_refracted, Sphere_locals, 0, 2);
								raydir_local_sum_foveation += trace(cornea_far, cornea_refracted, Sphere_locals, 0, 1);
#ifdef SHOW_RAYS
								static int randomIndex1 = (int) randomNumber(0,lens_ring_points.size());
								static int randomIndex2 = (int) randomNumber(0,lens_ring_points.size());
								static int randomIndex3 = (int) randomNumber(0,lens_ring_points.size());
								static int randomIndex4 = (int) randomNumber(0,lens_ring_points.size());
								//static int randomIndex5 = (int) randomNumber(0,lens_ring_points.size());
								if(/*rho == photoreceptor_to_trace_rho && thetaLogPolar == photoreceptor_to_trace_theta*/ 
									(i == randomIndex1 || i == randomIndex2 || i == randomIndex3 || i == randomIndex4 /*|| i == randomIndex5*/) && 
									show_reverse_trace_rays) {
								osg::Vec3 phit_lens_near_v3(lens_near.x,lens_near.y,lens_near.z);
								osg::Vec3 phit_lens_far_v3(lens_far.x,lens_far.y,lens_far.z);
								osg::Vec3 phit_pupil_near_v3(pupil_near.x,pupil_near.y,pupil_near.z);
								osg::Vec3 phit_pupil_nearref_v3(pupil_refracted.x,pupil_refracted.y,pupil_refracted.z);
								osg::Vec3 phit_cornea_near_v3(cornea_near.x,cornea_near.y,cornea_near.z);
								osg::Vec3 phit_cornea_far_v3(cornea_far.x,cornea_far.y,cornea_far.z);
								osg::Vec3 cornea_final_refracted_v3(cornea_refracted.x,cornea_refracted.y,cornea_refracted.z);

								//draw ray trace lines
								vertices_eye->push_back(raytrace_orig);
								vertices_eye->push_back(phit_lens_near_v3);
								//vertices_eye->push_back(phit_lens_near_v3+osg::Vec3(raydir_local.x,raydir_local.y,raydir_local.z)*1000);
								colors_eye->push_back(osg::Vec4(15/255.0,206/255.0,20/255.0,1));
								colors_eye->push_back(osg::Vec4(15/255.0,206/255.0,20/255.0,1));
								//colors_eye->push_back(osg::Vec4(15/255.0,206/255.0,20/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(15/255.0,206/255.0,20/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));

								vertices_eye->push_back(phit_lens_near_v3);
								vertices_eye->push_back(phit_lens_far_v3);
								//osg::Vec3 templl = phit_lens_far_v3-phit_lens_near_v3;
								//templl.normalize();
								//vertices_eye->push_back(phit_lens_near_v3 + templl*200);
								colors_eye->push_back(osg::Vec4(15/255.0,20/255.0,206/255.0,1));
								colors_eye->push_back(osg::Vec4(15/255.0,20/255.0,206/255.0,1));
								//colors_eye->push_back(osg::Vec4(15/255.0,20/255.0,206/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(15/255.0,20/255.0,206/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));

								vertices_eye->push_back(phit_lens_far_v3);
								vertices_eye->push_back(phit_pupil_near_v3);
								//osg::Vec3 templp = phit_pupil_near_v3-phit_lens_far_v3;
								//templp.normalize();
								//vertices_eye->push_back(phit_lens_far_v3 + templp*200);
								colors_eye->push_back(osg::Vec4(206/255.0,20/255.0,15/255.0,1));
								colors_eye->push_back(osg::Vec4(206/255.0,20/255.0,15/255.0,1));
								//colors_eye->push_back(osg::Vec4(206/255.0,20/255.0,15/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(206/255.0,20/255.0,15/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));

								vertices_eye->push_back(phit_pupil_near_v3);
								vertices_eye->push_back(phit_cornea_near_v3);
								colors_eye->push_back(osg::Vec4(206/255.0,206/255.0,15/255.0,1));
								colors_eye->push_back(osg::Vec4(206/255.0,206/255.0,15/255.0,1));
								//colors_eye->push_back(osg::Vec4(206/255.0,206/255.0,15/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(206/255.0,206/255.0,15/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));

								vertices_eye->push_back(phit_cornea_near_v3);
								vertices_eye->push_back(phit_cornea_far_v3);
								colors_eye->push_back(osg::Vec4(206/255.0,20/255.0,206/255.0,1));
								colors_eye->push_back(osg::Vec4(206/255.0,20/255.0,206/255.0,1));
								//colors_eye->push_back(osg::Vec4(206/255.0,20/255.0,206/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(206/255.0,20/255.0,206/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));

								osg::Vec3 endpoint = phit_cornea_far_v3+cornea_final_refracted_v3*1000;

								vertices_eye->push_back(phit_cornea_far_v3);
								vertices_eye->push_back(endpoint);
								colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,0.1));
								colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,0.1));
								//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));
								//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));

								double p1 [6] = { raytrace_orig.x(), raytrace_orig.y(), raytrace_orig.z(), phit_lens_near_v3.x(), phit_lens_near_v3.y(), phit_lens_near_v3.z() }; 
								double p2 [6] = { phit_lens_near_v3.x(), phit_lens_near_v3.y(), phit_lens_near_v3.z(), phit_lens_far_v3.x(), phit_lens_far_v3.y(), phit_lens_far_v3.z() };
								double p3 [6] = { phit_lens_far_v3.x(), phit_lens_far_v3.y(), phit_lens_far_v3.z(), phit_pupil_near_v3.x(), phit_pupil_near_v3.y(), phit_pupil_near_v3.z() };
								double p4 [6] = { phit_pupil_near_v3.x(), phit_pupil_near_v3.y(), phit_pupil_near_v3.z(), phit_cornea_near_v3.x(), phit_cornea_near_v3.y(), phit_cornea_near_v3.z() };
								double p5 [6] = { phit_cornea_near_v3.x(), phit_cornea_near_v3.y(), phit_cornea_near_v3.z(), phit_cornea_far_v3.x(), phit_cornea_far_v3.y(), phit_cornea_far_v3.z() };
								double p6 [6] = { phit_cornea_far_v3.x(), phit_cornea_far_v3.y(), phit_cornea_far_v3.z(), endpoint.x(), endpoint.y(), endpoint.z() };
								
								vector<double> p1vec (p1, p1 + sizeof(p1) / sizeof(p1[0]) );
								vector<double> p2vec (p2, p2 + sizeof(p2) / sizeof(p2[0]) );
								vector<double> p3vec (p3, p3 + sizeof(p3) / sizeof(p3[0]) );
								vector<double> p4vec (p4, p4 + sizeof(p4) / sizeof(p4[0]) );
								vector<double> p5vec (p5, p5 + sizeof(p5) / sizeof(p5[0]) );
								vector<double> p6vec (p6, p6 + sizeof(p6) / sizeof(p6[0]) );

								reverseRayBuffer.push_back(p1vec);
								reverseRayBuffer.push_back(p2vec);
								reverseRayBuffer.push_back(p3vec);
								reverseRayBuffer.push_back(p4vec);
								reverseRayBuffer.push_back(p5vec);
								reverseRayBuffer.push_back(p6vec);
								}
#endif
							}
						}
					}
				}
				//double scaling = (total_points == 0) ? 0 : 1.0 / (double)(total_points);
				double scaling = 1.0 / 45.066667; // TWEAK THIS
				double exp_term = -0.5 * (rho / gaussian_spread) * (rho / gaussian_spread);
				double gaussian_weight = 1.2 * exp(exp_term); // can adjust the peak weight and spread
				raydir_local_sum_pupil *= gaussian_weight;
				raydir_local_sum_lens *= gaussian_weight;
				raydir_local_sum_foveation *= gaussian_weight;

				//cout << "local sum: " << 3 * raydir_local_sum.x << endl;

				Vec3f_local final_val_pupil(
					min(1.0,raydir_local_sum_pupil.x * scaling * gaussian_weight), 
					min(1.0,raydir_local_sum_pupil.y * scaling * gaussian_weight), 
					min(1.0,raydir_local_sum_pupil.z * scaling * gaussian_weight)
				);

				Vec3f_local final_val_lens(
					min(1.0,raydir_local_sum_lens.x * scaling * gaussian_weight), 
					min(1.0,raydir_local_sum_lens.y * scaling * gaussian_weight), 
					min(1.0,raydir_local_sum_lens.z * scaling * gaussian_weight)
				);

				Vec3f_local final_val_foveation(
					min(1.0,raydir_local_sum_foveation.x * scaling * gaussian_weight), 
					min(1.0,raydir_local_sum_foveation.y * scaling * gaussian_weight), 
					min(1.0,raydir_local_sum_foveation.z * scaling * gaussian_weight)
				);
				
				/*if((rho == 0 && thetaLogPolar == 0) || (rho == 27 && thetaLogPolar == 0)) {
					cout << "raydir local sum: " << rho << " " << raydir_local_sum.x << " " << raydir_local_sum.y << " " << raydir_local_sum.z << endl;
					cout << "value: " << final_val.x << endl;
				}*/

				pupil_utility += final_val_pupil.x + final_val_pupil.y + final_val_pupil.z;
				*pixelPupil = final_val_pupil;

				*pixelLens = final_val_lens;

				*pixelFoveation = final_val_foveation;
			}
			
		}
#if defined(SHOW_RAYS)
		//draw ray trace lines
		osg::ref_ptr<osg::Geometry> geom_active = new osg::Geometry;
		geom_active->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, vertices_eye->size()));
		geom_active->setUseDisplayList(false);

		geom_active->setVertexArray(vertices_eye);
		geom_active->setColorArray(colors_eye);
		geom_active->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

		osg::ref_ptr<osg::Geode> geode_active = new osg::Geode;
		geode_active->addDrawable(geom_active.get());

		osg::StateSet* state_active = geode_active->getOrCreateStateSet();
		state_active->setMode(GL_LIGHTING, osg::StateAttribute::ON);
		state_active->setMode(GL_BLEND, osg::StateAttribute::ON);
		state_active->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
		osg::LineWidth* lw_active = new osg::LineWidth;
		lw_active->setWidth(5.0);
		//lw_active->setWidth(0.01);
		//osg::Point *point_size = new osg::Point(6.0f);
		state_active->setAttribute(lw_active, osg::StateAttribute::ON);
		osg::BlendFunc* blendfunc_active = new osg::BlendFunc();
		state_active->setAttributeAndModes(blendfunc_active, osg::StateAttribute::ON);
		pat_raytrace_lines = new osg::PositionAttitudeTransform();
		pat_raytrace_lines->addChild(geode_active);

		pat_raytrace_lines->setPosition(osg::Vec3d(0.0, 0, 0.0));

		if(raytraceLineGroup->getNumChildren()>0)
			raytraceLineGroup->removeChild(0,1);
		raytraceLineGroup->addChild(pat_raytrace_lines);
#endif
    }
	else{
		osg::Vec3 osg_eye_center(ray_orig.x,ray_orig.y,ray_orig.z);
		
		double xangle, yangle;
		xangle = ray_dir.x;
		yangle = ray_dir.z;

		double theta = xangle;
		double phi   = yangle;

		osg::Vec3 eye_direction_vec(0,-1,0);
		osg::Quat q2(theta, osg::X_AXIS, 0, osg::Y_AXIS,phi, osg::Z_AXIS);
		osg::Vec3 osg_eye_temp2 = q2*(eye_direction_vec);

#ifdef SHOW_RAYS
		//draw ray trace lines
		osg::Vec3Array* vertices_eye = new osg::Vec3Array;
		osg::Vec4Array* colors_eye = new osg::Vec4Array;
		vertices_eye->setName("VertexRayTrace");
#endif

		osg::Vec3 rotated_eye_dir;

		double ray_width = RAYTRACE_WIDTH;
		double maxRadius = 2.0/3.0*std::exp(ray_width/5.0*40.0/RAYTRACE_WIDTH);
		//5
		
		//#pragma omp parallel for num_threads(8) 
		for (int rho = 0; rho < RAYTRACE_WIDTH; rho++) {
			// Change_by_honglin 5: the increment is 1 instead of 4
			for (unsigned thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT*1; thetaLogPolar+=1, ++pixelPupil, ++pixelLens, ++pixelFoveation) {
				// Change_by_honglin 6
				double logPolarX = 2.0/3.0*std::exp(double(rho/5.0*40.0/RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * cos(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
				double logPolarY = 2.0/3.0* std::exp(double(rho/5.0*40.0/RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * sin(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));

				float yy = (1 * ((logPolarX) /maxRadius)) * spread*D2R;
				float xx = (1 * ((logPolarY) /maxRadius)) * spread*D2R;

				osg::Vec3 eye_direction_vec3(0,1,0);
				osg::Quat q3(xx, osg::X_AXIS, 0, osg::Y_AXIS,yy, osg::Z_AXIS);
				osg::Vec3 osg_eye_temp3 = q3*(eye_direction_vec3);

				osg_eye_temp3.normalize();
				osg_eye_temp2.normalize();
				
				osg::Vec3 raytrace_orig(osg_eye_center.x()+logPolarX/maxRadius*EYE_SIZE/2, osg_eye_center.y()+EYE_SIZE/2, osg_eye_center.z()+logPolarY/maxRadius*EYE_SIZE/2);
				osg::Vec3 pin_hole_pos = osg_eye_center + osg::Vec3(0,-1,0)*EYE_SIZE/2;
				
				Vec3f_local raytrace_orig_local(raytrace_orig.x(), raytrace_orig.y(), raytrace_orig.z());
				osg::Vec3 raydir = pin_hole_pos - raytrace_orig;
				raydir = q2*raydir;
				raydir.normalize();				
				Vec3f_local raydir_local(raydir.x(), raydir.y(),raydir.z());

				
				//*pixel = trace(raytrace_orig_local, raydir_local, Sphere_locals, 0);
				*pixelPupil = trace(raytrace_orig_local, raydir_local, Sphere_locals, 0, 0);
				*pixelLens = trace(raytrace_orig_local, raydir_local, Sphere_locals, 0, 2);
				*pixelFoveation = trace(raytrace_orig_local, raydir_local, Sphere_locals, 0, 1);


#ifdef SHOW_RAYS
				//draw ray trace lines
				vertices_eye->push_back(raytrace_orig);
				vertices_eye->push_back(raytrace_orig+raydir*4000.0);
				colors_eye->push_back(osg::Vec4(135/255.0,206/255.0,250/255.0,1));
				colors_eye->push_back(osg::Vec4(135/255.0,206/255.0,250/255.0,1));
				//colors_eye->push_back(osg::Vec4(0.9,0.9,0.3,1));
#endif
			}
		}
		
#ifdef SHOW_RAYS
		//draw ray trace lines
		osg::ref_ptr<osg::Geometry> geom_active = new osg::Geometry;
		geom_active->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, vertices_eye->size()));
		geom_active->setUseDisplayList(false);

		geom_active->setVertexArray(vertices_eye);
		geom_active->setColorArray(colors_eye);
		geom_active->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

		osg::ref_ptr<osg::Geode> geode_active = new osg::Geode;
		geode_active->addDrawable(geom_active.get());

		osg::StateSet* state_active = geode_active->getOrCreateStateSet();
		state_active->setMode(GL_LIGHTING, osg::StateAttribute::ON);
		state_active->setMode(GL_BLEND, osg::StateAttribute::ON);
		state_active->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
		osg::LineWidth* lw_active = new osg::LineWidth;
		lw_active->setWidth(0.1);
		//osg::Point *point_size = new osg::Point(6.0f);
		state_active->setAttribute(lw_active, osg::StateAttribute::ON);
		osg::BlendFunc* blendfunc_active = new osg::BlendFunc();
		state_active->setAttributeAndModes(blendfunc_active, osg::StateAttribute::ON);
		pat_raytrace_lines = new osg::PositionAttitudeTransform();
		pat_raytrace_lines->addChild(geode_active);

		pat_raytrace_lines->setPosition(osg::Vec3d(0.0, 0, 0.0));

		if(raytraceLineGroup->getNumChildren()>0)
			raytraceLineGroup->removeChild(0,1);
		raytraceLineGroup->addChild(pat_raytrace_lines);
#endif
	}

#if !defined(RECORD_FOVEATION) && !defined(TEST_FOVEATION)
	adjust_pupil_sphincter_muscles();
	adjust_lens_cilliary_muscles();
#endif

	for (unsigned i = 0; i < RAYTRACE_WIDTH * RAYTRACE_HEIGHT; i++) {
		returnedImagePupil[i].x = min(float(1), imagePupil[i].x);
		returnedImagePupil[i].y = min(float(1), imagePupil[i].y);
		returnedImagePupil[i].z = min(float(1), imagePupil[i].z);

		returnedImageLens[i].x = min(float(1), imageLens[i].x);
		returnedImageLens[i].y = min(float(1), imageLens[i].y);
		returnedImageLens[i].z = min(float(1), imageLens[i].z);

		returnedImageFoveation[i].x = min(float(1), imageFoveation[i].x);
		returnedImageFoveation[i].y = min(float(1), imageFoveation[i].y);
		returnedImageFoveation[i].z = min(float(1), imageFoveation[i].z);
	}

    delete [] imagePupil;
	delete [] imageLens;
	delete [] imageFoveation;
}

void exportReverseTraceBuffer() {
	vector< vector<double> >::iterator row;
	fprintf(fileRaytracePoints,"%d %d\n",frame,reverseRayBuffer.size());
	for (row = reverseRayBuffer.begin(); row != reverseRayBuffer.end(); row++) {
		vector<double> entry = *row;
		fprintf(fileRaytracePoints,"%.2f %.2f %.2f %.2f %.2f %.2f\n", entry.at(0), entry.at(1), entry.at(2), entry.at(3), entry.at(4), entry.at(5));
	}
	fflush(fileRaytracePoints);
}

void adjust_pupil_sphincter_muscles()
{
	double ideal_utility = 25830.0; // TWEAK THIS - MOST DIFFICULT TO FIX
	double pupil_utility_diff = abs(pupil_utility - ideal_utility);

	//cout << "pupil utility, diff: " << pupil_utility << " " << pupil_utility_diff << endl;

	static double acc = 0.0;
#ifndef ONLINE_PUPIL
#if !defined(TEST_LENS) && !defined(TEST_PUPIL) && !defined(TEST_FOVEATION)
#if defined(RECORD_LENS)
	if(new_target_ball){
		acc = randomNumber(0.0,1.0);
	}
#else
#ifdef LENS_VIDEO
	if(oscillatePupilSphincterActivation) {
		static int dir = 1;
		static int maxPupilTimes = 0;
		double sphincter_act_level = SysLEye->muscle(6)->actLevel();
		if(sphincter_act_level <= 0) {
			dir = 1;
		}
		else if(sphincter_act_level >= 1.0) {
			dir = -1;
			maxPupilTimes++;
			if(maxPupilTimes >= 3) {
				pupilMovementDemoComplete = true;
			}
		}
		acc = min(1.0,max(sphincter_act_level + dir*0.02, 0.0));
	}
	else {
#endif
	double alpha = 0.01; // TWEAK THIS
	double beta = 0.0003; // TWEAK THIS

	static int pupil_adjusted_delay = 0;

	double sphincter_act_level = SysLEye->muscle(6)->actLevel();
	double sphincter_act_adj = min(alpha*(exp(abs(beta*pupil_utility_diff))-1.0), 0.2); // TWEAK THIS

	static double diff_epsilon = 1500;
	static int num_adjustment_frames = 0;

	pupil_adjusted = false;
	if(pupil_utility_diff > diff_epsilon) {
		if(pupil_utility > ideal_utility && sphincter_act_level < 1.0) { // constrict
			pupil_adjusted_delay = 0;
			pupil_adjustment_direction = 1;
			num_adjustment_frames++;
		}
		else if(pupil_utility < ideal_utility && sphincter_act_level > 0.0) { // dilate
			pupil_adjusted_delay = 0;
			pupil_adjustment_direction = -1;
			num_adjustment_frames++;
		}
		else {
			if(pupil_adjusted_delay >= 1) {
				pupil_adjusted = true;
				pupil_adjusted_delay = 0;
				diff_epsilon = 1500;
				num_adjustment_frames = 0;
			}
			else {
				pupil_adjusted_delay++;
				num_adjustment_frames++;
			}
			pupil_adjustment_direction = 0; 
		}
	}
	else {
		if(pupil_adjusted_delay >= 1) {
			pupil_adjusted = true;
			pupil_adjusted_delay = 0;
			diff_epsilon = 1500;
			num_adjustment_frames = 0;
		}
		else {
			pupil_adjusted_delay++;
			num_adjustment_frames++;
		}
		pupil_adjustment_direction = 0; 
	}

	if(num_adjustment_frames > 0 && (num_adjustment_frames % 40) == 0) { // use 40 as final
		//cout << "too many frames - increasing epsilon: " << endl;
		diff_epsilon += 500;
	}

	//cout << "num adjusted frames, epsilon: " << num_adjustment_frames << " " << diff_epsilon << endl;

	double directional_act_adj = pupil_adjustment_direction*sphincter_act_adj;
#ifdef RECORD_LENS
	acc = 0.18; // REVERT THIS
#else
	acc = min(1.0,max(sphincter_act_level + directional_act_adj, 0.0));
	//acc = 0.18; // REVERT THIS
#endif
	//double directional_act_adj = 0.5;
#ifdef LENS_VIDEO
	}
#endif
#endif
	//cout << "error, curr and delt pupil: " << abs(pupil_utility_diff) << " " << sphincter_act_level << " " << directional_act_adj << endl;

	if(bPlay) {
		for(int i = 6; i < 14; i++) {
			SysLEye->muscle(i)->setActLevel(acc);
		}
	}
#endif
/*	int repeat = (int)(displayTimeStep/simulationTimeStep) + 1;
	for(int i=0;i<repeat;++i){
		SysLEye->clearAllForcesWithoutMuscle();
		SysLEye->updateExtForce();
		SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
		simCount++;

		curTime += simulationTimeStep;
	}*/
#endif
	/*int repeat = (int)(displayTimeStep/simulationTimeStep) + 1;
	for(int i=0;i<repeat;++i){
		SysLEye->clearAllForcesWithoutMuscle();
		SysLEye->updateExtForce();
		SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
		simCount++;

		curTime += simulationTimeStep;
	}
	compute_pupil_size();
	SysLEye->adjust_pupil(pupil_size);*/
}

void compute_pupil_size()
{
	double epsilon_correction = 0.5;

	gLink* pupilNorth = SysLEye->findLink("Iris_Sphincter_Point_North");
	gLink* pupilWest = SysLEye->findLink("Iris_Sphincter_Point_West");
	gLink* pupilSouth = SysLEye->findLink("Iris_Sphincter_Point_South");
	gLink* pupilEast = SysLEye->findLink("Iris_Sphincter_Point_East");
	gLink* pupilNorthWest = SysLEye->findLink("Iris_Sphincter_Point_NorthWest");
	gLink* pupilSouthWest = SysLEye->findLink("Iris_Sphincter_Point_SouthWest");
	gLink* pupilNorthEast = SysLEye->findLink("Iris_Sphincter_Point_NorthEast");
	gLink* pupilSouthEast = SysLEye->findLink("Iris_Sphincter_Point_SouthEast");

	gReal diff1 = (pupilNorth->frame().trn() - pupilSouth->frame().trn()).magnitude();
	gReal diff2 = (pupilWest->frame().trn() - pupilEast->frame().trn()).magnitude();
	gReal diff3 = (pupilNorthWest->frame().trn() - pupilSouthEast->frame().trn()).magnitude();
	gReal diff4 = (pupilSouthWest->frame().trn() - pupilNorthEast->frame().trn()).magnitude();

	pupil_size = 0.5 * ((diff1 + diff2 + diff3 + diff4) / 4.0) - epsilon_correction;
}

void maybeRecordBallPos() {
	cout << "lpfd len: " << lfpd.length() << endl;
	if(lfpd.length() <= 0.4) {
		gVec3 ballpos = visualTarget.get()->getMatrix().getTrans().ptr();
		finputlensmaybeballpos << ballpos.x() << "," << ballpos.y() << "," << ballpos.z() << "," << vis_target_ec << endl;
	}
}

void adjust_lens_cilliary_muscles()
{
#ifndef ONLINE_LENS
	Vec3f_local lens_focal_point = compute_focal_point();

	record_lens_data = (lens_focal_point.y == lens_focal_point.y);

	gLink* base;
	base = SysLEye->findLink("Eyeball");
	Vec3f_local ray_orig(base->frame().trn().x(),base->frame().trn().y(),base->frame().trn().z());
	osg::Vec3 osg_eye_center(ray_orig.x,ray_orig.y,ray_orig.z);

	osg::Vec3 forward(0,1,0);
	gVec3 ori_leye = getcurrent_ori_leye()*gDTR;
	osg::Quat q(ori_leye.x(), osg::X_AXIS, 0, osg::Y_AXIS, ori_leye.z(), osg::Z_AXIS);
	forward = q*(forward);

	osg::Vec3 eye_back_frontier = osg_eye_center + forward*EYE_SIZE;
	osg::Vec3 lfpv3(lens_focal_point.x,lens_focal_point.y,lens_focal_point.z);
	//osg::Vec3 lfpv3(stored_focal_point.x,stored_focal_point.y,stored_focal_point.z);
	osg::Vec3 diffLens = lfpv3 - eye_back_frontier;
	lfpd = diffLens;
#endif

	//cout << "acc: " << SysLEye->muscle(14)->actLevel() << endl;

	static double act = 0.0;
#ifndef ONLINE_LENS
#if !defined(TEST_LENS) && !defined(TEST_PUPIL) && !defined(TEST_FOVEATION)
#if defined(RECORD_PUPIL)
	if(new_target_ball){
		acc = randomNumber(0.0,1.0);
	}
	int direction = ((diffLens * forward) > 0) ? 1 : -1;
	double diffLensMagnitude = diffLens.length();

	lens_error = diffLensMagnitude;
#else
#ifdef LENS_VIDEO
	if(oscillateLensCilliaryActivation) {
		static int dir = 1;
		static int maxLensTimes = 0;
		double cilliary_act_level = SysLEye->muscle(14)->actLevel();
		if(cilliary_act_level <= 0) {
			dir = 1;
		}
		else if(cilliary_act_level >= 1.0) {
			dir = -1;
			maxLensTimes++;
			if(maxLensTimes >= 2) {
				lensMovementDemoComplete = true;
			}
		}
		acc = min(1.0,max(cilliary_act_level + dir*0.02, 0.0));
	}
	else {
#endif
	static double epsilon = 0.2;
	//static double epsilon = 0.35;
	//static double epsilon = 0.7; // constant rate = higher tolerance
	static int num_adjustment_frames = 0;

	static int lens_adjusted_delay = 0;

	int direction = ((diffLens * forward) > 0) ? 1 : -1;
	double diffLensMagnitude = diffLens.length();

	lens_error = diffLensMagnitude;

	//cout << "focal point location, error: " << lens_focal_point.y << " " << lens_error << endl;

	int lens_adjustment_direction = 0;
	double alpha = 0.01;
	double beta = 1.1;

	double cilliary_act_level = SysLEye->muscle(14)->actLevel();
	double cilliary_act_adj = min(alpha*(exp(beta*diffLensMagnitude) - 1.0), 0.05);

	//double cilliary_act_adj = 0.03; // constant rate

	//double lens_adjustment_rate = min(alpha*exp(abs(beta*yDiffLens)), 0.8);

	lens_adjusted = false;
	if(diffLens.length() > epsilon) {
		if(direction == 1 && cilliary_act_level < 1.0) {
			lens_adjusted_delay = 0;
			lens_adjustment_direction = 1;
			num_adjustment_frames++;
		}
		else if(direction == -1 && cilliary_act_level > 0.0) {
			lens_adjusted_delay = 0;
			lens_adjustment_direction = -1;
			num_adjustment_frames++;
		}
		else {
			if(lens_adjusted_delay >= 1) {
				lens_adjusted = true;
				lens_adjusted_delay = 0;
				epsilon = 0.2;
				num_adjustment_frames = 0;
			}
			else {
				lens_adjusted_delay++;
				num_adjustment_frames++;
			}
			lens_adjustment_direction = 0;
		}
	}
	else {
		if(lens_adjusted_delay >= 1) {
			lens_adjusted = true;
			lens_adjusted_delay = 0;
			epsilon = 0.2;
			num_adjustment_frames = 0;
		}
		else {
			lens_adjusted_delay++;
			num_adjustment_frames++;
		}
		lens_adjustment_direction = 0; // in focus
	}

	if(num_adjustment_frames > 0 && (num_adjustment_frames % 45) == 0) { // change to 40
		//cout << "too many frames - increasing epsilon: " << endl;
		epsilon += 0.15;
	}

	//cout << "num adjusted frames, epsilon: " << num_adjustment_frames << " " << epsilon << endl;

	double directional_act_adj = lens_adjustment_direction*cilliary_act_adj;

	//cout << "error, curr and delt lens: " << diffLensMagnitude << " " << cilliary_act_level << " " << directional_act_adj << endl;

	act = min(1.0,max(cilliary_act_level + directional_act_adj, 0.0));
	//acc = 0.0;
	//acc = 1.0;
#ifdef LENS_VIDEO
	}
#endif
#endif

	//cout << "acc: " << act << endl;

	if(bPlay) {
		for(int i = 14; i < 22; i++) {
			SysLEye->muscle(i)->setActLevel(act);
		}
		for(int i = 38; i < 54; i++) {
			SysLEye->muscle(i)->setActLevel(0.038*(1-act));
		}

		//cout << "lens act level from fxn: " << SysLEye->muscle(14)->actLevel() << endl;
	}

#endif
#endif
}


void adjust_lens_cilliary_muscles_trained_dnn(double direction)
{
	double epsilon = 0.1;
	int lens_adjustment_direction = 0;
	double cilliary_act_level = SysLEye->muscle(14)->actLevel();
	//double cilliary_act_adj = min(alpha*exp(beta*diffLensMagnitude), 0.2);

	//double lens_adjustment_rate = min(alpha*exp(abs(beta*yDiffLens)), 0.8);

	if(direction > 0 && cilliary_act_level < 1.0) {
		lens_adjustment_direction = 1;
	}
	else if(direction <0 && cilliary_act_level > 0.0) {
		lens_adjustment_direction = -1;
	}
	else {
		lens_adjustment_direction = 0;
	}

	double directional_act_adj = lens_adjustment_direction*0.1;

	double acc = min(1.0,max(cilliary_act_level + directional_act_adj, 0.0));

	for(int i = 14; i < 22; i++) {
		SysLEye->muscle(i)->setActLevel(acc);
	}
	for(int i = 38; i < 54; i++) {
		SysLEye->muscle(i)->setActLevel(0.072*(1-acc));
	}
}

void compute_lens_size()
{
	double epsilon_correction = 0.5;

	gLink* lensWestFront = SysLEye->findLink("Lens_Surface_Point_West_Front");
	gLink* lensWestRear = SysLEye->findLink("Lens_Surface_Point_West_Rear");
	gLink* lensNorthWestFront = SysLEye->findLink("Lens_Surface_Point_NorthWest_Front");
	gLink* lensNorthWestRear = SysLEye->findLink("Lens_Surface_Point_NorthWest_Rear");
	gLink* lensNorthFront = SysLEye->findLink("Lens_Surface_Point_North_Front");
	gLink* lensNorthRear = SysLEye->findLink("Lens_Surface_Point_North_Rear");
	gLink* lensNorthEastFront = SysLEye->findLink("Lens_Surface_Point_NorthEast_Front");
	gLink* lensNorthEastRear = SysLEye->findLink("Lens_Surface_Point_NorthEast_Rear");
	gLink* lensEastFront = SysLEye->findLink("Lens_Surface_Point_East_Front");
	gLink* lensEastRear = SysLEye->findLink("Lens_Surface_Point_East_Rear");
	gLink* lensSouthWestFront = SysLEye->findLink("Lens_Surface_Point_SouthWest_Front");
	gLink* lensSouthWestRear = SysLEye->findLink("Lens_Surface_Point_SouthWest_Rear");
	gLink* lensSouthFront = SysLEye->findLink("Lens_Surface_Point_South_Front");
	gLink* lensSouthRear = SysLEye->findLink("Lens_Surface_Point_South_Rear");
	gLink* lensSouthEastFront = SysLEye->findLink("Lens_Surface_Point_SouthEast_Front");
	gLink* lensSouthEastRear = SysLEye->findLink("Lens_Surface_Point_SouthEast_Rear");

	gReal diff1 = (lensWestFront->frame().trn() - lensWestRear->frame().trn()).magnitude();
	gReal diff2 = (lensNorthWestFront->frame().trn() - lensNorthWestRear->frame().trn()).magnitude();
	gReal diff3 = (lensNorthFront->frame().trn() - lensNorthRear->frame().trn()).magnitude();
	gReal diff4 = (lensNorthEastFront->frame().trn() - lensNorthEastRear->frame().trn()).magnitude();
	gReal diff5 = (lensEastFront->frame().trn() - lensEastRear->frame().trn()).magnitude();
	gReal diff6 = (lensSouthWestFront->frame().trn() - lensSouthWestRear->frame().trn()).magnitude();
	gReal diff7 = (lensSouthFront->frame().trn() - lensSouthRear->frame().trn()).magnitude();
	gReal diff8 = (lensSouthEastFront->frame().trn() - lensSouthEastRear->frame().trn()).magnitude();

	double d = (diff1 + diff2 + diff3 + diff4 + diff5 + diff6 + diff7 + diff8) / 8.0;
	double w = (double) MIN_LENS_WIDTH + ((d - dlensmin) / (dlensmax - dlensmin)) * ((double) MAX_LENS_WIDTH - (double) MIN_LENS_WIDTH);

	SysLEye->adjust_lens(w);
}


Vec3f_local compute_focal_point()
{
	osg::Vec3 forward(0,1,0);
	osg::Vec3 up(0,0,1);
	gVec3 targetPos = visualTarget.get()->getMatrix().getTrans().ptr();
	osg::Vec3 tposv3(targetPos.x(),targetPos.y(),targetPos.z());
	Vec3f_local orig_local(tposv3.x(), tposv3.y(), tposv3.z());

	gVec3 rot = getcurrent_ori_leye();
	osg::Quat eye_rot(rot.x()*gDTR, osg::X_AXIS, 0, osg::Y_AXIS, rot.z()*gDTR, osg::Z_AXIS);

	std::vector<Vec3f_local> final_box_int_points;

	Vec3f_local cornea_near,cornea_far,cornea_refracted,pupil_near,pupil_far,pupil_refracted,lens_near,lens_far,lens_refracted;

#if defined(SHOW_RAYS) || defined(SHOW_LENS_SPHERES) || defined(SHOW_CORNEA_SPHERES)
	//draw ray trace lines
	osg::Vec3Array* vertices_ball = new osg::Vec3Array;
	osg::Vec4Array* colors_ball = new osg::Vec4Array;
	osg::Vec3Array* normals_ball = new osg::Vec3Array;
	osg::Vec3 normalRayVertexDir = osg::Vec3(-1,0,0);

	osg::Vec3Array* vertices_ball_allrays = new osg::Vec3Array;
	osg::Vec4Array* colors_ball_allrays = new osg::Vec4Array;
	osg::Vec3Array* normals_ball_allrays = new osg::Vec3Array;
	vertices_ball->setName("VertexRayTraceBall");
#endif

	gLink* base = SysLEye->findLink("Eyeball");
	gVec3 baseWorldPos = base->frame().trn();

	gVec3 tP = visualTarget.get()->getMatrix().getTrans().ptr();
	tP += gVec3(0,SPHERE_SIZE,0);

	osg::Vec3 osg_eye_center(baseWorldPos.x(),baseWorldPos.y(),baseWorldPos.z());
	Vec3f_local eye_center(osg_eye_center.x(),osg_eye_center.y(),osg_eye_center.z());

	osg::Vec3 rot_f_vec = eye_rot*forward;
	Vec3f_local rot_f_vec_local(rot_f_vec.x(),rot_f_vec.y(),rot_f_vec.z());
	rot_f_vec_local.normalize();

	Vec3f_local ball_lens_ray = SysLEye->lens.center - orig_local;
	ball_lens_ray.normalize();

	//Vec3f_local min_b = eye_center + rot_f_vec_local*10000;
	//Vec3f_local max_b = eye_center - rot_f_vec_local*10000;

	gVec3 blr(ball_lens_ray.x,ball_lens_ray.y,ball_lens_ray.z);
	gVec3 upg(up.x(),up.y(),up.z());
	gVec3 left = upg % blr;
	left.normalize();
	gVec3 dirup = blr % left;
	dirup.normalize();

	Vec3f_local leftlocal(left.x(),left.y(),left.z());
	Vec3f_local uplocal(dirup.x(),dirup.y(),dirup.z());

	Vec3f_local min_b = SysLEye->lens.center + leftlocal*0.1 - uplocal*0.1;
	Vec3f_local max_b = orig_local + ball_lens_ray*10000 - leftlocal*0.1 + uplocal*0.1;

	Box_local intersection_plane = Box_local(min_b,max_b);

	double xrot;
	Vec3f_local maxz(0);
	Vec3f_local step_max(0);
	//#pragma omp parallel for num_threads(8)
	if(exportMotion && show_forward_trace_rays)
	{
		fprintf(fileRaytracePoints,"%d \n",frame);
	}
	for (int xrot_ind = -900; xrot_ind <= 900; xrot_ind+=1) {
		xrot = xrot_ind/ 10.0;
		//cout << xrot << endl;
		for (double zrot = -90.0; zrot <= 90.0; zrot+=0.1) {
			
			osg::Quat nrot(xrot*D2R, osg::X_AXIS, 0, osg::Y_AXIS, zrot*D2R, osg::Z_AXIS);
			//osg::Vec3 normal = nrot*forward;
			osg::Vec3 normal = eye_rot*nrot*forward;
			osg::Vec3 forward_relative = eye_rot*forward;

			normal.normalize();

			Vec3f_local normal_raydir_local(normal.x(), normal.y(), normal.z());
			Vec3f_local surface_point = orig_local + normal_raydir_local*SPHERE_SIZE;
			Vec3f_local forward_ray(forward_relative.x(),forward_relative.y(),forward_relative.z());

#ifdef SHOW_RAYS
			if(show_full_ball_trace) {
				if((int) xrot % 5 == 0 && (int) zrot % 5 == 0) {
					osg::Vec3 meh(surface_point.x,surface_point.y,surface_point.z);
					//draw ray trace lines
					vertices_ball_allrays->push_back(meh);
					vertices_ball_allrays->push_back(meh+normal*400);
					colors_ball_allrays->push_back(osg::Vec4(255.0/255.0,255.0/255.0,255.0/255.0,full_ball_alpha_channel));
					colors_ball_allrays->push_back(osg::Vec4(255.0/255.0,255.0/255.0,255.0/255.0,full_ball_alpha_channel));
					normals_ball_allrays->push_back(normalRayVertexDir);
					normals_ball_allrays->push_back(normalRayVertexDir);
				}
			}
#endif

			if(SysLEye->cornea.intersect_reverse(orig_local,normal_raydir_local,cornea_near,cornea_far,cornea_refracted)) {
				osg::Vec3 cornea_near_v3(cornea_near.x,cornea_near.y,cornea_near.z);
				osg::Vec3 cornea_far_v3(cornea_far.x,cornea_far.y,cornea_far.z);
				osg::Vec3 cornea_refracted_v3(cornea_refracted.x,cornea_refracted.y,cornea_refracted.z);
/*#ifdef SHOW_RAYS
				if(show_forward_trace_rays) {
				//draw ray trace lines
				vertices_ball->push_back(tposv3);
				vertices_ball->push_back(cornea_near_v3);
				colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));
				colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));

				vertices_ball->push_back(cornea_near_v3);
				vertices_ball->push_back(cornea_far_v3);
				colors_ball->push_back(osg::Vec4(10/255.0,10/255.0,206/255.0,1));
				colors_ball->push_back(osg::Vec4(10/255.0,10/255.0,206/255.0,1));
				}

#endif*/

				if(SysLEye->pupil.intersect_reverse(cornea_far, cornea_refracted, pupil_near, pupil_far, pupil_refracted)) {
/*#ifdef SHOW_RAYS
					osg::Vec3 pupil_near_v3(pupil_near.x,pupil_near.y,pupil_near.z);

					vertices_ball->push_back(cornea_far_v3);
					vertices_ball->push_back(pupil_near_v3);
					colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,1));
					colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,1));
#endif*/

					if(SysLEye->lens.intersect_reverse(pupil_near, pupil_refracted, lens_near, lens_far, lens_refracted)) {
#ifdef SHOW_RAYS
						if(show_forward_trace_rays) {
						osg::Vec3 pupil_near_v3(pupil_near.x,pupil_near.y,pupil_near.z);
						osg::Vec3 lens_near_v3(lens_near.x,lens_near.y,lens_near.z);
						osg::Vec3 lens_far_v3(lens_far.x,lens_far.y,lens_far.z);
						osg::Vec3 lens_refracted_v3(lens_refracted.x,lens_refracted.y,lens_refracted.z);

						vertices_ball->push_back(tposv3);
						vertices_ball->push_back(cornea_near_v3);
						normals_ball->push_back(up);
						//colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));
						//colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));
						colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,forward_rays_alpha_channel));
						colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,forward_rays_alpha_channel));
						normals_ball->push_back(normalRayVertexDir);
						normals_ball->push_back(normalRayVertexDir);

						vertices_ball->push_back(cornea_near_v3);
						vertices_ball->push_back(cornea_far_v3);
						normals_ball->push_back(up);
						//colors_ball->push_back(osg::Vec4(10/255.0,10/255.0,206/255.0,1));
						//colors_ball->push_back(osg::Vec4(10/255.0,10/255.0,206/255.0,1));
						colors_ball->push_back(osg::Vec4(10/255.0,10/255.0,206/255.0,forward_rays_alpha_channel));
						colors_ball->push_back(osg::Vec4(10/255.0,10/255.0,206/255.0,forward_rays_alpha_channel));
						normals_ball->push_back(normalRayVertexDir);
						normals_ball->push_back(normalRayVertexDir);

						vertices_ball->push_back(cornea_far_v3);
						vertices_ball->push_back(pupil_near_v3);
						normals_ball->push_back(up);
						//colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,1));
						//colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,1));
						colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,forward_rays_alpha_channel));
						colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,forward_rays_alpha_channel));
						normals_ball->push_back(normalRayVertexDir);
						normals_ball->push_back(normalRayVertexDir);

						vertices_ball->push_back(pupil_near_v3);
						vertices_ball->push_back(lens_near_v3);
						normals_ball->push_back(up);
						//colors_ball->push_back(osg::Vec4(206/255.0,206/255.0,10/255.0,1));
						//colors_ball->push_back(osg::Vec4(206/255.0,206/255.0,10/255.0,1));
						colors_ball->push_back(osg::Vec4(206/255.0,206/255.0,10/255.0,forward_rays_alpha_channel));
						colors_ball->push_back(osg::Vec4(206/255.0,206/255.0,10/255.0,forward_rays_alpha_channel));
						normals_ball->push_back(normalRayVertexDir);
						normals_ball->push_back(normalRayVertexDir);

						vertices_ball->push_back(lens_near_v3);
						vertices_ball->push_back(lens_far_v3);
						normals_ball->push_back(up);
						//colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,1));
						//colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,1));
						colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,forward_rays_alpha_channel));
						colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,forward_rays_alpha_channel));
						normals_ball->push_back(normalRayVertexDir);
						normals_ball->push_back(normalRayVertexDir);

						osg::Vec3 endpoint = lens_far_v3+lens_refracted_v3*50;

						vertices_ball->push_back(lens_far_v3);
						vertices_ball->push_back(endpoint);
						normals_ball->push_back(up);
						//colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,206/255.0,1));
						//colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,206/255.0,1));
						colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,206/255.0,forward_rays_alpha_channel));
						colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,206/255.0,forward_rays_alpha_channel));
						normals_ball->push_back(normalRayVertexDir);
						normals_ball->push_back(normalRayVertexDir);

						/*vertices_ball->push_back(tposv3);
						vertices_ball->push_back(tposv3+osg::Vec3(ball_lens_ray.x,ball_lens_ray.y,ball_lens_ray.z)*10000);
						colors_ball->push_back(osg::Vec4(255/255.0,0/255.0,0/255.0,1));
						colors_ball->push_back(osg::Vec4(255/255.0,0/255.0,0/255.0,1));*/

						if(exportMotion)
						{
							fprintf(fileRaytracePoints,"%g %g %g %g %g %g\n",tposv3.x(), tposv3.y(), tposv3.z(), cornea_near_v3.x(), cornea_near_v3.y(), cornea_near_v3.z());
							fprintf(fileRaytracePoints,"%g %g %g %g %g %g\n",cornea_near_v3.x(), cornea_near_v3.y(), cornea_near_v3.z(), cornea_far_v3.x(), cornea_far_v3.y(), cornea_far_v3.z());
							fprintf(fileRaytracePoints,"%g %g %g %g %g %g\n",cornea_far_v3.x(), cornea_far_v3.y(), cornea_far_v3.z(), pupil_near_v3.x(), pupil_near_v3.y(), pupil_near_v3.z());
							fprintf(fileRaytracePoints,"%g %g %g %g %g %g\n",pupil_near_v3.x(), pupil_near_v3.y(), pupil_near_v3.z(), lens_near_v3.x(), lens_near_v3.y(), lens_near_v3.z());
							fprintf(fileRaytracePoints,"%g %g %g %g %g %g\n",lens_near_v3.x(), lens_near_v3.y(), lens_near_v3.z(), lens_far_v3.x(), lens_far_v3.y(), lens_far_v3.z());
							fprintf(fileRaytracePoints,"%g %g %g %g %g %g\n",lens_far_v3.x(), lens_far_v3.y(), lens_far_v3.z(), endpoint.x(), endpoint.y(), endpoint.z());								
						}
						}
#endif
						lens_refracted.normalize();

						gVec3 origin_a(lens_far.x,lens_far.y,lens_far.z);
						//gVec3 origin_b(SysLEye->lens.center.x,SysLEye->lens.center.y,SysLEye->lens.center.z);
						gVec3 origin_b(orig_local.x,orig_local.y,orig_local.z);

						gVec3 direction_a(lens_refracted.x,lens_refracted.y,lens_refracted.z);
						//gVec3 direction_b(forward.x(),forward.y(),forward.z());
						//gVec3 direction_b((eye_rot*forward).x(),(eye_rot*forward).y(),(eye_rot*forward).z());
						gVec3 direction_b(ball_lens_ray.x,ball_lens_ray.y,ball_lens_ray.z);

						gVec3 intersection;

						if(lens_far.z > maxz.z) {
							maxz = lens_far;
							step_max = lens_refracted;
						}

						if((lens_far.z - base->frame().trn().z()) < 2.0 && compute_vector_intersection(origin_a,origin_b,direction_a,direction_b,intersection))
						{
							Vec3f_local int_point(intersection.x(),intersection.y(),intersection.z());
							final_box_int_points.push_back(int_point);
						}

						/*Vec3f_local near_int,far_int;
						if(intersection_plane.intersect(lens_far, lens_refracted, near_int, far_int))
						{
							Vec3f_local int_point = (near_int + far_int) * 0.5;
							final_box_int_points.push_back(int_point);
						}*/

						/*else // in case the prior exact analytical method fails
						{
							cout << "analytical method failed" << endl;
							Vec3f_local diff = lens_refracted - eye_center;
							Box_local intersection_plane = (abs(diff.x) > abs(diff.z)) ? vertical_plane : horizontal_plane; // determine if intersecting with horizontal or vertical plane

							Vec3f_local near_int,far_int;
							intersection_plane.intersect(lens_far, lens_refracted, near_int, far_int);
							Vec3f_local int_point = (near_int + far_int) * 0.5;

							final_box_int_points.push_back(int_point);
						}*/
					}
				}
			}
		}
	}

	double steps = abs(maxz.z - base->frame().trn().z()) / abs(step_max.z);
	Vec3f_local approxim_foc = Vec3f_local(base->frame().trn().x(), maxz.y, base->frame().trn().z()) + Vec3f_local(0,1,0) * steps;

	Vec3f_local summation_points;
	for(std::vector<Vec3f_local>::iterator it = final_box_int_points.begin(); it != final_box_int_points.end(); ++it) {
		summation_points += (*it);
	}
	double multiplier = 1.0 / ((double) final_box_int_points.size());
	Vec3f_local consensus_intersection_point = summation_points * multiplier;

	//cout << "consensus: " << consensus_intersection_point.x << " " << consensus_intersection_point.y << " " << consensus_intersection_point.z << endl;

#ifdef SHOW_RAYS
	osg::Vec3 int_point_v3(consensus_intersection_point.x,consensus_intersection_point.y,consensus_intersection_point.z);
	osg::Vec3 approxim_foc_v3(approxim_foc.x,approxim_foc.y,approxim_foc.z);
	osg::Vec3 approxim_foc_v3_1(approxim_foc.x,approxim_foc.y+0.1,approxim_foc.z);
	osg::Vec3 approxim_foc_v3_2(approxim_foc.x,approxim_foc.y-0.1,approxim_foc.z);
	osg::Quat x_marks_the_spot_rot_1(0, osg::X_AXIS, 45.0*gDTR, osg::Y_AXIS, 0, osg::Z_AXIS);
	osg::Quat x_marks_the_spot_rot_2(0, osg::X_AXIS, -45.0*gDTR, osg::Y_AXIS, 0, osg::Z_AXIS);
	//draw ray trace lines
	if(show_forward_trace_rays) {
	/*vertices_ball->push_back(int_point_v3-(eye_rot*up)*200);
	vertices_ball->push_back(int_point_v3+(eye_rot*up)*200);
	colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));
	colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));*/

	/*vertices_ball->push_back(int_point_v3_rt-(eye_rot*up)*200);
	vertices_ball->push_back(int_point_v3_rt+(eye_rot*up)*200);
	colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,1));
	colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,1));*/

	double xheight = 15.0;

	osg::Vec3 farXlower1 = approxim_foc_v3_1-(x_marks_the_spot_rot_1*eye_rot*up)*xheight;
	osg::Vec3 farXupper1 = approxim_foc_v3_1+(x_marks_the_spot_rot_1*eye_rot*up)*xheight;
	osg::Vec3 farXlower2 = approxim_foc_v3_1-(x_marks_the_spot_rot_2*eye_rot*up)*xheight;
	osg::Vec3 farXupper2 = approxim_foc_v3_1+(x_marks_the_spot_rot_2*eye_rot*up)*xheight;

	osg::Vec3 nearXlower1 = approxim_foc_v3_2-(x_marks_the_spot_rot_1*eye_rot*up)*xheight;
	osg::Vec3 nearXupper1 = approxim_foc_v3_2+(x_marks_the_spot_rot_1*eye_rot*up)*xheight;
	osg::Vec3 nearXlower2 = approxim_foc_v3_2-(x_marks_the_spot_rot_2*eye_rot*up)*xheight;
	osg::Vec3 nearXupper2 = approxim_foc_v3_2+(x_marks_the_spot_rot_2*eye_rot*up)*xheight;

	vertices_ball->push_back(farXlower1);
	vertices_ball->push_back(farXupper1);
	vertices_ball->push_back(farXlower2);
	vertices_ball->push_back(farXupper2);
	vertices_ball->push_back(nearXlower1);
	vertices_ball->push_back(nearXupper1);
	vertices_ball->push_back(nearXlower2);
	vertices_ball->push_back(nearXupper2);
	//colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,1));
	//colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,1));
	colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,forward_rays_alpha_channel));
	colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,forward_rays_alpha_channel));
	colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,forward_rays_alpha_channel));
	colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,forward_rays_alpha_channel));
	colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,forward_rays_alpha_channel));
	colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,forward_rays_alpha_channel));
	colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,forward_rays_alpha_channel));
	colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,forward_rays_alpha_channel));
	normals_ball->push_back(normalRayVertexDir);
	normals_ball->push_back(normalRayVertexDir);
	normals_ball->push_back(normalRayVertexDir);
	normals_ball->push_back(normalRayVertexDir);
	normals_ball->push_back(normalRayVertexDir);
	normals_ball->push_back(normalRayVertexDir);
	normals_ball->push_back(normalRayVertexDir);
	normals_ball->push_back(normalRayVertexDir);

	if(exportMotion)
	{
		fprintf(fileRaytracePoints,"%g %g %g %g %g %g\n",farXlower1.x(), farXlower1.y(), farXlower1.z(), farXupper1.x(), farXupper1.y(), farXupper1.z());
		fprintf(fileRaytracePoints,"%g %g %g %g %g %g\n",farXlower2.x(), farXlower2.y(), farXlower2.z(), farXupper2.x(), farXupper2.y(), farXupper2.z());
		fprintf(fileRaytracePoints,"%g %g %g %g %g %g\n",nearXlower1.x(), nearXlower1.y(), nearXlower1.z(), nearXupper1.x(), nearXupper1.y(), nearXupper1.z());
		fprintf(fileRaytracePoints,"%g %g %g %g %g %g\n",nearXlower2.x(), nearXlower2.y(), nearXlower2.z(), nearXupper2.x(), nearXupper2.y(), nearXupper2.z());						
		
		fflush(fileRaytracePoints);
	}

	/*vertices_ball->push_back(fp_v3-(eye_rot*up)*200);
	vertices_ball->push_back(fp_v3+(eye_rot*up)*200);
	colors_ball->push_back(osg::Vec4(10/255.0,10/255.0,206/255.0,1));
	colors_ball->push_back(osg::Vec4(10/255.0,10/255.0,206/255.0,1));

	vertices_ball->push_back(ip_v3-(eye_rot*up)*200);
	vertices_ball->push_back(ip_v3+(eye_rot*up)*200);
	colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,1));
	colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,1));*/

	/*vertices_ball->push_back(osg_eye_center+(eye_rot*forward)*3000);
	vertices_ball->push_back(osg_eye_center-(eye_rot*forward)*3000);
	colors_ball->push_back(osg::Vec4(206/255.0,206/255.0,206/255.0,1));
	colors_ball->push_back(osg::Vec4(206/255.0,206/255.0,206/255.0,1));*/
	}
#endif

#ifdef SHOW_LENS_SPHERES
	osg::Vec3 sphere_near_center_v3(SysLEye->lens.sphere1.center.x,SysLEye->lens.sphere1.center.y,SysLEye->lens.sphere1.center.z);
	osg::Vec3 sphere_far_center_v3(SysLEye->lens.sphere2.center.x,SysLEye->lens.sphere2.center.y,SysLEye->lens.sphere2.center.z);
	double radius_near = SysLEye->lens.sphere1.radius;
	double radius_far = SysLEye->lens.sphere2.radius;

	for (double zrot = 0.0; zrot < 180.0; zrot += 45.0) {
		for (int xrot = 0.0; xrot < 180.0; xrot += 45.0) {
			osg::Quat steprot(xrot*gDTR, osg::X_AXIS, 0, osg::Y_AXIS, zrot*gDTR, osg::Z_AXIS);
			osg::Vec3 fin_dir = steprot*eye_rot*forward;
			fin_dir.normalize();

			vertices_ball->push_back(sphere_near_center_v3-fin_dir*radius_near);
			vertices_ball->push_back(sphere_near_center_v3+fin_dir*radius_near);
			colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));
			colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));

			vertices_ball->push_back(sphere_far_center_v3-fin_dir*radius_far);
			vertices_ball->push_back(sphere_far_center_v3+fin_dir*radius_far);
			colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));
			colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));
		}
	}
	
#endif
	
#if defined(SHOW_RAYS) || defined(SHOW_LENS_SPHERES)
		//draw ray trace lines
		osg::ref_ptr<osg::Geometry> geom_active = new osg::Geometry;
		geom_active->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, vertices_ball->size()));
		geom_active->setUseDisplayList(false);

		geom_active->setVertexArray(vertices_ball);
		geom_active->setColorArray(colors_ball);
		geom_active->setNormalArray(normals_ball);
		geom_active->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		geom_active->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

		osg::ref_ptr<osg::Geode> geode_active = new osg::Geode;
		geode_active->addDrawable(geom_active.get());

		osg::StateSet* state_active = geode_active->getOrCreateStateSet();
		state_active->setMode(GL_LIGHTING, osg::StateAttribute::ON);
		state_active->setMode(GL_BLEND, osg::StateAttribute::ON);
		state_active->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
		osg::LineWidth* lw_active = new osg::LineWidth;
		lw_active->setWidth(1.5);
		state_active->setAttribute(lw_active, osg::StateAttribute::ON);
		osg::BlendFunc* blendfunc_active = new osg::BlendFunc();
		state_active->setAttributeAndModes(blendfunc_active, osg::StateAttribute::ON);

		pat_raytrace_lines = new osg::PositionAttitudeTransform();
		pat_raytrace_lines->addChild(geode_active);

		if(show_full_ball_trace) {
			osg::ref_ptr<osg::Geometry> geom_active1 = new osg::Geometry;
			geom_active1->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, vertices_ball_allrays->size()));
			geom_active1->setUseDisplayList(false);

			geom_active1->setVertexArray(vertices_ball_allrays);
			geom_active1->setColorArray(colors_ball_allrays);
			geom_active1->setNormalArray(normals_ball_allrays);
			geom_active1->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
			geom_active1->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

			osg::ref_ptr<osg::Geode> geode_active1 = new osg::Geode;
			geode_active1->addDrawable(geom_active1.get());

			osg::StateSet* state_active1 = geode_active1->getOrCreateStateSet();
			state_active1->setMode(GL_LIGHTING, osg::StateAttribute::ON);
			state_active1->setMode(GL_BLEND, osg::StateAttribute::ON);
			state_active1->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
			state_active1->setRenderBinDetails(5,"DepthSortedBin");
			osg::LineWidth* lw_active1 = new osg::LineWidth;
			lw_active1->setWidth(0.01);
			state_active->setAttribute(lw_active1, osg::StateAttribute::ON);
			osg::BlendFunc* blendfunc_active1 = new osg::BlendFunc();
			state_active1->setAttributeAndModes(blendfunc_active1, osg::StateAttribute::ON);
			pat_raytrace_lines->addChild(geode_active1);
		}

		pat_raytrace_lines->setPosition(osg::Vec3d(0.0, 0, 0.0));

		if(focalPointLineGroup->getNumChildren()>0)
			focalPointLineGroup->removeChild(0,1);
		focalPointLineGroup->addChild(pat_raytrace_lines);
#endif

	return approxim_foc;
	//return consensus_intersection_point;
}




void raytracer(vector<Vec3f_local> spos, float radius, Vec3f_local ray_orig, Vec3f_local ray_di, Vec3f_local* returnedImagePupil, Vec3f_local* returnedImageLens, Vec3f_local* returnedImageFoveation, int side)
{
	std::vector<Sphere_local> Sphere_locals;

	//Vec3f_local(0.00, 0.00, 0.00)
	for(std::vector<Vec3f_local>::iterator it = spos.begin(); it != spos.end(); ++it) {
		Sphere_locals.push_back(Sphere_local( *it, radius, Vec3f_local(vis_target_ec), 0, 0.0, Vec3f_local(vis_target_ec)));
	}
	//Sphere_locals.push_back(Sphere_local( pos, radius, Vec3f_local(vis_target_ec), 0, 0.0, Vec3f_local(vis_target_ec)));

#ifdef OLD_RETINA // If want faster speed
    render(Sphere_locals, ray_orig, ray_di, returnedImage, side);
#else
	render_ultimate(Sphere_locals, ray_orig, ray_di, returnedImagePupil, returnedImageLens, returnedImageFoveation, side);

#ifdef RECORD_PUPIL
	if (start_recording_pupil) {
		static int iteration = 0;
		static double prev_activation = 0.0;
		double ideal_utility = 25830.0;

		double currentAct = SysLEye->muscle(6)->actLevel();
		gVec3 ballpos = visualTarget.get()->getMatrix().getTrans().ptr();
		if(!final_record_iteration_pupil) {
			for(int i = 0; i< RAYTRACE_WIDTH*RAYTRACE_HEIGHT; i++) {
				finputpupil << returnedImage[i].x << ",";
			}
			finputpupil << currentAct << endl;
			finputpupilact << currentAct << endl;

			foutputpupilballpos << ballpos.x() << "," << ballpos.y() << "," << ballpos.z() << "," << vis_target_ec << endl;

			foutputpupilrandomlensact << SysLEye->muscle(14)->actLevel() << endl;
			foutputpupilutility << pupil_utility << "," << pupil_utility-ideal_utility << endl;

			foutputpupilsize << SysLEye->pupil.radius << endl;
		}

		iteration++;
		if (iteration > 1)
		{
			foutputpupil << currentAct - prev_activation << endl;
			foutputpupil.flush();
		}
		prev_activation = SysLEye->muscle(6)->actLevel();

		finputpupil.flush();
		finputpupilact.flush();
		foutputpupilballpos.flush();
		foutputpupilrandomlensact.flush();
		foutputpupilutility.flush();
		foutputpupilsize.flush();
	}
#endif

#ifdef RECORD_LENS
	if (start_recording_lens) {
		/*static int iteration = 0;
		static double prev_activation = 0.0;

		double currentAct = SysLEye->muscle(14)->actLevel();
		gVec3 ballpos = visualTarget.get()->getMatrix().getTrans().ptr();
		if(!final_record_iteration_lens && lens_error == lens_error) {
			for(int i = 0; i< RAYTRACE_WIDTH*RAYTRACE_HEIGHT; i++) {
				finputlens << returnedImage[i].x << ",";
			}
			finputlens << currentAct << endl;
			finputlensact << currentAct << endl;

			foutputpupilrandompupilact << SysLEye->muscle(6)->actLevel() << endl;

			foutputlensballpos << ballpos.x() << "," << ballpos.y() << "," << ballpos.z() << "," << vis_target_ec << endl;
		}
		
		iteration++;
		if (iteration > 1)
		{
			//foutputlens << lens_error << endl;
			foutputlens << currentAct - prev_activation << endl;
			foutputlens.flush();
		}

		prev_activation = SysLEye->muscle(14)->actLevel();*/

		gVec3 ballpos = visualTarget.get()->getMatrix().getTrans().ptr();
		if(record_lens_data) {
			for(int i = 0; i< RAYTRACE_WIDTH*RAYTRACE_HEIGHT; i++) {
				finputlens << returnedImage[i].x << ",";
			}
			finputlens << SysLEye->muscle(14)->actLevel() << endl;
			finputlensact << SysLEye->muscle(14)->actLevel() << endl;
			foutputlens << lens_error << endl;

			foutputpupilrandompupilact << SysLEye->muscle(6)->actLevel() << endl;

			foutputlensballpos << ballpos.x() << "," << ballpos.y() << "," << ballpos.z() << "," << vis_target_ec << endl;
		}

		finputlens.flush();
		finputlensact.flush();
		foutputlensballpos.flush();
		foutputpupilrandompupilact.flush();
		foutputlens.flush();
	}
#endif

				
#ifdef RECORD_FOVEATION
	if (start_recording_foveation) {
		gLink* base = SysLEye->findLink("Eyeball");
		gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
		gVec3 localTargetPos = targetWorldPos-base->frame().trn();

		gVec3 ballpos = visualTarget.get()->getMatrix().getTrans().ptr();

		double theta = theta_vec_conversion(localTargetPos);
		double phi = -phi_vec_conversion(localTargetPos);
		for(int i = 0; i < (RAYTRACE_WIDTH*RAYTRACE_HEIGHT)-1; i++) {
			finputperception << returnedImageFoveation[i].x << ",";
		}
		finputperception << returnedImageFoveation[(RAYTRACE_WIDTH*RAYTRACE_HEIGHT)-1].x << endl;
		foutputperception << theta << "," << phi << endl;
		foutputperceptionballpos << ballpos.x() << "," << ballpos.y() << "," << ballpos.z() << "," << vis_target_ec << endl;
		foutputpupilrandompupilactperception << SysLEye->pupil.radius << endl;
		foutputpupilrandomlensactperception << SysLEye->lens.width << endl;
		finputperception.flush();
		foutputperception.flush();
		foutputpupilrandompupilactperception.flush();
		foutputpupilrandomlensactperception.flush();
	}
#endif

#endif
}

void createEyeLine()
{
	eyeLineGroup = new osg::Group();
	sceneGroup->addChild(eyeLineGroup);
	eyeLineGroupRight = new osg::Group();
	sceneGroup->addChild(eyeLineGroupRight);
}

void createRaytraceLine()
{
	raytraceLineGroup = new osg::Group();
	sceneGroup->addChild(raytraceLineGroup);

}

void createFocalPointLine()
{
	focalPointLineGroup = new osg::Group();
	sceneGroup->addChild(focalPointLineGroup);

}



void drawTargetBox(){
	//if(visualTarget)
	//	pat->setPosition(visualTarget.get()->getMatrix().getTrans()); /////osg use z up coordinate, x is positive toward right
	//else
	//	pat->setPosition(osg::Vec3d(0.0, 3.98, 40.0));
	if(visualTarget)
		pat2->setPosition(visualTarget.get()->getMatrix().getTrans()); /////osg use z up coordinate, x is positive toward right
	else
		pat2->setPosition(osg::Vec3d(0.0, 100, 1033.0));
		//pat2->setPosition(osg::Vec3d(0.0, 3.98, 40.0));

}
void drawTargetBoxRight(){
	//if(visualTarget)
	//	pat->setPosition(visualTarget.get()->getMatrix().getTrans()); /////osg use z up coordinate, x is positive toward right
	//else
	//	pat->setPosition(osg::Vec3d(0.0, 3.98, 40.0));
	if(visualTarget)
		pat2Right->setPosition(visualTarget.get()->getMatrix().getTrans()); /////osg use z up coordinate, x is positive toward right
	else
		pat2Right->setPosition(osg::Vec3d(0.0, 100, 1033.0));
		//pat2->setPosition(osg::Vec3d(0.0, 3.98, 40.0));

}

void computeThetaPhi()
{
	drawTargetBox();

	gVec3 ori_leye = getcurrent_ori_leye();

	gLink* base;
	base = SysLEye->findLink("Eyeball");

	osg::Vec3 eye_direction_vec(0,-1,0);
	osg::Quat q2(eye_cxl, osg::X_AXIS, 0, osg::Y_AXIS, eye_cyl, osg::Z_AXIS);
	//osg::Vec3 eye_direction_vec_cl = q2*(eye_direction_vec);
	eye_direction_vec = q2*(eye_direction_vec);

	osg::Vec3 eye_direction_vec_new(base->frame().trn().x()+eye_direction_vec.x(),base->frame().trn().y()+eye_direction_vec.y(),base->frame().trn().z()+eye_direction_vec.z());
	
	osg::Vec3 eye_up_vec(0,0,1);

	sceneViewLeft->setViewMatrixAsLookAt(
		osg::Vec3(base->frame().trn().x(),base->frame().trn().y(),base->frame().trn().z()), // eye pos

		eye_direction_vec_new,    // gaze at 

		eye_up_vec);   // usual up vector//somehow this up vector does not affect.
	osg::Vec3 eye, centre, up;
	sceneViewLeft->getViewMatrixAsLookAt(eye,centre,up);

	if(visualTarget.get()){
		 

		//raytrace_counter++;
		//raytrace_counter = 0;
		osg::Vec3d spherePos1 = visualTarget.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local1 = Vec3f_local(float(spherePos1.x()),float(spherePos1.y()),float(spherePos1.z()));
		
		vector<Vec3f_local> spehere_poss;
		spehere_poss.push_back(sherePos_local1);
#ifdef SECOND_VISUAL_TARGET
		osg::Vec3d spherePos2 = visualTarget2.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local2 = Vec3f_local(float(spherePos2.x()),float(spherePos2.y()),float(spherePos2.z()));
		spehere_poss.push_back(sherePos_local2);
#endif

		Vec3f_local returnedImagePupil[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];
		Vec3f_local returnedImageLens[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];
		Vec3f_local returnedImageFoveation[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];
		//raytracer(sherePos_local, SPHERE_SIZE, Vec3f_local(eye.x(),eye.y(), eye.z()), Vec3f_local(ori_leye.x()*gDTR,0,ori_leye.z()*gDTR),returnedImage,0);
		raytracer(spehere_poss, SPHERE_SIZE, Vec3f_local(eye.x(),eye.y(), eye.z()), Vec3f_local(ori_leye.x()*gDTR,0,ori_leye.z()*gDTR),returnedImagePupil,returnedImageLens,returnedImageFoveation,0);

#ifdef ONLINE_FOVEATION
		vector<float> ray_vec;
#endif
#ifdef ONLINE_PUPIL
		vector<float> ray_pupil;
#endif
#ifdef ONLINE_LENS
		vector<float> ray_lens;
		static double prev_lens_act = 0;
#endif
		bool targetExist = false;

		active_idx_pupil.clear();
		active_idx_lens.clear();
		active_idx_foveation.clear();

		active_idx_color_pupil.clear();
		active_idx_color_lens.clear();
		active_idx_color_foveation.clear();

		double maxX = 0.0;
		double maxY = 0.0;
		double maxZ = 0.0;
		for(int i = 0; i< RAYTRACE_WIDTH*RAYTRACE_HEIGHT; i++)
			{
				Vec3f_local vp = returnedImagePupil[i];
				Vec3f_local vl = returnedImageLens[i];
				Vec3f_local vf = returnedImageFoveation[i];

				if(vp.x>0.0 || vp.y>0.0 || vp.z>0.0)
				{
					targetExist = true;
					active_idx_pupil.push_back(i);
				}

				if(vl.x>0.0 || vl.y>0.0 || vl.z>0.0)
				{
					targetExist = true;
					active_idx_lens.push_back(i);
				}

				if(vf.x>0.0 || vf.y>0.0 || vf.z>0.0)
				{
					targetExist = true;
					active_idx_foveation.push_back(i);
				}

				active_idx_color_pupil.push_back(vp);
				active_idx_color_lens.push_back(vl);
				active_idx_color_foveation.push_back(vf);

#ifdef ONLINE_FOVEATION
				ray_vec.push_back(v.x);
#endif
#ifdef ONLINE_PUPIL
				ray_pupil.push_back(v.x);
#endif
#ifdef ONLINE_LENS
				ray_lens.push_back(v.x);
#endif
			}
		
		double tyl, txl;
		static double prior_lens_error = FLT_MAX;
		static int prior_step_lens = 0;
		static bool firstTimeLens = true;
 		if(targetExist){
#ifdef ONLINE_FOVEATION
			//cout << "current estimated theta, phi (degrees): " << eye_cyl*gRTD << " " << eye_cxl*gRTD << endl;
			sampleRayKeras->set_data(ray_vec);
			vector<float> returned = mRayKeras->compute_output(sampleRayKeras);
			ray_vec.clear();
#endif

#ifdef ONLINE_PUPIL
			//cout << "current act pupil: " << SysLEye->muscle(6)->actLevel() << endl;
			ray_pupil.push_back(SysLEye->muscle(6)->actLevel());
			samplePupilKeras->set_data(ray_pupil);
			vector<float> returnedPupil = mPupilKeras->compute_output(samplePupilKeras);
			ray_pupil.clear();
#endif

#ifdef ONLINE_LENS
			
			//cout << "current act lens: " << SysLEye->muscle(14)->actLevel() << endl;
			//cout << "current act lens38: " << SysLEye->muscle(38)->actLevel() << endl;
			//cout << "current act lens: " << prev_lens_act << endl;
			ray_lens.push_back(SysLEye->muscle(14)->actLevel());
			//ray_lens.push_back(prev_lens_act);
			sampleLensKeras->set_data(ray_lens);
			/*for(int i=0; i< ray_lens.size(); i++){
				if(ray_lens[i]>0)
					cout<< ray_lens[i];
			}*/
			vector<float> returnedLens = mLensKeras->compute_output(sampleLensKeras);
			ray_lens.clear();
#endif

#ifdef ONLINE_FOVEATION
			tyl = returned[0]*stdValueLeft[0] + meanValueLeft[0] + eye_cyl;
			txl = returned[1]*stdValueLeft[1] + meanValueLeft[1] + eye_cxl;
			tyl_temp = tyl;
			txl_temp = txl;
			//cout << "new estimated raw theta, phi (degrees): " << tyl*gRTD << " " << txl*gRTD << endl;
#endif

#ifdef ONLINE_PUPIL
			double newActPupil = returnedPupil[0]*stdValuePupil[0] + meanValuePupil[0] + SysLEye->muscle(6)->actLevel();
			//newActPupil = 0.1;
			//cout << "new attempted act pupil: " << newActPupil << endl;

			double ideal_utility = 25830.0;
			double pupil_utility_diff = abs(pupil_utility - ideal_utility);
			//cout << "utility: " << pupil_utility << endl;
			//cout << "diff: " << pupil_utility_diff << endl;
			if(bPlay) {
				for(int j=6; j<14; j++){
					SysLEye->setActLevel(j,gMin(1.0, gMax(newActPupil,0)));
				}
			}
#endif

#ifdef ONLINE_LENS

			//double newActLens = returnedLens[0]*stdValueLens[0] + meanValueLens[0] + SysLEye->muscle(14)->actLevel();
			//double newActLens = returnedLens[0] + SysLEye->muscle(14)->actLevel();
			/*double newActLens = returnedLens[0] + SysLEye->muscle(14)->actLevel();
			prev_lens_act = newActLens;*/
			//cout << "new attempted act lens: " << newActLens << endl;
			
			double newActLens;
			if(bPlay) {
				double currLensEstimatedError = returnedLens[0]*stdValueLens[0] + meanValueLens[0];
				cout << "error severity, prior: " << currLensEstimatedError << " " << prior_lens_error << endl;
				double diff = (currLensEstimatedError >= 0) ? currLensEstimatedError - prior_lens_error : 0;
				//double diff = currLensEstimatedError - prior_lens_error;
				double actDelta;
				if(firstTimeLens) {
					//actDelta = (diff < 0) ? 1 : -1;
					//actDelta = (rand() % 2 == 1) ? 1 : -1;
					actDelta = 1;
					firstTimeLens = false;
				}
				else {
					cout << "diff, prior step: " << diff << " " << prior_step_lens << endl;
					actDelta = (diff < 0) ? prior_step_lens : -prior_step_lens;
				}
				cout << "act Delta: " << actDelta << endl;

				if(currLensEstimatedError > 0.7) {
					if(actDelta > 0 && SysLEye->muscle(14)->actLevel() <= 0.4) {
						newActLens = 8*actDelta*0.02 + SysLEye->muscle(14)->actLevel();
					}
					else if(actDelta < 0 && SysLEye->muscle(14)->actLevel() >= 0.8) {
						newActLens = 5*actDelta*0.02 + SysLEye->muscle(14)->actLevel();
					}
					else {
						newActLens = actDelta*0.02 + SysLEye->muscle(14)->actLevel();
					}
				}
				else if(currLensEstimatedError > 0.0) {
					double alpha = 0.01;
					double beta = 1.1;

					double cilliary_act_adj = min(alpha*(exp(beta*currLensEstimatedError) - 1.0), 0.05);
					newActLens = actDelta*cilliary_act_adj + SysLEye->muscle(14)->actLevel();
				}
				else {
					newActLens = actDelta*0.04 + SysLEye->muscle(14)->actLevel();
				}
				cout << "desired act lens: " << newActLens << endl;

				prior_lens_error = currLensEstimatedError;
				prior_step_lens = actDelta;
			}
			else {
				newActLens = 0;
			}

			//newActLens = 1.0;
			//maybeRecordBallPos();
			//newActLens =0.0;
			
			double newActLens_passive = 0.072*(1-newActLens);
			for(int j = 14; j < 22; j++) {
				SysLEye->setActLevel(j,gMin(1.0, gMax(newActLens,0)));
			}
			for(int j = 38; j < 54; j++) {
				SysLEye->setActLevel(j,gMin(1.0, gMax(newActLens_passive,0)));
			}

			/*double newActLens = returnedLens[0];
			adjust_lens_cilliary_muscles_trained_dnn(newActLens);*/
#endif

		}else{

#ifdef CONTROL_LEGS
			tyl = 0;
			txl = 30*gDTR;
#else
			if (counter_start_L < counter_total)
			{
				tyl = tyl_temp * (counter_total - counter_start_L) / double(counter_total);
				txl = txl_temp * (counter_total - counter_start_L) / double(counter_total);
				counter_start_L++;
			}
			else{
				tyl = 0.000f;
				txl = 0.000f;
			}
			
			//txl = 0*gDTR;
#endif
		}

		cout << "nn tyl, txl: " << tyl*gRTD << " " << txl*gRTD << endl;
		cout << "actual tyl, txl: " << theta_vec_conversion(targeting_location)*gRTD << " " << -phi_vec_conversion(targeting_location)*gRTD << endl;

		double angle = atan2(tyl-eye_cyl,txl-eye_cxl);
		double gaze_error = sqrt((txl-eye_cxl)*(txl-eye_cxl)+(tyl-eye_cyl)*(tyl-eye_cyl));
			
		double param=1.0;
		/*if(gaze_error>0.3)
			param *= 1;
		else if(gaze_error>0.2)
			param *= 0.5;
		else if(gaze_error>0.1)
			param *= 0.1;
		else if(gaze_error>0.05)
			param *= 0.01;
		else
			param *= 0.001;*/

		param = 1.0 / (1.0 + exp(-10*(gaze_error - 0.38)));

		//param=1.0;
		double maxEyeVelocity = 3.0;//*10;
		double maxEyeLength = 0.610865*10;
		
		double timestep = displayTimeStep;
		double len = gMin(gaze_error, maxEyeVelocity*timestep*param);
	
		eye_cxl += len*cos(angle);
		eye_cyl += len*sin(angle);
		angle = atan2(eye_cyl,eye_cxl);
		len = gMin( sqrt(eye_cxl*eye_cxl+eye_cyl*eye_cyl), maxEyeLength);
	
		eye_cxl = len*cos(angle);
		eye_cyl = len*sin(angle);

		//eye_cxl = txl;
		//eye_cyl = tyl;
	}
}

void displayLeft(void)
{
	computeThetaPhi();

	sceneViewLeft->update();
	// do the cull traversal, collect all objects in the view frustum into a sorted set of rendering bins
	sceneViewLeft->cull();
	sceneViewLeft->draw();

	// Swap Buffers
	glutSwapBuffers();
    glutPostRedisplay();
}

//#define SHOW_PUPIL
#define SHOW_LENS
//#define SHOW_FOVEATION
void display_PR(void)
{	
	
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glClearColor (0.0f, 0.0f, 0.0f, 1.0f);
	//sceneView_PR->getCamera()->setClearColor(osg::Vec4(0.4f, 0.4f, 0.4f, 1.0f)); 
	sceneView_PR->getCamera()->setClearColor(osg::Vec4(0.878431, 0.521569, 0.313725, 1.0f)); 
	//sceneView_PR->getCamera()->setClearColor(osg::Vec4(1, 1, 1, 0.0f));
 
	//active photo receptor rendering with colors
	osg::Vec3Array* vertices_active = new osg::Vec3Array;
	osg::Vec4Array* colors_active = new osg::Vec4Array;
	osg::Vec3Array *normalArray = new Vec3Array();

	//int count = 0;
	vertices_active->setName("VertexActive");

	 //unsigned width = wind[MAIN_WINDOW].width, height = wind[MAIN_WINDOW].height;
    unsigned width =RAYTRACE_WIDTH, height = RAYTRACE_HEIGHT;
    Vec3f_local *image = new Vec3f_local[width * height], *pixel = image;
    float invWidth = 1 / float(width), invHeight = 1 / float(height);
    float fov = 65, aspectratio = width / float(height);

    //float fov = 240, aspectratio = width / float(height);
    float angle = tan(M_PI * 0.5 * fov / 180.);

	double ray_width = RAYTRACE_WIDTH;
	//double divisor = 5.0;
	double divisor = 10.0;
	double maxRadius = 2.0/3.0*std::exp(ray_width/divisor);
	//double maxRadius = 2.6 * std::exp(ray_width/5.0*40.0/RAYTRACE_WIDTH);
	int idx = 0;
	//5

		/*	for (unsigned rho = 0; rho < RAYTRACE_WIDTH; rho++) {
			// Change_by_honglin 3: the increment is 1 instead of 4
			for (unsigned thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT*1; thetaLogPolar+=1, ++pixel) {

				// Change_by_honglin 4: generate new noise.txt and change the increment from 4 to 1
				double logPolarX = 2.0/3.0* std::exp(double(rho/5.0*40.0/RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * cos(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
				double logPolarY  = 2.0/3.0* std::exp(double(rho/5.0*40.0/RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * sin(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
				*/
#ifdef OLD_RETINA
	for (unsigned rho = 0; rho < RAYTRACE_WIDTH; rho++) {
		for (unsigned thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT*4; thetaLogPolar+=4) {
				
			double logPolarX =2.0* std::exp(double(rho/5.0)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/4]*3) * cos(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/4]));
			double logPolarY =2.0* std::exp(double(rho/5.0)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/4]*3) * sin(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/4]));

			float yy = (2 * ((logPolarX) /maxRadius)-1) * fov*D2R/2;
			float xx = (2 * ((logPolarY) /maxRadius)-1) * fov*D2R/2;
#else
	
	for (int rho = 0; rho < RAYTRACE_WIDTH; rho++) {
			// Change_by_honglin 3: the increment is 1 instead of 4
			for (unsigned thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT*1; thetaLogPolar+=1) {

			// Change_by_honglin 4: generate new noise.txt and change the increment from 4 to 1
			//double logPolarX = 2.0/3.0* std::exp(double(rho/5.0*40.0/RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * cos(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
			//double logPolarY  = 2.0/3.0* std::exp(double(rho/5.0*40.0/RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * sin(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));

			// revert to 2 for 10
			double logPolarX;
			double logPolarY;
			if(rho < 30) {
				logPolarX = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0/3.0* std::exp(double(rho/divisor)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * cos(double(thetaLogPolar*DEG_TO_RAD+2*retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
				logPolarY  = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0/3.0* std::exp(double(rho/divisor)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * sin(double(thetaLogPolar*DEG_TO_RAD+2*retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
			}
			else {
				logPolarX = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0/3.0* std::exp(double(rho/divisor)+4*retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * cos(double(thetaLogPolar*DEG_TO_RAD+2*retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
				logPolarY = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0/3.0* std::exp(double(rho/divisor)+4*retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * sin(double(thetaLogPolar*DEG_TO_RAD+2*retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
			}
			//double logPolarX = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0/3.0* std::exp(double(rho/divisor)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * cos(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
			//double logPolarY  = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0/3.0* std::exp(double(rho/divisor)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * sin(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));

			//double logPolarX = 2.6 * std::exp(double(rho/5.0*46.9 /RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * cos(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
			//double logPolarY = 2.6 * std::exp(double(rho/5.0*46.9 /RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * sin(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));

			float yy = (1 * ((logPolarX) /maxRadius)) * fov*D2R;
			float xx = (1 * ((logPolarY) /maxRadius)) * fov*D2R;
#endif	

			osg::Vec3 eye_direction_vec3(0,1,0);
			osg::Quat q3(xx, osg::X_AXIS, 0, osg::Y_AXIS, yy, osg::Z_AXIS);
			osg::Vec3 osg_eye_temp3 = q3*(eye_direction_vec3);

			osg::Vec3 osg_eye_center(0,0,0);
			osg_eye_temp3.normalize();
			/*double randX = ((rho+thetaLogPolar)*100%99)/100.0;
			double randZ =((rho+thetaLogPolar)*100%88)/100.0;*/
			//osg::Vec3 raytrace_orig = osg_eye_center + osg_eye_temp3*EYE_SIZE;
			osg::Vec3 raytrace_orig(logPolarX/maxRadius*EYE_SIZE/2, EYE_SIZE/2, logPolarY/maxRadius*EYE_SIZE/2);
			vertices_active->push_back(raytrace_orig);
			
			bool isThisActive = false;

			vector<int> active_idx;
			vector<Vec3f_local> active_idx_color;

#ifdef SHOW_PUPIL
			active_idx = active_idx_pupil;
			active_idx_color = active_idx_color_pupil;
#endif
#ifdef SHOW_LENS
			active_idx = active_idx_lens;
			active_idx_color = active_idx_color_lens;
#endif
#ifdef SHOW_FOVEATION
			active_idx = active_idx_foveation;
			active_idx_color = active_idx_color_foveation;
#endif
			
			for(std::vector<int>::iterator it = active_idx.begin(); it != active_idx.end(); ++it) {
				if(*it==idx){
					isThisActive = true;
				}
			}

			
			if(isThisActive) {
				//colors_active->push_back(osg::Vec4(1.0,1.0,1.0,1));
				Vec3f_local cv = active_idx_color.at(idx);
				colors_active->push_back(osg::Vec4(cv.x,cv.y,cv.z,1));
			}
			else {
				colors_active->push_back(osg::Vec4(0.0,0.0,0.0,1));
			}

			idx++;

			
			normalArray->push_back(osg::Vec3(0, -1, 0));
		}
	}


	osg::ref_ptr<osg::Geometry> geom_active = new osg::Geometry;
    geom_active->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, vertices_active->size()));
    geom_active->setUseDisplayList(false);

    geom_active->setVertexArray(vertices_active);
    geom_active->setColorArray(colors_active);
	geom_active->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geom_active->setNormalArray(normalArray);
	geom_active->setNormalBinding(Geometry::BIND_PER_VERTEX);

    osg::ref_ptr<osg::Geode> geode_active = new osg::Geode;
    geode_active->addDrawable(geom_active.get());

    osg::StateSet* state_active = geode_active->getOrCreateStateSet();
    state_active->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    state_active->setMode(GL_BLEND, osg::StateAttribute::ON);
   // state_active->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
   // osg::LineWidth* lw_active = new osg::LineWidth;
    //lw_active->setWidth(10.f);
	osg::Point *point_size = new osg::Point(2.0f);
    state_active->setAttribute(point_size, osg::StateAttribute::ON);
    osg::BlendFunc* blendfunc_active = new osg::BlendFunc();
    state_active->setAttributeAndModes(blendfunc_active, osg::StateAttribute::ON);
	pat_PR_active = new osg::PositionAttitudeTransform();
	pat_PR_active->addChild(geode_active);
	
	pat_PR_active->setPosition(osg::Vec3d(0.0, 0, 0.0));
	//osg::Group *child = new osg::Group();
	sceneGroup_PR->removeChild(0,1);
	sceneGroup_PR->addChild(pat_PR_active);
	//sceneGroup_PR->removeChild(-1,1);
	
	sceneView_PR->update();
	// do the cull traversal, collect all objects in the view frustum into a sorted set of rendering bins
	sceneView_PR->cull();
	sceneView_PR->draw();

	// Swap Buffers
	glutSwapBuffers();
    glutPostRedisplay();


	if(dumpImages)
	{
		//{image dumping
		if(!imageDumpRetina){
			imageDumpRetina = new osg::Image;
			imageDumpRetina->allocateImage( wndWidthRetina,wndHeightRetina, 1, GL_RGB, GL_UNSIGNED_BYTE);
			system("rmdir /s /q dumpRetina");
			system("mkdir dumpRetina");
			//}
		}
		sprintf(imageNameRetina,"./dumpRetina/image%06d.bmp",dumpFrameRetina);
		imageDumpRetina->readPixels( 0, 0, wndWidthRetina,wndHeightRetina, GL_RGB, GL_UNSIGNED_BYTE);
		osgDB::writeImageFile( *(imageDumpRetina.get()), imageNameRetina ); //int,bmp,rgb
		dumpFrameRetina++;
	}
}

void display_PRTest(void)
{	
	
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glClearColor (0.0f, 0.0f, 0.0f, 1.0f);
	//sceneView_PR->getCamera()->setClearColor(osg::Vec4(0.4f, 0.4f, 0.4f, 1.0f)); 
	sceneView_PRTest->getCamera()->setClearColor(osg::Vec4(0.878431, 0.521569, 0.313725, 1.0f)); 
	//vector<float> active_idx = targetDetect->getActiveCellIdx();
 
	//active photo receptor rendering with colors
	//vector<float> active_idx = targetDetect->getActiveCellIdx();
	osg::Vec3Array* vertices_activeTest = new osg::Vec3Array;
	osg::Vec4Array* colors_activeTest = new osg::Vec4Array;
	osg::Vec3Array *normalArray = new Vec3Array();

	//int count = 0;
	vertices_activeTest->setName("VertexActiveTest");

	 //unsigned width = wind[MAIN_WINDOW].width, height = wind[MAIN_WINDOW].height;
    unsigned width =RAYTRACE_WIDTH, height = RAYTRACE_HEIGHT;
    Vec3f_local *image = new Vec3f_local[width * height], *pixel = image;
    float invWidth = 1 / float(width), invHeight = 1 / float(height);
    //float fov = 30, aspectratio = width / float(height);

    float fov = 240, aspectratio = width / float(height);
    float angle = tan(M_PI * 0.5 * fov / 180.);

	double ray_width = RAYTRACE_WIDTH;
	double divisor = 10.0;
	double maxRadius = 2.0/3.0*std::exp(ray_width/divisor);
	//double maxRadius = 2.6 * std::exp(ray_width/5.0*40.0/RAYTRACE_WIDTH);
	int idx = 0;
	//5

		/*	for (unsigned rho = 0; rho < RAYTRACE_WIDTH; rho++) {
			// Change_by_honglin 3: the increment is 1 instead of 4
			for (unsigned thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT*1; thetaLogPolar+=1, ++pixel) {

				// Change_by_honglin 4: generate new noise.txt and change the increment from 4 to 1
				double logPolarX = 2.0/3.0* std::exp(double(rho/5.0*40.0/RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * cos(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
				double logPolarY  = 2.0/3.0* std::exp(double(rho/5.0*40.0/RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * sin(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
				*/
#ifdef OLD_RETINA
	for (unsigned rho = 0; rho < RAYTRACE_WIDTH; rho++) {
		for (unsigned thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT*4; thetaLogPolar+=4) {
				
			double logPolarX =2.0* std::exp(double(rho/5.0)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/4]*3) * cos(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/4]));
			double logPolarY =2.0* std::exp(double(rho/5.0)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/4]*3) * sin(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/4]));

			float yy = (2 * ((logPolarX) /maxRadius)-1) * fov*D2R/2;
			float xx = (2 * ((logPolarY) /maxRadius)-1) * fov*D2R/2;
#else
	
	for (int rho = 0; rho < RAYTRACE_WIDTH; rho++) {
			// Change_by_honglin 3: the increment is 1 instead of 4
			for (unsigned thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT*1; thetaLogPolar+=1) {

			// Change_by_honglin 4: generate new noise.txt and change the increment from 4 to 1
			//double logPolarX = 2.0/3.0* std::exp(double(rho/5.0*40.0/RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * cos(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
			//double logPolarY  = 2.0/3.0* std::exp(double(rho/5.0*40.0/RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * sin(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));

			double logPolarX = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0/3.0* std::exp(double(rho/divisor)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * cos(double(thetaLogPolar*DEG_TO_RAD+2*retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
			double logPolarY  = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0/3.0* std::exp(double(rho/divisor)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * sin(double(thetaLogPolar*DEG_TO_RAD+2*retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));

			//double logPolarX = 2.6 * std::exp(double(rho/5.0*46.9 /RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * cos(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));
			//double logPolarY = 2.6 * std::exp(double(rho/5.0*46.9 /RAYTRACE_WIDTH)+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]) * sin(double(thetaLogPolar*DEG_TO_RAD+retina_distribution_noise[rho+RAYTRACE_WIDTH*thetaLogPolar/1]));

			float yy = (1 * ((logPolarX) /maxRadius)) * fov*D2R;
			float xx = (1 * ((logPolarY) /maxRadius)) * fov*D2R;
#endif	

			osg::Vec3 eye_direction_vec3(0,1,0);
			osg::Quat q3(xx, osg::X_AXIS, 0, osg::Y_AXIS, yy, osg::Z_AXIS);
			osg::Vec3 osg_eye_temp3 = q3*(eye_direction_vec3);

			osg::Vec3 osg_eye_center(0,0,0);
			osg_eye_temp3.normalize();
			/*double randX = ((rho+thetaLogPolar)*100%99)/100.0;
			double randZ =((rho+thetaLogPolar)*100%88)/100.0;*/
			//osg::Vec3 raytrace_orig = osg_eye_center + osg_eye_temp3*EYE_SIZE;
			osg::Vec3 raytrace_orig(logPolarX/maxRadius*EYE_SIZE/2, EYE_SIZE/2, logPolarY/maxRadius*EYE_SIZE/2);
			vertices_activeTest->push_back(raytrace_orig);

			//colors_activeTest->push_back(osg::Vec4(0.0,0.0,0.0,1));
#ifdef TEST_PUPIL
			double color = image_leye_pupil[rho*RAYTRACE_HEIGHT + thetaLogPolar];
#elif defined(TEST_LENS)
			double color = image_leye_lens[rho*RAYTRACE_HEIGHT + thetaLogPolar];
#elif defined(TEST_FOVEATION)
			double color = image_leye_perception[rho*RAYTRACE_HEIGHT + thetaLogPolar];
#else
			double color = 0.0;
#endif
			colors_activeTest->push_back(osg::Vec4(color,color,color,1));
			normalArray->push_back(osg::Vec3(0, -1, 0));
		}
	}


	osg::ref_ptr<osg::Geometry> geom_active = new osg::Geometry;
    geom_active->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, vertices_activeTest->size()));
    geom_active->setUseDisplayList(false);

    geom_active->setVertexArray(vertices_activeTest);
    geom_active->setColorArray(colors_activeTest);
	geom_active->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geom_active->setNormalArray(normalArray);
	geom_active->setNormalBinding(Geometry::BIND_PER_VERTEX);

    osg::ref_ptr<osg::Geode> geode_active = new osg::Geode;
    geode_active->addDrawable(geom_active.get());

    osg::StateSet* state_active = geode_active->getOrCreateStateSet();
    state_active->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    state_active->setMode(GL_BLEND, osg::StateAttribute::ON);
   // state_active->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
   // osg::LineWidth* lw_active = new osg::LineWidth;
    //lw_active->setWidth(10.f);
	osg::Point *point_size = new osg::Point(2.0f);
    state_active->setAttribute(point_size, osg::StateAttribute::ON);
    osg::BlendFunc* blendfunc_active = new osg::BlendFunc();
    state_active->setAttributeAndModes(blendfunc_active, osg::StateAttribute::ON);
	pat_PR_activeTest = new osg::PositionAttitudeTransform();
	pat_PR_activeTest->addChild(geode_active);
	
	pat_PR_activeTest->setPosition(osg::Vec3d(0.0, 0, 0.0));
	//osg::Group *child = new osg::Group();
	sceneGroup_PRTest->removeChild(0,1);
	sceneGroup_PRTest->addChild(pat_PR_activeTest);
	//sceneGroup_PR->removeChild(-1,1);
	
	sceneView_PRTest->update();
	// do the cull traversal, collect all objects in the view frustum into a sorted set of rendering bins
	sceneView_PRTest->cull();
	sceneView_PRTest->draw();

	// Swap Buffers
	glutSwapBuffers();
    glutPostRedisplay();
}


void reshape_PR( int w, int h )
{
    // update the viewport dimensions, in case the window has been resized.
    sceneView_PR->setViewport( 0, 0, w, h );
}
void init_glut_PR()
{
	glutInitWindowSize( wndWidthRetina	, wndHeightRetina );
	glutInitWindowPosition(200,650);
    windowPhotoReceptorLeft = glutCreateWindow( "Retina Image left" );
	glutSetWindow(windowPhotoReceptorLeft);
    glutDisplayFunc( display_PR );
    glutReshapeFunc( reshape_PR );
}
void reshape_PRTest( int w, int h )
{
    // update the viewport dimensions, in case the window has been resized.
    sceneView_PRTest->setViewport( 0, 0, w, h );
}
void init_glut_PRTest()
{
	glutInitWindowSize( wndWidthRetina	, wndHeightRetina );
	glutInitWindowPosition(200,650);
    windowPhotoReceptorLeftTest = glutCreateWindow( "Retina Image left test" );
	glutSetWindow(windowPhotoReceptorLeftTest);
    glutDisplayFunc( display_PRTest );
    glutReshapeFunc( reshape_PRTest );
}
void setCameraParameter_PR()
{
#ifdef OLD_RETINA
	sceneView_PR->setProjectionMatrixAsOrtho(-20,20,-20,20, 1,100);
#else
	sceneView_PR->setProjectionMatrixAsOrtho(-8,8,-8,8, 1,100); 
	sceneView_PRTest->setProjectionMatrixAsOrtho(-8,8,-8,8, 1,100); 
#endif
    sceneView_PR->setViewMatrixAsLookAt(
		osg::Vec3(0, -1, 0), // eye pos
		osg::Vec3(0, 1, 0),    // gaze at 
		osg::Vec3(0, 0, 1));   // usual up vector//somehow this up vector does not affect.

	sceneView_PRTest->setViewMatrixAsLookAt(
		osg::Vec3(0, -1, 0), // eye pos
		osg::Vec3(0, 1, 0),    // gaze at 
		osg::Vec3(0, 0, 1));   // usual up vector//somehow this up vector does not affect.

}

void init_osg_PR(void)
{
    // create the view of the scene.
	sceneView_PR = new osgUtil::SceneView;
    sceneView_PR->setDefaults();
	sceneView_PR->setClearColor(osg::Vec4(0,0,0,0.0)); //clear color-white
	sceneGroup_PR = new osg::Group;
	sceneView_PR->setSceneData(sceneGroup_PR.get());   
	sceneView_PR->getLight()->setConstantAttenuation(1);
	sceneView_PR->getLight()->setLinearAttenuation(0.00005);
	sceneView_PR->getLight()->setQuadraticAttenuation(0);
	sceneView_PR->getLight()->setPosition(osg::Vec4(0,-100,0,1));
	sceneView_PR->getLight()->setDirection(osg::Vec3(0,1,0));

	sceneView_PRTest = new osgUtil::SceneView;
    sceneView_PRTest->setDefaults();
	sceneView_PRTest->setClearColor(osg::Vec4(0,0,0,0.0)); //clear color-white
	sceneGroup_PRTest = new osg::Group;
	sceneView_PRTest->setSceneData(sceneGroup_PRTest.get());   
	sceneView_PRTest->getLight()->setConstantAttenuation(1);
	sceneView_PRTest->getLight()->setLinearAttenuation(0.00005);
	sceneView_PRTest->getLight()->setQuadraticAttenuation(0);
	sceneView_PRTest->getLight()->setPosition(osg::Vec4(0,-100,0,1));
	sceneView_PRTest->getLight()->setDirection(osg::Vec3(0,1,0));
	
	/*activeCellGroup = new osg::Group();
	sceneGroup_PR->addChild(activeCellGroup);*/

	//2nd light
	osg::Light* light = new osg::Light();
	light->setConstantAttenuation(1);
	light->setLinearAttenuation(0.00005);
	light->setQuadraticAttenuation(0);
	light->setPosition(osg::Vec4(0,-100,1,1));
	light->setDirection(osg::Vec3(0,1,0));
	light->setLightNum(1);
	osg::LightSource* ls = new osg::LightSource();
	ls->setLight(light);
	ls->addChild(sceneGroup_PR.get());
	sceneView_PR->setSceneData(ls->asGroup());

	osg::Light* light2 = new osg::Light();
	light2->setConstantAttenuation(1);
	light2->setLinearAttenuation(0.00005);
	light2->setQuadraticAttenuation(0);
	light2->setPosition(osg::Vec4(0,-100,1,1));
	light2->setDirection(osg::Vec3(0,1,0));
	light2->setLightNum(1);
	osg::LightSource* ls2 = new osg::LightSource();
	ls2->setLight(light2);
	ls2->addChild(sceneGroup_PRTest.get());
	sceneView_PRTest->setSceneData(ls2->asGroup());


	setCameraParameter_PR();
}


int windowSceneLeft, windowPhotoReceptorRight, windowSceneRight;


void reshapeLeft( int w, int h )
{
    // update the viewport dimensions, in case the window has been resized.
    sceneViewLeft->setViewport( 0, 0, w, h );
}

void reshapeRight( int w, int h )
{
    // update the viewport dimensions, in case the window has been resized.
    sceneViewRight->setViewport( 0, 0, w, h );
}
void init_glutLeft()
{
	glutInitWindowSize( 180	, 180 );
	glutInitWindowPosition(1200, 600);
    windowSceneLeft = glutCreateWindow( "PerceptionLeft " );
	glutSetWindow(windowSceneLeft);
    glutDisplayFunc( displayLeft );
    glutReshapeFunc( reshapeLeft );
}

/*void init_glutRight()
{
	glutInitWindowSize( 180	, 180 );
	glutInitWindowPosition(1500, 600);
    windowSceneRight = glutCreateWindow( "Perception Right" );
	glutSetWindow(windowSceneRight);
    glutDisplayFunc( displayRight );
    glutReshapeFunc( reshapeRight );
}*/

void init_osgLeft(void)
{
    // create the view of the scene.
	sceneViewLeft = new osgUtil::SceneView;
    sceneViewLeft->setDefaults();
	sceneViewLeft->setClearColor(osg::Vec4(0,0,0,0.0)); //clear color-white
	sceneGroupLeft = new osg::Group;
	sceneViewLeft->setSceneData(sceneGroupLeft.get());   
	sceneViewLeft->getLight()->setConstantAttenuation(0);
	sceneViewLeft->getLight()->setLinearAttenuation(0.00005);
	sceneViewLeft->getLight()->setQuadraticAttenuation(0);
	sceneViewLeft->getLight()->setPosition(osg::Vec4(0,0,500,1));
	sceneViewLeft->getLight()->setDirection(osg::Vec3(0,0,-1));

	//2nd light
	//osg::Light* light = new osg::Light();
	//light->setConstantAttenuation(1);
	//light->setLinearAttenuation(0.00005);
	//light->setQuadraticAttenuation(0);
	//light->setPosition(osg::Vec4(0,60,0,1));
	//light->setDirection(osg::Vec3(0,-1,0));
	//light->setLightNum(1);
	//osg::LightSource* ls = new osg::LightSource();
	//ls->setLight(light);
	//ls->addChild(sceneGroupLeft.get());
	//sceneViewLeft->setSceneData(ls->asGroup());	
	
	//osg::Light* light2 = new osg::Light();
	//light2->setConstantAttenuation(1);
	//light2->setLinearAttenuation(0.00005);
	//light2->setQuadraticAttenuation(0);
	//light2->setPosition(osg::Vec4(0, 0, -60,1));
	//light2->setDirection(osg::Vec3(0, 0,1));
	//light2->setLightNum(2);
	//osg::LightSource* ls2 = new osg::LightSource();
	//ls2->setLight(light2);
	//ls2->addChild(sceneGroupLeft.get());
	//sceneViewLeft->setSceneData(ls2->asGroup());	

	setCameraParameter();
}

void initTargetBox(){
	osg::Group* root2 = new osg::Group();

	// Declare a box class (derived from shape class) instance
	// This constructor takes an osg::Vec3 to define the center
	// and a float to define the height, width and depth.
	// (an overloaded constructor allows you to specify unique
	// height, width and height values.)
	osg::Sphere* unitCube2 = new osg::Sphere( osg::Vec3(0,0,0), SPHERE_SIZE);

	// Declare an instance of the shape drawable class and initialize 
	// it with the unitCube shape we created above.
	// This class is derived from 'drawable' so instances of this
	// class can be added to Geode instances.
	osg::ShapeDrawable* unitCubeDrawable2 = new osg::ShapeDrawable(unitCube2);
	unitCubeDrawable2->setColor(osg::Vec4(1,0,0,1));
	// Declare a instance of the geode class: 
	osg::Geode* basicShapesGeode2 = new osg::Geode();

	// Add the unit cube drawable to the geode:
	basicShapesGeode2->addDrawable(unitCubeDrawable2);

	// Add the goede to the scene:
	//root->addChild(basicShapesGeode);
	
	pat2 = new osg::PositionAttitudeTransform();
	pat2->addChild(basicShapesGeode2);
	//pat2->setPosition(osg::Vec3d(0.0, 3.98,100.0)); /////osg use z up coordinate, x is positive toward right
	if(visualTarget)
		pat2->setPosition(visualTarget.get()->getMatrix().getTrans()); /////osg use z up coordinate, x is positive toward right
	else
		pat2->setPosition(osg::Vec3d(0.0, 100, -33.0));
		//pat2->setPosition(osg::Vec3d(0.0, 3.98, 33.0));
	root2->addChild(pat2);
	sceneGroupLeft->addChild(root2);
}


void initTargetBoxRight(){

//copy from above for the second scene graph
//object for scene graph 2
		// Declare a group to act as root node of a scene:
	osg::Group* root2 = new osg::Group();

	// Declare a box class (derived from shape class) instance
	// This constructor takes an osg::Vec3 to define the center
	// and a float to define the height, width and depth.
	// (an overloaded constructor allows you to specify unique
	// height, width and height values.)
	osg::Sphere* unitCube2 = new osg::Sphere( osg::Vec3(0,0,0), SPHERE_SIZE);

	// Declare an instance of the shape drawable class and initialize 
	// it with the unitCube shape we created above.
	// This class is derived from 'drawable' so instances of this
	// class can be added to Geode instances.
	osg::ShapeDrawable* unitCubeDrawable2 = new osg::ShapeDrawable(unitCube2);
	unitCubeDrawable2->setColor(osg::Vec4(1,0,0,1));
	// Declare a instance of the geode class: 
	osg::Geode* basicShapesGeode2 = new osg::Geode();

	// Add the unit cube drawable to the geode:
	basicShapesGeode2->addDrawable(unitCubeDrawable2);

	// Add the goede to the scene:
	//root->addChild(basicShapesGeode);
	
	pat2Right = new osg::PositionAttitudeTransform();
	pat2Right->addChild(basicShapesGeode2);
	//pat2->setPosition(osg::Vec3d(0.0, 3.98,100.0)); /////osg use z up coordinate, x is positive toward right
	if(visualTarget)
		pat2Right->setPosition(visualTarget.get()->getMatrix().getTrans()); /////osg use z up coordinate, x is positive toward right
	else
		pat2Right->setPosition(osg::Vec3d(0.0, 1000, 33.0));
		//pat2->setPosition(osg::Vec3d(0.0, 3.98, 33.0));
	root2->addChild(pat2Right);
	sceneGroupRight->addChild(root2);
}
void setCameraParameter()
{
#ifdef CONTROL_LEGS
	
	sceneViewLeft->setProjectionMatrixAsPerspective(145.0, 1, 1, 10000); 
#else
	sceneViewLeft->setProjectionMatrixAsPerspective(145.0, 1, 1, 10000); 
#endif

    sceneViewLeft->setViewMatrixAsLookAt(
		//osg::Vec3(L_eye.X(), L_eye.Y(), L_eye.Z() ), // eye pos
		osg::Vec3(EYE_DIFF_X,0,1848),
		osg::Vec3(0, -500, 0),    // gaze at 
		osg::Vec3(0, 0, 1));   // usual up vector//somehow this up vector does not affect.
	
	static bool targetInitilized = false;
	if(!targetInitilized){
		initTargetBox();
		targetInitilized = true;
	}
}

void setCameraParameterRight()
{

#ifdef CONTROL_LEGS
	
	sceneViewRight->setProjectionMatrixAsPerspective(145.0, 1, 1, 10000);  
#else
	sceneViewRight->setProjectionMatrixAsPerspective(145.0, 1, 1, 10000); 
#endif

    sceneViewRight->setViewMatrixAsLookAt(
		//osg::Vec3(R_eye.X(), R_eye.Y(), R_eye.Z() ), // eye pos
		osg::Vec3(-EYE_DIFF_X,0,1700),
		osg::Vec3(0, -500, 0),    // gaze at 
		osg::Vec3(0, 0, 1));   // usual up vector//somehow this up vector does not affect.
	
	static bool targetInitilized = false;
	if(!targetInitilized){
		initTargetBoxRight();
		targetInitilized = true;
	}
}


void init_osgRight(void)
{
    // create the view of the scene.
	sceneViewRight = new osgUtil::SceneView;
    sceneViewRight->setDefaults();
	sceneViewRight->setClearColor(osg::Vec4(0,0,0,0.0)); //clear color-white
	sceneGroupRight = new osg::Group;
	sceneViewRight->setSceneData(sceneGroupRight.get());   
	sceneViewRight->getLight()->setConstantAttenuation(0);
	sceneViewRight->getLight()->setLinearAttenuation(0.00005);
	sceneViewRight->getLight()->setQuadraticAttenuation(0);
	sceneViewRight->getLight()->setPosition(osg::Vec4(0,0,500,1));
	sceneViewRight->getLight()->setDirection(osg::Vec3(0,0,-1));

	setCameraParameterRight();
}

static gReal randomNumber(gReal low, gReal high)
{
	return low + ((high-low) * rand()) / RAND_MAX;
}



static gReal randomRange(gReal low, gReal up)
{
	return rand()*((up-low)/RAND_MAX) + low;
}


double prev_lx,prev_ly,prev_lz;
double prev_rx,prev_ry,prev_rz;

int lastStoredFrame = 0;
int preKinE=0;
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
void display(void)
{
	int ind;

	gVec3 targetWorldPos;
	gVec3 localTargetPos;
	gLink* base;
	// set the view
	osg::Vec3 viewPos(
		viewRadius * cos(viewElev) * sin(viewRot),
		viewRadius * cos(viewElev) * cos(viewRot),
		viewRadius * sin(viewElev));
	sceneView->setViewMatrixAsLookAt( centerPos-viewPos, centerPos, osg::Vec3(0.0f,0.0f,1.0f) );
	
	if(bPlay)
	{
		/*if(show_forward_trace_rays) {
			compute_focal_point();
		}*/
		if(show_reverse_trace_rays) {
			exportReverseTraceBuffer();
		}
		if(exportMotion)
		{
			SysLEye->writePoses(frame, fileMotionLEye, fileMuscleMotionLEye, fileDimensionsPupil, fileDimensionsLens);
			if(visualTarget.get())
				writeBallPoses(frame,fileBall);
		}
		
		double elapsed_time=double(frame)/30.0;

		switch(RUNNING_MODE)
		{
			case GENERATE_TRAINING_DATASET_PUPIL:
			{
				// COMMENT IN MACRO #define RECORD_PUPIL WHEN GENERATING AND COMMENT OUT WHEN TESTING
				// COMMENT IN MACRO #define TEST_PUPIL WHEN TESTING ONLY
#ifdef TEST_PUPIL
				testGeneratedDatasetFromTrainingLeyePupil();
#else
				generateTrainingDatasetPupil();
#endif
				break;
			}
			case GENERATE_TRAINING_DATASET_LENS:
			{
				// COMMENT IN MACRO #define RECORD_LENS WHEN GENERATING AND COMMENT OUT WHEN TESTING
				// COMMENT IN MACRO #define TEST_LENS WHEN TESTING ONLY
#ifdef TEST_LENS
				testGeneratedDatasetFromTrainingLeyeLens();
#else
				generateTrainingDatasetLens();
#endif
				break;
			}
			case GENERATE_TRAINING_DATASET_FOVEATION:
			{
				// COMMENT IN MACRO #define RECORD_FOVEATION WHEN GENERATING
				// COMMENT IN MACRO #define TEST_FOVEATION WHEN TESTING ONLY
#ifdef TEST_FOVEATION
				testGeneratedDatasetFromTrainingLeyeFoveation();
#else
				generateTrainingDatasetFoveation();
#endif
				break;
			}
			case GENERATE_TRAINING_DATASET_MOTOR:
			{
				// COMMENT IN MACRO #define OLD_RETINA WHEN GENERATING - MAKES A LOT FASTER
#ifdef TEST_MOTOR
				testGeneratedDatasetFromTrainingLeyeMotor();
#else
				testLoopKinematicAndDNNFixatonsDisplay();
				//generateTrainingDatasetMotorWithDisplay_fixaton();
				//generateTrainingDatasetMotor();
				//generateTrainingDatasetMotor_fixaton_dataset();
#endif
				break;
			}
			case EYE_CONTROL:
			{
				// Pupil demonstration
				//move_visual_target_ball_thrown(visualTarget.get(), curTime);

				// Pupil Training

				//move_visual_target_pupil_training(visualTarget.get(), curTime);

#ifdef LENS_VIDEO
				static int close_times = 0;
				static bool oscillate_ball = true;
				//static bool oscillate_ball = false;
				static int color_Osc_ctr = 0;
				static bool once_only = true;
				if(oscillate_ball) {
					if(close_times >= 2) {
						double currEc = vis_target_ec;
						double diff = 0.645944 - currEc;
						if(diff > 0) {
							vis_target_ec = min(vis_target_ec+0.03,0.645944);
						}
						else {
							vis_target_ec = max(vis_target_ec-0.03,0.645944);
						}
					}
					else {
						double ec = 0.55 + 0.4*sin(3*color_Osc_ctr*gDTR);
						vis_target_ec = ec;
					}
					move_visual_target_oscillate_straight(visualTarget.get(), curTime);
				}
				else if(holdBallPosition) {
					move_visual_target_lens_online_fixed_positions(visualTarget.get(), curTime);
				}
				else {
					double ec = 0.55 + 0.4*sin(3*color_Osc_ctr*gDTR);
					vis_target_ec = ec;
					move_visual_target_oscillate_straight(visualTarget.get(), curTime);
				}
				targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
				double vis_yval = targetWorldPos.y();
				if(vis_yval >= -305.0) {
					close_times++;
				}
				if(close_times >= 2) {
					if(vis_yval <= -473.329 && once_only) {
						oscillate_ball = false;
						holdBallPosition = true;
						once_only = false;
					}
				}
				color_Osc_ctr++;
#else
				// Sensory demonstration
				//move_visual_target_ball_straight(visualTarget.get(), curTime);
				//move_visual_target_oscillate_straight(visualTarget.get(), curTime);
				//move_visual_target_lens_training(visualTarget.get(), curTime);
				//move_visual_target_lens_online_fixed_positions(visualTarget.get(), curTime);
#endif

				//move_visual_target_foveation_training(visualTarget.get(), curTime);

				// Specific point to look at demonstration
				//move_visual_target_fixed_position(visualTarget.get(), curTime);

				// Sweep through many points for training
				//move_visual_target_eye_training(visualTarget.get(), curTime);

				// Sweep through many points demonstration - fixed
				move_visual_target_saccade_fixed(visualTarget.get(), curTime);

				// Smooth pursuit
				//move_visual_target_smooth_pursuit(visualTarget.get(), curTime);

				//move_visual_target_smooth_lens_adjustment(visualTarget.get(), curTime);

				// Smooth pursuit
				//move_visual_target_smooth_pursuit_lateral(visualTarget.get(), curTime);

				//move_visual_target_foveation_training(visualTarget.get(), curTime);
		
				base = SysLEye->findLink("Eyeball");
				targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
				localTargetPos = targetWorldPos-base->frame().trn();
				targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());
				
#ifdef ONLINE_MOTOR
				// Keras Control
				doMuscleControlKerasLEye(targeting_location);
#else
				// Inverse Dynamics
				//doInverseDynamicMuscleControlLeye(targeting_location);
				doInverseDynamicMuscleControlLeyeNew(targeting_location);
				//doInverseDynamicMuscleControlLeyeAndOutputDataThatIsLessExtensiveThanTrainingDataButStillUsefulForPlotting(targeting_location);

				// trying foveation recording with display
				//setRandomPupilAndLensLengths();
#endif

				// Camera Pan
				/*viewRot = (viewRot < 2*PI) ? (viewRot + 0.5*gDTR) : 0;

				if(curTime < 21.0) {
					viewElev = gMax(-PI / 12.f, viewElev - 0.5*gDTR);
				}
				else {
					viewElev = gMin(PI / 12.f, viewElev + 0.5*gDTR);
				}*/

#ifdef LENS_VIDEO
				//jumphere
				static double eyeTransparency = 0.0;
				static int direction = -1;
				static int delay_counter = 0;
				static int start_delay_counter = 0;
				static int rayswitch_delay_counter = 0;
				static bool begin_translation = false;
				static int photoreceptor_change_ctr = 0;

				//lensMovementDemoComplete = true;
				//eyeTransparency = 0.8;
				//SysLEye->makeEyeTransparent(eyeTransparency, 3);
				if(curTime > 2.0) {
					viewRadius = max(viewRadius-10.0,100.0);
					if(begin_translation) {
						if(viewRot <= 89.0*gDTR) {
							eyeTransparency = 0.8;
							SysLEye->makeEyeTransparent(eyeTransparency, 3);
						}
						centerPos[1] = min(centerPos[1]+7.25,-105.f);
						viewRot = max(viewRot-1.1*gDTR,70.1*gDTR);
						viewElev = min(viewElev+1.75*gDTR,0.0*gDTR);
					}
					else if(!lensMovementDemoComplete) {
						centerPos[1] = min(centerPos[1]+2.5,-245.f);
						viewElev = max(viewElev-0.5*gDTR,-35.0*gDTR);
					}
				}

				if(viewRadius <= 100.0) {
					static int lens_delay_ctr = 0;
					if(viewRot <= -70.0*gDTR) {
						targetMoveOverride = true;
						holdBallPosition = false;
						if(rayswitch_delay_counter >= 115) {
							show_reverse_trace_rays = true;
							reverse_rays_alpha_channel = min(reverse_rays_alpha_channel+0.05,1.0);
						}
						else if(rayswitch_delay_counter >= 90) {
							forward_rays_alpha_channel = max(forward_rays_alpha_channel-0.05,0.0);
							if(forward_rays_alpha_channel <= 0.0) {
								show_forward_trace_rays = false;
							}
						}
						direction = 0;
						rayswitch_delay_counter++;
					}
					else if(viewRot >= 70.0*gDTR && lens_delay_ctr >= 30) {
						oscillateLensCilliaryActivation = true;
						direction = -1;
					}
					else if(viewRot <= 21.0*gDTR && viewRot >= 19.0*gDTR) {
						oscillatePupilSphincterActivation = true;
						direction = -1;
					}
					else {
						lens_delay_ctr++;
					}
					
					if(show_reverse_trace_rays) {
						int steps_per_change = 20;
						static int priorSelection = 0;

						if((photoreceptor_change_ctr % steps_per_change) == 0) {
							if(priorSelection == 1) {
								photoreceptor_to_trace_rho = randomNumber(36,40);
								photoreceptor_to_trace_theta = randomNumber(0,360);
								priorSelection = 0;
							}
							else {
								photoreceptor_to_trace_rho = randomNumber(0,30);
								photoreceptor_to_trace_theta = randomNumber(0,360);
								priorSelection = 1;
							}
						}

						photoreceptor_change_ctr++;
					}
					if((lensMovementDemoComplete && !oscillatePupilSphincterActivation) || pupilMovementDemoComplete) {
						begin_translation = false;
						oscillateLensCilliaryActivation = false;
						oscillatePupilSphincterActivation = false;
						viewRot += direction*1.5*gDTR;
					}
				}
				else if(viewRadius <= 300.0) {
					if(start_delay_counter < 30) {
						viewRadius = 300.0;
						start_delay_counter++;
					}
					else if(start_delay_counter < 50) {
						show_full_ball_trace = true;
						full_ball_alpha_channel = min(full_ball_alpha_channel+0.0005,0.01);
						viewRadius = 300.0;
						start_delay_counter++;
					}
					else if(start_delay_counter < 70) {
						viewRadius = 300.0;
						start_delay_counter++;
					}
					else if(eyeTransparency < 0.8) {
						static double startCurTime = curTime;
						viewRadius = 300.0;
						eyeTransparency = min(0.8, max(0.0, 0.5*(curTime - startCurTime)));
						SysLEye->makeEyeTransparent(eyeTransparency, 2);
					}
					else if(delay_counter < 90) {
						viewRadius = 300.0;
						if(delay_counter > 60) {
							full_ball_alpha_channel = max(full_ball_alpha_channel-0.0004,0.0);
							if(full_ball_alpha_channel <= 0) {
								show_full_ball_trace = false;
							}
						}
						else if(delay_counter > 30) {
							show_forward_trace_rays = true;
							forward_rays_alpha_channel = min(forward_rays_alpha_channel+0.05,1.0);
						}
						delay_counter++;
					}
					else if(delay_counter < 110) {
						viewRadius = 300.0;
						delay_counter++;
					}
					else {
						begin_translation = true;
					}	
				}

#endif

#ifdef SHOW_EYELINES
				//draw line from eye to the direction of the eye

				base = SysLEye->findLink("Eyeball");
				osg::Vec3 eye(base->frame().trn().x(),base->frame().trn().y(),base->frame().trn().z());
					
				osg::Vec3 eye_direction_vec(0,-1,0);
				gVec3 rot = getcurrent_ori_leye();
				osg::Quat q3(rot.x()*gDTR, osg::X_AXIS, 0, osg::Y_AXIS, rot.z()*gDTR, osg::Z_AXIS);
				eye_direction_vec = q3*(eye_direction_vec);
				osg::Vec3 eye_direction_vec_new2(eye_direction_vec.x(),eye_direction_vec.y(),eye_direction_vec.z());
	
				osg::Vec3Array* vertices_eye = new osg::Vec3Array;
				osg::Vec4Array* colors_eye = new osg::Vec4Array;

				vertices_eye->setName("VertexEyeLeft");

				if(exportMotion)
				{
					fprintf(fileRaytracePoints,"%d\n",frame);
				}

				/*if(RUNNING_MODE==EYE_CONTROL) {
					Vec3f_local ray_orig(eye.x(),eye.y(),eye.z());
					Vec3f_local ray_dir(eye_direction_vec_new2.x(),eye_direction_vec_new2.y(),eye_direction_vec_new2.z());

					Vec3f_local near_int, far_int;
					vector<Vec3f_local> tnears;
					bool hit = false;
					if (eyeTraceHitBox.intersect(ray_orig, ray_dir, near_int, far_int)) {
						hit = true;
					}
					if(hit) {
						Vec3f_local phit = near_int;
						after_pos = gVec3(phit.x,phit.y,phit.z);

						if(!firstTimeHit) {
							move_draw(prev_pos, after_pos);
						}
						else {
							firstTimeHit = false;
						}

						
						prev_pos = gVec3(after_pos.x(),after_pos.y(),after_pos.z());
					}
				}*/

				osg::Vec3f endpoint = eye_direction_vec_new2*5000.0;

				vertices_eye->push_back(osg::Vec3f(0.0,0.0,0.0));
				vertices_eye->push_back(endpoint);

				if(exportMotion)
				{
					osg::Vec3f absoluteEndpoint = eye + eye_direction_vec_new2*550.0;
					fprintf(fileRaytracePoints,"%g %g %g %g %g %g\n",eye.x(), eye.y(), eye.z(), absoluteEndpoint.x(), absoluteEndpoint.y(), absoluteEndpoint.z());					
					fflush(fileRaytracePoints);
				}
	
				colors_eye->push_back(osg::Vec4(0.9,0.3,0.3,1));
				colors_eye->push_back(osg::Vec4(0.9,0.3,0.3,1));
	
				osg::ref_ptr<osg::Geometry> geom_active = new osg::Geometry;
				geom_active->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, vertices_eye->size()));
				geom_active->setUseDisplayList(false);

				geom_active->setVertexArray(vertices_eye);
				geom_active->setColorArray(colors_eye);
				geom_active->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

				osg::ref_ptr<osg::Geode> geode_active = new osg::Geode;
				geode_active->addDrawable(geom_active.get());

				osg::StateSet* state_active = geode_active->getOrCreateStateSet();
				state_active->setMode(GL_LIGHTING, osg::StateAttribute::ON);
				state_active->setMode(GL_BLEND, osg::StateAttribute::ON);
				state_active->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
				osg::LineWidth* lw_active = new osg::LineWidth;
				lw_active->setWidth(300);
				//osg::Point *point_size = new osg::Point(6.0f);
				state_active->setAttribute(lw_active, osg::StateAttribute::ON);
				osg::BlendFunc* blendfunc_active = new osg::BlendFunc();
				state_active->setAttributeAndModes(blendfunc_active, osg::StateAttribute::ON);
				pat_eye_line = new osg::PositionAttitudeTransform();
				pat_eye_line->addChild(geode_active);
	
				pat_eye_line->setPosition(osg::Vec3d(eye.x(), eye.y(), eye.z()));

				if(eyeLineGroup->getNumChildren()>0)
					eyeLineGroup->removeChild(0,1);
				eyeLineGroup->addChild(pat_eye_line);

#endif

				break;
			}


		}


		frame++;
	}
	
	//manipulator
	if(visManip.get() && Manipulator)
	{
		osg::Matrixd mat;
		gXMatToOsgMatrix(mat,Manipulator->frame());
		visManip.get()->setMatrix( mat );
	}

	// draw the rendering bins.
	VisLEye->update();

	// do the update traversal the scene graph - such as updating animations
	sceneView->update();
	// do the cull traversal, collect all objects in the view frustum into a sorted set of rendering bins
	sceneView->cull();
	sceneView->draw();

	// Swap Buffers
	glutSwapBuffers();
	glutPostRedisplay();


	if(dumpImages)
	{
		//{image dumping
		if(!imageDump){
			imageDump = new osg::Image;
			imageDump->allocateImage( wndWidth,wndHeight, 1, GL_RGB, GL_UNSIGNED_BYTE);
			system("rmdir /s /q dump");
			system("mkdir dump");
			//}
		}
		sprintf(imageName,"./dump/image%06d.bmp",dumpFrame);
		imageDump->readPixels( 0, 0, wndWidth,wndHeight, GL_RGB, GL_UNSIGNED_BYTE);
		osgDB::writeImageFile( *(imageDump.get()), imageName ); //int,bmp,rgb
		dumpFrame++;
	}
}

void reshape( int w, int h )
{
	// update the viewport dimensions, in case the window has been resized.
	sceneView->setViewport( 0, 0, w, h );
}

void mousedown( int button, int state, int x, int y )
{
	/*switch(button)
	{
	case GLUT_LEFT_BUTTON:
	mouseMode = eRotate;
	break;
	case GLUT_MIDDLE_BUTTON:
	mouseMode = eTrans;	
	break;
	}*/

	lastx = x;
	lasty = y;
}

void mousemove( int x, int y )
{
	switch(mouseMode)
	{
	case eRotate:
		viewRot  += (x - lastx) / 100.0f;
		viewElev -= (y - lasty) / 100.0f;
		if( viewElev < -1.5f ) viewElev = -1.5f;
		if( viewElev >  1.5f ) viewElev =  1.5f;
		break;

	case eTrans:
		centerPos[0] -= (x - lastx);
		centerPos[2] += (y - lasty);
		break;

	case eZoom:
		viewRadius -= ((x - lastx) + (y - lasty));
		break;
	}

	lastx = x;
	lasty = y;

	//printf("Rot:%f Elev:%f Rad:%f Cen:%f %f %f\n",viewRot,viewElev,viewRadius,centerPos[0],centerPos[1],centerPos[2]);
}

void keyboard(unsigned char key, int /*x*/, int /*y*/ )
{
	gXMat T;
	switch( key )
	{
	case 'g':
		{
			std::cout << "Simulation: Free Fall - default does not do anything" << std::endl;
			RUNNING_MODE = FREE_FALL;
			bPlay = true;
		}
		break;
	case 'p':
		{
			std::cout << "Generating or testing pupil training dataset...." << std::endl;
			RUNNING_MODE = GENERATE_TRAINING_DATASET_PUPIL;
			bPlay = true;
		}
		break;
	case 'l':
		{
			std::cout << "Generating or testing lens training dataset...." << std::endl;
			RUNNING_MODE = GENERATE_TRAINING_DATASET_LENS;
			bPlay = true;
		}
		break;
	case 'f':
		{	
			std::cout << "Generating foveation training dataset...." << std::endl;
			RUNNING_MODE = GENERATE_TRAINING_DATASET_FOVEATION;
			bPlay = true;
		}
		break;
	case 'm':
		{
			std::cout << "Generating or testing motor training dataset...." << std::endl;
			RUNNING_MODE = GENERATE_TRAINING_DATASET_MOTOR;
			bPlay = true;
		}
		break;
	case 'k':
		{
			std::cout << "Running full eye simulation...." << std::endl;
			RUNNING_MODE = EYE_CONTROL;
			bPlay = true;
		}
		break;
	case 27:
		finish();
		exit(0);
		break;

	case ' ':
		bPlay = true;
		break;
	case 's':
	case 'S':
		bPlay = false;
		dumpImages = false;
		break;
	case 'd': //dump images
	case 'D':
		bPlay = true;
		dumpImages = true;
		break;
	case 'r':
	case 'R':
		mouseMode = eRotate;
		break;
	case 't':
	case 'T':
		mouseMode = eTrans;
		break;
	case 'z':
	case 'Z':
		mouseMode = eZoom;
		break;
	default:
		break;
	}
}

void menu( int selection )
{
	std::cout << "menu selection = " << selection << std::endl;
}

void init_glut(int argc, char** argv)
{
	glutInit(&argc, argv);

	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA/* | GLUT_MULTISAMPLE */);
	//glutInitWindowPosition( 1650, 100 ); //DISPLAY TO SECOND MONITOR...
	glutInitWindowPosition( 0, 0 ); //DISPLAY TO SECOND MONITOR...
	//glutInitWindowSize( 1200, 900 );
	glutInitWindowSize( wndWidth, wndHeight );
	glutCreateWindow( argv[0] );
	glutDisplayFunc( display );	
	//glutIdleFunc( display );	

	glutReshapeFunc( reshape );
	glutMouseFunc( mousedown );
	glutMotionFunc( mousemove );
	glutKeyboardFunc( keyboard );

	glutCreateMenu( menu );
	glutAddMenuEntry( "item 0", 0 );
	glutAddMenuEntry( "item 1", 1 );
	glutAttachMenu( GLUT_RIGHT_BUTTON );

	glLineWidth(2);
}

void init_osg(void)
{
	// create the view of the scene.
	sceneView = new osgUtil::SceneView;
	sceneView->setDefaults();
	sceneView->setClearColor(osg::Vec4(0.05,0.05,0.05,1.0)); //clear color-white
	//sceneView->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0)); //clear color-white
	sceneGroup = new osg::Group;
	sceneView->setSceneData(sceneGroup.get());
	sceneView->setLightingMode(osgUtil::SceneView::HEADLIGHT); 

	/*osg::Light* defaultlight = sceneView->getLight(); 
	if (defaultlight != NULL) 
	{ 
		defaultlight->setAmbient(osg::Vec4(0,0,0,1)); 
		defaultlight->setDiffuse(osg::Vec4(0,0,0,1)); 
		defaultlight->setSpecular(osg::Vec4(0,0,0,1)); 
	}*/

	sceneView->getLight()->setConstantAttenuation(1);
	sceneView->getLight()->setLinearAttenuation(0);
	sceneView->getLight()->setQuadraticAttenuation(0);
	//sceneView->getLight()->setPosition(osg::Vec4(0,0,2500,1));
	sceneView->getLight()->setPosition(osg::Vec4(35,-110,2500,1));
	sceneView->getLight()->setDirection(osg::Vec3(0,0,-1));
	sceneView->getLight()->setSpotCutoff(30.0);
	sceneView->getLight()->setSpotExponent(2.f);

	osg::Light* light = new osg::Light();
	light->setConstantAttenuation(1);
	light->setLinearAttenuation(0);
	light->setQuadraticAttenuation(0);
	//2500

	/*light->setPosition(osg::Vec4(140,-700,1840,1));
	light->setDirection(osg::Vec3(0,1,0));
	light->setDiffuse(osg::Vec4(0.0,0.0,0.0,1));
	light->setAmbient(osg::Vec4(0.0,0.0,0.0,1));
	light->setSpecular(osg::Vec4(0,0,0,1));*/

	light->setPosition(osg::Vec4(35,-120,1848,1));
	light->setDirection(osg::Vec3(0,1,0));
	light->setDiffuse(osg::Vec4(1.0,1.0,1.0,1));
	light->setAmbient(osg::Vec4(1.0,1.0,1.0,1));
	light->setSpecular(osg::Vec4(1.0,1.0,1.0,1));

	/*// not pupil
	light->setPosition(osg::Vec4(0,0,2500,1));
	light->setDirection(osg::Vec3(0,0,-1));*/

	// pupil
	/*light->setPosition(osg::Vec4(140,-700,1840,1));
	light->setDirection(osg::Vec3(0,1,0));
	light->setDiffuse(osg::Vec4(0.0,0.0,0.0,1));
	light->setAmbient(osg::Vec4(0.0,0.0,0.0,1));
	light->setSpecular(osg::Vec4(0,0,0,1));*/

	light->setLightNum(1);
	light->setSpotCutoff(25.0f);
	//light->setSpotCutoff(0.0);
	light->setSpotExponent(2.0f);
	//light->setSpotExponent(0.0f);
	osg::LightSource* ls = new osg::LightSource();
	ls->setLight(light);
	ls->addChild(sceneGroup.get());
	sceneView->setSceneData(ls->asGroup());

	/*sceneView->getLight()->setConstantAttenuation(1);
	sceneView->getLight()->setLinearAttenuation(0);
	sceneView->getLight()->setQuadraticAttenuation(0);
	sceneView->getLight()->setPosition(osg::Vec4(3000,-3000,3000,1));
	sceneView->getLight()->setDirection(osg::Vec3(-1,1,-1));
	sceneView->getLight()->setSpotCutoff(30.0);
	sceneView->getLight()->setSpotExponent(2.0f);
	*/
	
	//2nd light
	/*osg::Light* light2 = new osg::Light();
	light2->setConstantAttenuation(1);
	light2->setLinearAttenuation(0);
	light2->setQuadraticAttenuation(0);
	light2->setPosition(osg::Vec4(0,-1000,1848,1));
	light2->setDirection(osg::Vec3(0,1,0));
	light2->setLightNum(2);
	light2->setSpotCutoff(30.0f);
	light2->setSpotExponent(2.0f);
	osg::LightSource* ls2 = new osg::LightSource();
	ls2->setLight(light2);
	ls2->addChild(sceneGroup.get());
	sceneView->setSceneData(ls2->asGroup());*/

	//// 3rd light
	//osg::Light* light_3 = new osg::Light();
	//light_3->setConstantAttenuation(1);
	//light_3->setLinearAttenuation(0);
	//light_3->setQuadraticAttenuation(0);
	//light_3->setPosition(osg::Vec4(1000,0,3000,1));
	//light_3->setDirection(osg::Vec3(-1,0,-1));
	//light_3->setLightNum(1);
	//osg::LightSource* ls_3 = new osg::LightSource();
	//ls_3->setLight(light_3);
	//ls_3->addChild(sceneGroup.get());
	//sceneView->setSceneData(ls_3->asGroup());

	//// 3rd light
	//osg::Light* light_4 = new osg::Light();
	//light_4->setConstantAttenuation(1);
	//light_4->setLinearAttenuation(0);
	//light_4->setQuadraticAttenuation(0);
	//light_4->setPosition(osg::Vec4(0,0,30000,1));
	//light_4->setDirection(osg::Vec3(0,0,-1));
	//light_4->setLightNum(1);
	//osg::LightSource* ls_4 = new osg::LightSource();
	//ls_4->setLight(light_4);
	//ls_4->addChild(sceneGroup.get());
	//sceneView->setSceneData(ls_4->asGroup());
	
}


/*void testPose(){
	gBallLink* bl;
	g1dLink* rl;
	gRotMat R;
	bl = (gBallLink*)Sys->findLink("Luarm");
	R.makeExp(gVec3UnitY,90*gDTR);
	bl->setCoord(R);
	bl = (gBallLink*)Sys->findLink("Ruarm");
	R.makeExp(gVec3UnitY,-90*gDTR);
	bl->setCoord(R);

	rl = (g1dLink*)Sys->findLink("Lulnar");
	rl->setCoord(90*gDTR);
	rl = (g1dLink*)Sys->findLink("Rulnar");
	rl->setCoord(90*gDTR);

	Sys->updateKVariablesUptoAcc();	
}*/

//tau -= P*FLV*a
/*static void storeCorrectiveForceToTau()
{
	int j,k;
	gReal x;
	for(j=0;j<Sys->dof();j++)
	{
		x = 0;
		for(k=0;k<Sys->JM[j]->size();k++)
		{
			int mid = (*Sys->JM[j])[k];
			x += GM_get(Sys->P,j,mid) * Sys->muscle(mid)->FLV() * Sys->muscle(mid)->actLevel();
		}			
		GV_set(Sys->force(), j, GV_get(Sys->force(),j) - x);
	}
}*/

/*void test_moment_arm_matrix()
{
	static bool first = true;
	static GV* f=NULL;
	static GV* w=NULL;
	static GM* ws1=NULL;
	static GV* ws2=NULL;
	static gsl_permutation* permu=NULL;

	//compute tau
	GV_set_zero(Sys->acc());
	//GV_set_zero(Sys->actLevel());
	GV_set_zero(Sys->force());
	Sys->updateExtForce();
	Sys->inverseDynamics();

	if(first){
		if(!P) P = GM_alloc(Sys->dof(),Sys->numMuscles());
		f = GV_alloc(P->size2);
		w = GV_alloc(P->size2);
		GV_set_all(w,1);
		ws1 = GM_alloc(Sys->numMuscles(),Sys->numMuscles());
		ws2 = GV_alloc(Sys->numMuscles());
		permu = gsl_permutation_alloc(Sys->numMuscles());
		first = false;
	}

	Sys->solMomentArmMatrix(P);
	//GM_print(P,"%5.4f",true,0);	
	solve_Ax_y(f,P,Sys->force(),w,1e-5,ws1,ws2,permu);
	for(int i=0; i<Sys->numMuscles(); i++)
	{
		gReal FL = Sys->muscle(i)->FL();
		gReal FV = Sys->muscle(i)->FV();
		gReal a = GV_get(f,i)/(FL*FV);
		Sys->muscle(i)->setActLevel(a);
	}
	GV_set_zero(Sys->force());
	//GV_print(f,"%6.5f",true,0);
}*/

//e = qd - q, de = dqd - dq
static gReal desiredAccBody(gReal e, gReal de)
{
	gReal h = 0.5, gamma = 0.5; //meaning error will be 0.9 (gamma) of current error after 0.1 (h) second.
	return 2/h*( (1-gamma)*e/h + de );
}

static gReal desiredAcc(gReal h, gReal gamma, gReal e, gReal de)
{
	//meaning error will be (gamma) of current error after (h) second.
	return 1/h*( (1-gamma)*e/h + de );
}

void set_zero_tau(gLinkList& bonelist)
{	
	for(gLinkList::iterator it = bonelist.begin(); it != bonelist.end(); it++)
	{
		(*it)->clearFrameForce();
	}
}

void prepareRecord()
{
	flarm = fopen("t_larm.m","w");
	
	fprintf(flarm,"L = ["); fflush(flarm);
}

static int get_indexOfString(std::vector<std::string>& strList, std::string& str, int nth)
{
	int cnt = 0;
	for(unsigned int i=0; i<strList.size(); i++)
	{
		if(strList[i] == str){ 
			if(cnt++ == nth)	return i;
		}
	}
	return -1;
}

static int count_sameNames(std::vector<std::string>& strList, std::string& str, int idx)
{
	unsigned int cnt = 0;
	for(unsigned int i=0; i<idx; i++)
	{
		if(strList[i] == str) cnt++;
	}
	return cnt;
}

//selectMax = true: max, false: min
//max_ : maximum weight allowed
void SymmetrizeWeightFile(const char* filename, double max_, bool selectMax)
{	
	using namespace std;

	char name[128], val[128];
	vector<string> nameList;
	vector<gReal>  valList;

	FILE* fp = fopen(filename,"r");
	while( EOF != fscanf(fp,"%s %s",name,val) )
	{
		nameList.push_back(name);
		valList.push_back(atof(val));
	}
	fclose(fp);

	const char* ptr;

	for(unsigned int i=0;i<nameList.size();i++)
	{
		ptr = nameList[i].c_str();
		if(ptr[0] == 'l')	//if name starts with 'l'
		{
			//cound how many muscles of the same name are before i 
			int nth = count_sameNames(nameList,nameList[i],i);

			//find symmetric muscle
			string tar = nameList[i];
			tar[0] = 'r';
			int idx = get_indexOfString(nameList,tar,nth);
			if(idx > -1)
			{
				gReal sval = selectMax? gMin(max_, gMax(valList[i],valList[idx])) : gMin(max_, gMin(valList[i],valList[idx]));
				valList[i] = valList[idx] = sval;
			}
		}
	}

	//save
	strcpy(name,filename);
	strcat(name,".sym");
	FILE* fp1 = fopen(name,"w");
	for(unsigned int i=0;i<nameList.size();i++)
	{
		fprintf(fp1,"%s %6.5f\n",nameList[i].c_str(),valList[i]);
	}
	fclose(fp1);
}

#include <time.h>
char timestr[9];

void printMuscleNames(gMuscleList m, const char* filename)
{	
	FILE* fp = fopen(filename, "w");

	int cnt = 0;
	for (gMuscleList::iterator it = m.begin(); it != m.end(); it++)
	{
		fprintf(fp, "%d %s\n", cnt++, (*it)->name());
	}

	fclose(fp);
}

//normalize column vectors
static void normalizeCol(GM* M)
{
	gReal s,x;
	unsigned int i,j;
	for(i=0;i<M->size2;i++)
	{
		s = 0;
		for(j=0;j<M->size1;j++)
		{
			x = GM_get(M,j,i);
			s += x*x;
		}
		s = 1.0/gSqrt(s);

		for(j=0;j<M->size1;j++)
		{
			GM_set(M,j,i, GM_get(M,j,i) * s);
		}
	}
}

void finish()
{
	//if(recordMuscleAct ) finishRecord();

	if(exportMotion){
		fclose(fileBall);
		fclose(fileMotionLEye);
		fclose(fileMuscleMotionLEye);
		fclose(fileDimensionsPupil);
		fclose(fileDimensionsLens);
		fileBall = 0;
		fileMotionLEye = 0;
		fileMuscleMotionLEye = 0;
		fileDimensionsPupil = 0;
		fileDimensionsLens = 0;
		fileRaytracePoints = 0;
	}

	if(fpQ)
	{
		fprintf(fpQ,"];");
		fclose(fpQ);
	}

	if(fpPt)
	{
		fprintf(fpPt,"];");
		fclose(fpPt);
	}

	if(fpPh)
	{
		fprintf(fpPh,"];");
		fclose(fpPh);
	}

	if(snRw){
		delete[] snRw;
		snRw = 0;
	}
	if(snIw) {
		delete[] snIw;
		snIw = 0;
	}
	if(snCw) {
		delete[] snCw;
		snCw = 0;
	}
}

void writeActivation(int frame,FILE* fp,gReal up)
{
	unsigned int i;

	assert(fp != NULL);
	fprintf(fp,"%d\n",frame);

	char name[128];
	std::vector<gMuscleList>::iterator git;
	gReal a;
	for(git = exportMuscles.begin(); git != exportMuscles.end(); git++)
	{
		if( git->size() > 1 ){
			a = 0;
			for(i=0; i< git->size(); i++)
			{
				a = gMax(a, (*git)[i]->actLevel());
			}
			a /= git->size(); // mean activation

			//if name ends with _1, remove it.
			const char* oriname = (*git)[0]->name();		
			if(oriname[strlen(oriname)-1]=='1') {
				strncpy(name,oriname,strlen(oriname)-2);
				name[strlen(oriname)-2] = '\0';
			}
			else
				strcpy(name,oriname);

			//bound
			a = gMin(up,gMax(0,a));

			fprintf(fp,"%s %g\n",name, a);
		}
		else
		{
			a = (*git)[0]->actLevel();
			a = gMin(up,gMax(0,a));
			fprintf(fp,"%s %g\n",(*git)[0]->name(),a);
		}
	}	
}

void writeBallPoses(int frame, FILE* fp)
{
	assert(fp != NULL);
	
	fprintf(fp,"%d %g\n",frame,vis_target_ec);
	fprintf(fp,"%g %g %g\n", visualTarget.get()->getMatrix().getTrans().x(),visualTarget.get()->getMatrix().getTrans().y(), visualTarget.get()->getMatrix().getTrans().z());

	fflush(fp);
}




//dst = diag * src
void multDiagMat(GM* dst, double* diag, const GM* src)
{
	size_t i,j;
	assert( (dst->size2 == src->size2) && (dst->size1 == src->size1) );

	for(i=0; i<dst->size1; i++)
	{
		for(j=0; j<dst->size2; j++)
		{
			GM_set(dst,i,j, diag[i] * GM_get(src, i,j));
		}
	}
}

void update_PF_ub_Leye()
{
	size_t i,j;
	gReal x;
	//PF_ub
	//cout << "PF ub size2 " << PF_ub->size2 << endl;
	//cout << "PF ub size1 " << PF_ub->size1 << endl;
	for(j=0;j<PF_ubLeye->size2;j++)
	{
		x = SysLEye->muscle(j)->FLV();
		for(i=0;i<PF_ubLeye->size1;i++)
		{	
			GM_set(PF_ubLeye,i,j, GM_get(SysLEye->P,i,j) * x );
		}
	}
}

//Usage: call CreateDumbbellFiles("Dumbbell.txt") in the beginning of main function
void CreateDumbbellFiles(const char* motion)
{
	using namespace osg;

	char wd1[128],w1[64],w2[64],w3[64],fname[64];
	char fr[128];
	char line[1000];
	int mode; //1: apply Tshaft, 2: apply Tweight
	double x,y,z,w,p1,p2,p3,s;
	osg::Matrix Tshaft, Tweight, Dimension;
	osg::Quat quat;
	osg::Vec3 vin, vout;

	Dimension.makeScale(0.001,0.001,0.001);

	FILE* fin = fopen(motion,"r");
	while(EOF != fscanf(fin,"%s",fr))
	{
		//left
		fscanf(fin,"%s",wd1); //"left"
		fscanf(fin,"%s",wd1); x = atof(wd1); //x
		fscanf(fin,"%s",wd1); y = atof(wd1); //y
		fscanf(fin,"%s",wd1); z = atof(wd1); //z
		fscanf(fin,"%s",wd1); w = atof(wd1); //w
		fscanf(fin,"%s",wd1); p1 = atof(wd1); //p1
		fscanf(fin,"%s",wd1); p2 = atof(wd1); //p2
		fscanf(fin,"%s",wd1); p3 = atof(wd1); //p3
		fscanf(fin,"%s",wd1); s = atof(wd1);  //scale

		quat.set(x,y,z,w);
		Tshaft.set(quat);
		Tshaft.setTrans(p1,p2,p3);
		Tshaft.postMult(Dimension);

		Tweight.makeScale(s,s,1);
		Tweight.postMult(Tshaft);

		ifstream fobj;
		fobj.open("props/dumbbell.obj",fstream::in);

		fstream fout;
		sprintf(fname,"%s_l.obj",fr);		
		fout.open(fname,fstream::out);		

		while(1)
		{
			fobj.getline(line,1000);

			if(fobj.fail()) break;

			if(!strcmp(line,"g Loft01"))
			{
				mode = 1;
			}
			else if(!strcmp(line,"g Line02") || !strcmp(line,"g Line03"))
			{
				mode = 2; 
			}

			if(!strncmp(line,"v ",2)) //starts with "v "
			{
				//extract 3 numbers
				sscanf(line+2,"%s %s %s",w1,w2,w3);
				vin.set(atof(w1),atof(w2),atof(w3));

				if(mode==1)
					vout = Tshaft.preMult(vin);
				else
					vout = Tweight.preMult(vin);

				fout << "v " << vout.x() <<" " << vout.y() << " " << vout.z() << endl;
			}
			else
			{	
				fout.write(line,strlen(line));				
				fout << endl;
			}
		}

		fout.close();
		fobj.close();


		//right
		fscanf(fin,"%s",wd1); //right
		fscanf(fin,"%s",wd1); x = atof(wd1); //x
		fscanf(fin,"%s",wd1); y = atof(wd1); //y
		fscanf(fin,"%s",wd1); z = atof(wd1); //z
		fscanf(fin,"%s",wd1); w = atof(wd1); //w
		fscanf(fin,"%s",wd1); p1 = atof(wd1); //p1
		fscanf(fin,"%s",wd1); p2 = atof(wd1); //p2
		fscanf(fin,"%s",wd1); p3 = atof(wd1); //p3
		fscanf(fin,"%s",wd1); s = atof(wd1);  //scale

		quat.set(x,y,z,w);
		Tshaft.set(quat);
		Tshaft.setTrans(p1,p2,p3);
		Tshaft.postMult(Dimension);

		Tweight.makeScale(s,s,1);
		Tweight.postMult(Tshaft);

		ifstream fobj2;
		fobj2.open("props/dumbbell.obj",fstream::in);
		sprintf(fname,"%s_r.obj",fr);		
		fout.open(fname,fstream::out);		

		while(1)
		{
			fobj2.getline(line,1000);

			if(fobj2.fail()) break;

			if(!strcmp(line,"g Loft01"))
			{
				mode = 1;
			}
			else if(!strcmp(line,"g Line02") || !strcmp(line,"g Line03"))
			{
				mode = 2; 
			}

			if(!strncmp(line,"v ",2)) //starts with "v "
			{
				//extract 3 numbers
				sscanf(line+2,"%s %s %s",w1,w2,w3);
				vin.set(atof(w1),atof(w2),atof(w3));

				if(mode==1)
					vout = Tshaft.preMult(vin);
				else
					vout = Tweight.preMult(vin);

				fout << "v " << vout.x() <<" " << vout.y() << " " << vout.z() << endl;
			}
			else
			{	
				fout.write(line,strlen(line));				
				fout << endl;
			}
		}

		fout.close();
		fobj2.close();
	}
	fclose(fin);
}

void calculateMuscleForceLEye()
{
	double coact = 0.1;

	GV* a_buf = 0;
	a_buf = GV_alloc(6);

	SysLEye->sol_momentArmMatrixForBoneM_Sparse();
	update_PF_ub_Leye();

	if(coact>1e-5){
		GV_scale(SysLEye->force(),-1);
		//compute antagonist muscle activation
		SysLEye->sol_muscleForceUpperBodyAntagonist_SNOPT();
		GV_memcpy(a_buf,SysLEye->actLevel());
		GV_scale(SysLEye->force(),-1);
	}
	//compute agonist muscle activation
	SysLEye->sol_muscleForceUpperBody_SNOPT();

	if(coact>1e-5){
		GV_scale(SysLEye->actLevel(),1+coact);
		GV_scale(a_buf,coact);
		GV_add(SysLEye->actLevel(),a_buf);
	}

	GV_free(a_buf);
}

bool calculateMuscleForceLEyeNew()
{
	bool useComputedActivations = true;
	double coact = 0.1;
	GV* a_buf = 0;
	if(!a_buf) a_buf = GV_alloc(6);
	SysLEye->sol_momentArmMatrixForBoneM_Sparse();
	update_PF_ub_Leye();

	if(coact>1e-5){
		GV_scale(SysLEye->force(),-1);
		//compute antagonist muscle activation
		useComputedActivations = useComputedActivations && SysLEye->sol_muscleForceEyeAntagonist_SNOPT();
		GV_memcpy(a_buf,SysLEye->actLevel());
		GV_scale(SysLEye->force(),-1);
	}
	useComputedActivations = useComputedActivations && SysLEye->sol_muscleForceEye_SNOPT();
	if(coact>1e-5){
		GV_scale(SysLEye->actLevel(),1+coact);
		GV_scale(a_buf,coact);
		GV_add(SysLEye->actLevel(),a_buf);
	}
	GV_free(a_buf);

	return useComputedActivations;
}

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}


//functions below are the ones I am currently working on
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initialConfig()
{
	useMocap = false;
}

void SetDesiredAngles_FrontCrawl(gReal time)
{
	gReal phase=time/2;
	if(phase>gPI*2)
	{
		phase=phase-(int(phase/(gPI*2)))*(gPI*2);
	}

	float t_s=phase/(gPI*2);

	float t_s_arm = t_s*3;
	t_s_arm -= floor(t_s_arm);
	float t_s_arm_r = t_s_arm+0.5;
	if(t_s_arm_r>1)
		t_s_arm_r -= 1.0;

	//Sys->D_la0[0] = 30;
	//Sys->D_la0[1] = 30;

}

//this is real time inverse dynamics solution
gVec3 computeTargetPos2(double* target, gVec3 cur_pos)
{

	gVec3 target_vector_la; target_vector_la.set(target);
	//always normazlize the target vector first, and then multiply whatever we want to move in one step as the codfficient
	target_vector_la = target_vector_la - cur_pos;
	if(target_vector_la.magnitude()<1){
		bPlay = false;
		bTargetPosAchieved=true;
		return cur_pos;
	}


	target_vector_la.normalize();
	target_vector_la *= 1;
	gVec3 target_pos = target_vector_la + cur_pos;
	return target_pos;
}

//this is real time inverse dynamics solution
gVec3 computeTargetUnitVec(double* target, gVec3 cur_pos)
{

	gVec3 target_vector_la; target_vector_la.set(target);
	gVec3 new_target_vector_la;
	new_target_vector_la = target_vector_la - cur_pos;
	
	new_target_vector_la.normalize();
	new_target_vector_la *= 1;
	return new_target_vector_la;
}

gVec3 computeTargetUnitVec(double* target, gVec3 cur_pos, double &magnizued)
{

	gVec3 target_vector_la; target_vector_la.set(target);
	gVec3 new_target_vector_la;
	new_target_vector_la = target_vector_la - cur_pos;
	magnizued = new_target_vector_la.magnitude();
	
	if(magnizued > 1)
	{
		new_target_vector_la.normalize();
	}
	return new_target_vector_la;
}

void generateTrainingDatasetByPoseBasedTargetInverseDynamicsWithDisplay()
{
	unsigned int i;
	ofstream foutput("outputTest.csv", ios::app);
	ofstream foutputDelta("outputTestDelta.csv", ios::app);
	ofstream finput("inputTest.csv", ios::app);
	//srand(time(NULL));

	static int iteration_count = 0;

	//doOneStepPoseBasedTargetInverseDynamicsControlForGenerateTrainingDatasetIntergrateToNextTimestep(curTime, 0,  2.5, 3, foutput,finput, false);
	//doOneStepMuscleDrivenControl(curTime, 0,  2.5, 3, foutput,finput, false);
	//doOneStepMuscleDrivenControlGeneratingTrainingDataset(curTime, 0,  40, 3, foutput,finput, true);
	//doOneStepPoseBasedTargetInverseDynamicsControlForGenerateTrainingDatasetIntergrateToNextTimestep(0,  2.5, 3, foutput, finput, true);
	//doOneStepPoseBasedTargetInverseDynamicsControlForGenerateTrainingDatasetIntergrateToNextTimestep2(0,  2.5, 3, foutput, finput, true);
	//doOneStepPoseBasedTargetInverseDynamicsControlForGenerateTrainingDatasetIntergrateToNextTimestepChangingMagnitude(0,  2.5, 3, foutput,foutputDelta, finput, true);

	iteration_count++;
	cout << "iteration No. = " << iteration_count << endl;
	
	foutput.close();
	foutputDelta.close();
	finput.close();
}

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

//this is real time inverse dynamics solution
gVec3 computeTargetUnitVecRegularized(double* target, gVec3 cur_pos)
{

	gVec3 target_vector_la; target_vector_la.set(target);
	//always normazlize the target vector first, and then multiply whatever we want to move in one step as the codfficient
	target_vector_la = target_vector_la - cur_pos;
	
	/*if(target_vector_la.magnitude()<10){
		bPlay = false;
		bTargetPosAchieved=true;
		return cur_pos;
	}*/


	target_vector_la.normalize();
	target_vector_la *= 1;
	gVec3 target_vector_la_regulalized((target_vector_la.x() + 1) /2,(target_vector_la.y() + 1) /2,(target_vector_la.z() + 1) /2);
	return target_vector_la_regulalized;
}

void readStdMeanLEye()
{
	int p=0;
	cout << "reading mean and std"<<endl;
	//csvファイルを1行ずつ読み込む
	ifstream ifs("meanEye.csv");
	if(!ifs){
		cout<<"Could not load mean file";
		return;
	}
	string str;
	while(getline(ifs,str)){
		string token;
		istringstream stream(str);\
		while(getline(stream,token,',')){
			float temp=stof(token);
			meanValueLEye[p] = temp;
			p++;
		}
	}

	p=0;

	ifstream ifs2("stdEye.csv");
	if(!ifs2){
		cout<<"Could not load std file";
		return;
	}
	while(getline(ifs2,str)){
		string token;
		istringstream stream(str);
		while(getline(stream,token,',')){
			float temp=stof(token);
			stdValueLEye[p] = temp;
			p++;
		}
	}
}

#define STEP_MOVE_REFLEX
//#define STEP_MOVE_VOLUNTARY
#define VARIOUS_MAGNITUDE

//this is real time inverse dynamics solution
gVec3 computeTargetUnitVecNeck(double* target, gVec3 cur_pos)
{
	gVec3 target_vector_la; target_vector_la.set(target);
	gVec3 new_target_vector_la;
	new_target_vector_la = (target_vector_la - cur_pos);
	new_target_vector_la.normalize();
	new_target_vector_la *= 1;
	return new_target_vector_la;
}

gVec3 computeTargetUnitVecLEye(double* target, gVec3 cur_pos)
{
	gVec3 target_vector_la; target_vector_la.set(target);
	gVec3 new_target_vector_la;
	new_target_vector_la = (target_vector_la - cur_pos);
	new_target_vector_la.normalize();
	new_target_vector_la *= 1;
	return new_target_vector_la;
}

double theta_vec_conversion(gVec3 v){
	return gMin(90*gDTR,gMax(-90*gDTR, atan2(v.x(),-v.y())));
}

double phi_vec_conversion(gVec3 v){
	return gMin(90*gDTR, gMax(-90*gDTR,atan2(v.z(),sqrt( pow(v.x(),2) + pow(-v.y(),2)))));
}

gVec3 getcurrent_ori_leye()
{	
	gVec3 f;
	double orit[3] ={0.0, 0.0, 0.0};

	gBallLink* l = (gBallLink*)(SysLEye->eyeball);
	f = gVec3::log(l->coord());

	orit[0] += f.x();
	orit[1] += f.y();
	orit[2] += f.z();

	gVec3 cur_ori_in(orit[0],orit[1],orit[2]);
	return cur_ori_in*gRTD;
}

void setDesiredAccHNTALLeye(gReal h, gReal gamma)
{	
	gBallLink* l = (gBallLink*)(SysLEye->eyeball);
	gVec3 d = gVec3::log(~l->coord()*SysLEye->D_e[0]);

	gVec3 currentVel(l->genVel(0), l->genVel(1), l->genVel(2));
	//cout << "vel: " << currentVel.magnitude() << endl;

	double x_accel = desiredAcc( h, gamma, d.x() , -l->genVel(0) );
	double y_accel = desiredAcc( h, gamma, d.y() , -l->genVel(1) );
	double z_accel = desiredAcc( h, gamma, d.z() , -l->genVel(2) );
	
	l->setGenAcc(0 , x_accel );
	l->setGenAcc(1 , y_accel );
	l->setGenAcc(2 , z_accel );

	for(int k = 2; k < SysLEye->numLinks(); k++) {
		gLink* l = SysLEye->link(k);
		switch(l->type()) {
		case TYPE_1D_LINK:
			{
				g1dLink* pl = (g1dLink*)l;
				pl->setGenAcc(0);
			}
			break;
		}
	}
}

void setMaxVelAccLeye(gReal h, gReal gamma)
{	
	gReal MaxVel = 9;

	gBallLink* l = (gBallLink*)(SysLEye->eyeball);
	gVec3 error = gVec3::log(~l->coord()*SysLEye->D_e[0]);
	gVec3 currentVel(l->genVel(0), l->genVel(1), l->genVel(2));

	//cout << "error mag: " << error.magnitude()*gRTD << endl;
	if(error.magnitude()*gRTD > 9.0) {
		double a = (MaxVel - currentVel.magnitude()) / displayTimeStep;
		error.normalize();
		error = error * a;
		cout << "could not reach in one time step - capping velocity" << endl;
		l->setGenAcc(0 , error.x() );
		l->setGenAcc(1 , error.y() );
		l->setGenAcc(2 , error.z() );
	}
	else {
		setDesiredAccHNTALLeye(h,gamma);
	}

}

bool setTargetJointAngleLEye(double theta, double phi){
	gRotMat ori_leye_new;
	ori_leye_new.makeRotateXYZ(phi,0,theta);
	bool isNewTargetSet = SysLEye->inv_kinLEye(&ori_leye_new,NULL,1);
	return isNewTargetSet;
}

GV* tmp1 = NULL;
GV* tmp2 = NULL;
void computeDesiredMuscleState1LEye(GV* srcAcc, gReal timestep)
{
	int i;
	if(!tmp2) tmp2 = GV_alloc(SysLEye->dof());

	//store q,dq
	GV_memcpy(tmp2,SysLEye->vel());
	SysLEye->storeCoord();

	//compute dq/q and set dq<- dqd, q<- qd
	GB_axpy(timestep,srcAcc,SysLEye->vel());		//dq += h*ddqd
	SysLEye->timeIntegrateCoordinates(timestep);	// q += h*dq
	SysLEye->updateKVariablesUptoVel(); 

	for(i=0;i<SysLEye->numMuscles();i++)
	{
		ReflexLEye->set_e_d(i, SysLEye->muscle(i)->strain());
		ReflexLEye->set_de_d(i, SysLEye->muscle(i)->strainRate());
	}

	//restore q,dq
	GV_memcpy(SysLEye->vel(),tmp2);
	SysLEye->restoreCoord();
	SysLEye->updateKVariablesUptoVel();	
}

void computeDesiredMuscleState2Leye(double theta_next, double phi_next, gReal timestep)
{
	int i;
	double *prev_strain;
	prev_strain = new double [SysLEye->numMuscles()];

	SysLEye->storeCoord();

	bool target_leye_set = setTargetJointAngleLEye(theta_next, phi_next);

	if(!target_leye_set){
		cout << "***IK not solvable" << endl;
	}

	SysLEye->set_jointFrom_D_e();
	SysLEye->updateKVariablesUptoPos(); 

	for(i=0;i<SysLEye->numMuscles();i++)
	{
		ReflexLEye->set_e_d(i, SysLEye->muscle(i)->strain());
		prev_strain[i] = SysLEye->muscle(i)->strain();
	}

	SysLEye->restoreCoord();
	SysLEye->updateKVariablesUptoPos();	
	SysLEye->storeCoord();

	double theta2 = theta_next*2.0;
	double phi2 = phi_next*2.0;
	
	bool target_leye_set2 = setTargetJointAngleLEye(theta2, phi2);

	if(!target_leye_set){
		cout << "***IK not solvable" << endl;
	}

	SysLEye->set_jointFrom_D_e();
	SysLEye->updateKVariablesUptoPos(); 
	for(i=0;i<SysLEye->numMuscles();i++)
	{
		ReflexLEye->set_de_d(i, (SysLEye->muscle(i)->strain()-prev_strain[i])/timestep);
		//cout<<"rate: "<<(Sys->muscle(i)->strain()-prev_strain[i])/timestep<<endl;
	}

	SysLEye->restoreCoord();
	SysLEye->updateKVariablesUptoPos();

	delete[] prev_strain;
}

void doOneStepPoseBasedTargetInverseDynamicsControlForGenerateTrainingDatasetLeye(ofstream& foutputDelta, ofstream& finput, ofstream& resets, gVec3 target)
{
	int num_eoc_muscles = 6;
	static GV *prev_activation = GV_alloc(num_eoc_muscles); // only 6 EOC muscles
	for(int i = 0; i < 6; i++) {
		GV_set(prev_activation,i,GV_get(SysLEye->actLevel(),i));
	}
	static int iteration = 0;

	//target.normalize(); masaki commenting out for the scanpath fixaton
	//doInverseDynamicMuscleControlLeye(target);

	bool preCallStatus = leye_IK_solved && leye_muscle_solved;
	doInverseDynamicMuscleControlLeyeNew(target);
	bool postCallStatus = leye_IK_solved && leye_muscle_solved;
	/*for(int i = 0; i < 6; i++) {
		cout << GV_get(SysLEye->actLevel(),i) << endl;
	}*/
	bool writeReset = !preCallStatus && postCallStatus;

	iteration++;
	if(writeReset)
		iteration=0;

	if(iteration>1000)
	{
		iteration=0;
		resetToInitialPoseLeye();
	}
	if (iteration > 0)
	{
		gVec3 ori_leye = getcurrent_ori_leye();
		
		if(leye_IK_solved && leye_muscle_solved){

			//finput << std::fixed;
			//foutputDelta << std::fixed;
			finput << idcyl <<","<< idcxl << ",";

			for(int i=0; i < num_eoc_muscles-1; i++){
				finput << (double) GV_get(prev_activation, i) <<",";
				foutputDelta << (double) (GV_get(SysLEye->actLevel(), i) - GV_get(prev_activation,i)) << ",";
			}
			finput << (double) GV_get(prev_activation, num_eoc_muscles-1) << endl;
			foutputDelta << (double) (GV_get(SysLEye->actLevel(), num_eoc_muscles-1) - GV_get(prev_activation,num_eoc_muscles-1)) << endl;

			int reset = (writeReset) ? 1 : 0;
			resets << reset << endl;

			finput.flush();
			foutputDelta.flush();
			resets.flush();

		}

	}
}


void doOneStepPoseBasedTargetInverseDynamicsControlForGenerateTrainingDatasetLeye_fixaton(ofstream& foutputDelta, ofstream& finput, ofstream& resets, gVec3 target, float duration)
{
	static int iteration = 0;

	//if(iteration>50)
	//{
	//	iteration=0;
	//	re_create_eye_ball_left();
	//	//resetToInitialPoseLeye();
	//}


	int num_eoc_muscles = 6;
	static GV *prev_activation = GV_alloc(num_eoc_muscles); // only 6 EOC muscles
	for(int i = 0; i < 6; i++) {
		GV_set(prev_activation,i,GV_get(SysLEye->actLevel(),i));
	}
	
	//target.normalize(); //comment out by Masaki for fixaton
	//doInverseDynamicMuscleControlLeye(target);

	bool preCallStatus = leye_IK_solved && leye_muscle_solved;
	doInverseKinematicsControlLeyeNew_fixaton(target, duration);
	//doMuscleControlKerasLEye(target);
	bool postCallStatus = leye_IK_solved && leye_muscle_solved;

	bool writeReset = !preCallStatus && postCallStatus;

	iteration++;
	if(writeReset)
		iteration=0;


	if (iteration > 0)
	{
		gVec3 ori_leye = getcurrent_ori_leye();
		
		if(leye_IK_solved && leye_muscle_solved){

			//finput << std::fixed;
			//foutputDelta << std::fixed;
			finput << idcyl <<","<< idcxl << ",";

			for(int i=0; i < num_eoc_muscles-1; i++){
				finput <<  (double) GV_get(prev_activation, i) <<",";
				foutputDelta <<  (double) (GV_get(SysLEye->actLevel(), i) - GV_get(prev_activation,i)) << ",";
			}
			finput <<  (double) GV_get(prev_activation, num_eoc_muscles-1) << endl;
			foutputDelta <<  (double) (GV_get(SysLEye->actLevel(), num_eoc_muscles-1) - GV_get(prev_activation,num_eoc_muscles-1)) << endl;

			int reset = (writeReset) ? 1 : 0;
			resets << reset << endl;

			finput.flush();
			foutputDelta.flush();
			resets.flush();

		}

	}

	// Everything below this just records data for plotting
	ofstream foutput("kerasOutputLeyeActivationsForPlotting.csv", ios::app);
	ofstream foutputori("outputOriLeye.csv", ios::app);
	ofstream foutputvel("outputVelLeye.csv", ios::app);
	ofstream foutputacc("outputAccLeye.csv", ios::app);

	static double prev_vel[2] ={0.0, 0.0};

	for(int i=0; i<num_eoc_muscles-1; i++){
		foutput << GV_get(SysLEye->actLevel(), i) << ",";
	}
	foutput << GV_get(SysLEye->actLevel(), num_eoc_muscles-1) << endl;
	foutput.flush();

	gVec3 ori_leye = getcurrent_ori_leye();

	foutputori << ori_leye.x() << ",";
	foutputori << ori_leye.z() << "," << endl;
	foutputori.flush();

	gBallLink* l = (gBallLink*)(SysLEye->eyeball);

	double velx = l->genVel(0);
	double velz = l->genVel(2);

	foutputvel << velx << ",";
	foutputvel << velz << "," << endl;
	foutputvel.flush();

	double accx = velx - prev_vel[0];
	double accz = velz - prev_vel[1];

	foutputacc << accx << ",";
	foutputacc << accz << "," << endl;
	foutputacc.flush();

	prev_vel[0] = velx;
	prev_vel[1] = velz;
}



void testKinematicAndDNNFixatons(ofstream& foutputDelta, ofstream& finput, ofstream& resets, gVec3 target, float duration, bool kinematic)
{
	static int iteration = 0;

	//if(iteration>50)
	//{
	//	iteration=0;
	//	re_create_eye_ball_left();
	//	//resetToInitialPoseLeye();
	//}


	int num_eoc_muscles = 6;
	static GV *prev_activation = GV_alloc(num_eoc_muscles); // only 6 EOC muscles
	for(int i = 0; i < 6; i++) {
		GV_set(prev_activation,i,GV_get(SysLEye->actLevel(),i));
	}
	
	//target.normalize(); //comment out by Masaki for fixaton
	//doInverseDynamicMuscleControlLeye(target);

	bool preCallStatus = leye_IK_solved && leye_muscle_solved;
	if(kinematic)
	{
	 doInverseKinematicsControlLeyeNew_fixaton(target, duration);
	}
	else{
		doMuscleControlKerasLEye(target);
	}

	// Everything below this just records data for plotting
	ofstream foutput("kerasOutputLeyeActivationsForPlotting.csv", ios::app);
	ofstream foutputori("outputOriLeye.csv", ios::app);
	ofstream foutputvel("outputVelLeye.csv", ios::app);
	ofstream foutputacc("outputAccLeye.csv", ios::app);

	static double prev_vel[2] ={0.0, 0.0};

	for(int i=0; i<num_eoc_muscles-1; i++){
		foutput << GV_get(SysLEye->actLevel(), i) << ",";
	}
	foutput << GV_get(SysLEye->actLevel(), num_eoc_muscles-1) << endl;
	foutput.flush();

	gVec3 ori_leye = getcurrent_ori_leye();

	foutputori << ori_leye.x() << ",";
	foutputori << ori_leye.z() << "," << endl;
	foutputori.flush();

	gBallLink* l = (gBallLink*)(SysLEye->eyeball);

	double velx = l->genVel(0);
	double velz = l->genVel(2);

	foutputvel << velx << ",";
	foutputvel << velz << "," << endl;
	foutputvel.flush();

	double accx = velx - prev_vel[0];
	double accz = velz - prev_vel[1];

	foutputacc << accx << ",";
	foutputacc << accz << "," << endl;
	foutputacc.flush();

	prev_vel[0] = velx;
	prev_vel[1] = velz;
}

void generateTrainingDatasetMotorWithDisplay()
{
	ofstream foutputDelta(".\\motorData\\outputTestDeltaMotor.csv", ios::app);
	ofstream finput(".\\motorData\\inputTestMotor.csv", ios::app);
	ofstream fresets(".\\motorData\\resets.csv", ios::app);

	move_visual_target_eye_training(visualTarget.get(), curTime);
	gLink* base = SysLEye->findLink("Eyeball");
	gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
	gVec3 localTargetPos = targetWorldPos-base->frame().trn();
	targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());

	doOneStepPoseBasedTargetInverseDynamicsControlForGenerateTrainingDatasetLeye(foutputDelta, finput, fresets, targeting_location);

	foutputDelta.close();
	finput.close();
}

void generateTrainingDatasetMotor()
{
	static ofstream foutputDelta(".\\motorData\\outputTestDeltaMotor.csv", ios::app);
	static ofstream finput(".\\motorData\\inputTestMotor.csv", ios::app);
	static ofstream fresets(".\\motorData\\resets.csv", ios::app);

	unsigned int i;
	int cnt=0;

	//3000000
	while(cnt <= 300000)
	{
		move_visual_target_eye_training(visualTarget.get(), curTime);
		gLink* base = SysLEye->findLink("Eyeball");
		gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
		gVec3 localTargetPos = targetWorldPos-base->frame().trn();
		targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());
		//computeThetaPhi();
		/*compute_pupil_size();
		compute_lens_size();
		SysLEye->adjust_pupil(pupil_size);
		SysLEye->adjust_cornea();*/
		doOneStepPoseBasedTargetInverseDynamicsControlForGenerateTrainingDatasetLeye(foutputDelta, finput, fresets, targeting_location);
		if(leye_IK_solved && leye_muscle_solved) {
			cnt++;
		}
		if(cnt%10==0) {
			cout << "iteration = " << cnt << endl;
		}

		/*if(cnt%100==0) {
			leye_muscle_solved = false;
			resetToInitialPoseLeye();
		}*/
	}
	
	foutputDelta.close();
	finput.close();
	fresets.close();
	finish();
	exit(0);
}


void generateTrainingDatasetMotor_fixaton_dataset()
{
	static ofstream foutputDelta(".\\motorData\\outputTestDeltaMotorFixaton.csv", ios::app);
	static ofstream finput(".\\motorData\\inputTestMotorFixaton.csv", ios::app);
	static ofstream fresets(".\\motorData\\resetsFixaton.csv", ios::app);

	int cnt=0;

	//for(int index=0; index<5000; index++){
	for(int index=6155; index<12367; index++){
	//for(int index=12367; index<12367 + 10000; index++){
		move_visual_target_eye_training(visualTarget.get(), curTime);
		gLink* base = SysLEye->findLink("Eyeball");
		//gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
		//gVec3 localTargetPos = targetWorldPos-base->frame().trn();
		//targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());
	
		gVec3 baseWorldPos =base->frame().trn();
		gVec3 targetWorldPos(baseWorldPos.x() + scanpath[index][0]-600, baseWorldPos.y() - 1200, baseWorldPos.z() + scanpath[index][1] - 600);
		cout << targetWorldPos.x() << ", " << targetWorldPos.y() << ", " << targetWorldPos.z() << endl;
		gVec3 localTargetPos = targetWorldPos-base->frame().trn();
		targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());
	

		doOneStepPoseBasedTargetInverseDynamicsControlForGenerateTrainingDatasetLeye_fixaton(foutputDelta, finput, fresets, targeting_location, scanpath[index][3] - scanpath[index][2]);

		cout << index++ <<endl;
		//cout << "size of the scanpath data " << sizeof(scanpath)/sizeof(scanpath[0]) << endl;
		//if(index==122743)
	}
	
	foutputDelta.close();
	finput.close();
	fresets.close();
	finish();
	exit(0);
}

void generateTrainingDatasetMotorWithDisplay_fixaton()
{
	static ofstream foutputDelta(".\\motorData\\outputTestDeltaMotorFixaton.csv", ios::app);
	static ofstream finput(".\\motorData\\inputTestMotorFixaton.csv", ios::app);
	static ofstream fresets(".\\motorData\\resetsFixaton.csv", ios::app);
	
	static float current_time = 0;
	static int index = 0;
		
	move_visual_target_eye_training(visualTarget.get(), curTime);
	gLink* base = SysLEye->findLink("Eyeball");
	//gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
	//gVec3 localTargetPos = targetWorldPos-base->frame().trn();
	//targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());
	
	gVec3 baseWorldPos =base->frame().trn();
	gVec3 targetWorldPos(baseWorldPos.x() + scanpath[index][0]-600, baseWorldPos.y() - 1200, baseWorldPos.z() + scanpath[index][1] - 600);
	cout << targetWorldPos.x() << ", " << targetWorldPos.y() << ", " << targetWorldPos.z() << endl;
	gVec3 localTargetPos = targetWorldPos-base->frame().trn();
	targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());
	
	if(scanpath[index][3]> current_time)
		doOneStepPoseBasedTargetInverseDynamicsControlForGenerateTrainingDatasetLeye_fixaton(foutputDelta, finput, fresets, targeting_location, scanpath[index][3] - scanpath[index][2]);

	if(current_time < scanpath[index+1][4])
		cout << index++ <<endl;

	current_time+= displayTimeStep;
	//cout << "size of the scanpath data " << sizeof(scanpath)/sizeof(scanpath[0]) << endl;
	//if(index==122743)
	if(index==63)
	{
		cout << "end of data" << endl;
		exit(1);
	}
	foutputDelta.close();
	finput.close();
}

void testLoopKinematicAndDNNFixatonsDisplay()
{
	bool kinematcFlag = false;

	static ofstream foutputDelta(".\\motorData\\outputTestDeltaMotorFixaton.csv", ios::app);
	static ofstream finput(".\\motorData\\inputTestMotorFixaton.csv", ios::app);
	static ofstream fresets(".\\motorData\\resetsFixaton.csv", ios::app);
	
	static float current_time = 0;
	static int index = 0;
		
	move_visual_target_eye_training(visualTarget.get(), curTime);
	gLink* base = SysLEye->findLink("Eyeball");
	//gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
	//gVec3 localTargetPos = targetWorldPos-base->frame().trn();
	//targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());
	
	gVec3 baseWorldPos =base->frame().trn();
	//cout << baseWorldPos.x() << endl;
	gVec3 targetWorldPos(baseWorldPos.x() + scanpath[index][0]-175, baseWorldPos.y() - 1200, baseWorldPos.z() + scanpath[index][1] - 300);
	//gVec3 targetWorldPos(baseWorldPos.x() + scanpath[index][0], baseWorldPos.y() - 1200, baseWorldPos.z() + scanpath[index][1]);
//gVec3 targetWorldPos(scanpath[index][0]-175, - 1200, baseWorldPos.z() + scanpath[index][1] - 300);
	cout << "Target position: " << targetWorldPos.x() << ", " << targetWorldPos.y() << ", " << targetWorldPos.z() << endl;
	gVec3 localTargetPos = targetWorldPos-base->frame().trn();
	targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());
		
	cout << "Current time: " << current_time << endl;
	cout << "Fixation start time: " << scanpath[index][2] << endl;
	cout << "Fixation finishing time: " << scanpath[index][3] << endl;

	if(kinematcFlag){
	/*	if(current_time > scanpath[index][3])
		{
			testKinematicAndDNNFixatons(foutputDelta, finput, fresets, targeting_location, scanpath[index][3] - scanpath[index][2]);
			if(current_time > scanpath[index+1][4])
				cout << index++ <<endl;
		}
		current_time+= displayTimeStep;*/
		testKinematicAndDNNFixatons(foutputDelta, finput, fresets, targeting_location, scanpath[index][3] - scanpath[index][2], kinematcFlag);
		cout << index++ <<endl;
		current_time+= displayTimeStep;
	}else{
		if(current_time > scanpath[index][2])
		{
			testKinematicAndDNNFixatons(foutputDelta, finput, fresets, targeting_location, scanpath[index][3] - scanpath[index][2], kinematcFlag);
			if(current_time > scanpath[index][3])
				cout << index++ <<endl;
		}else{
			if(index>0)
			{
				
				gVec3 targetWorldPos2(baseWorldPos.x() + scanpath[index-1][0]-175, baseWorldPos.y() - 1200, baseWorldPos.z() + scanpath[index-1][1] - 300);
				localTargetPos = targetWorldPos2-base->frame().trn();
				targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());
				testKinematicAndDNNFixatons(foutputDelta, finput, fresets, targeting_location, scanpath[index-1][3] - scanpath[index-1][2], kinematcFlag);

			}
		}
		current_time+= displayTimeStep;
	}
	//cout << "size of the scanpath data " << sizeof(scanpath)/sizeof(scanpath[0]) << endl;
	//if(index==122743)
	if(index==13)
	{
		cout << "end of data" << endl;
		foutputDelta.close();
		finput.close();
		fresets.close();
		exit(1);
	}
}

void generateTrainingDatasetPupil()
{
	unsigned int i;
	int cnt=0;
	int iterations = 100;
	final_record_iteration_pupil = false;

	//500000
	while(cnt <= iterations)
	{
		if(cnt == iterations) {
			final_record_iteration_pupil = true;
		}
		move_visual_target_pupil_training(visualTarget.get(), curTime);
		gVec3 ori_leye = getcurrent_ori_leye();
		gLink* base = SysLEye->findLink("Eyeball");
		gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
		gVec3 localTargetPos = targetWorldPos-base->frame().trn();
		targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());
		osg::Vec3 eye(base->frame().trn().x(),base->frame().trn().y(),base->frame().trn().z());

		/*osg::Vec3d spherePos = visualTarget.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local = Vec3f_local(float(spherePos.x()),float(spherePos.y()),float(spherePos.z()));
		Vec3f_local returnedImage[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];*/

		osg::Vec3d spherePos1 = visualTarget.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local1 = Vec3f_local(float(spherePos1.x()),float(spherePos1.y()),float(spherePos1.z()));

		vector<Vec3f_local> spehere_poss;
		spehere_poss.push_back(sherePos_local1);

#ifdef SECOND_VISUAL_TARGET
		osg::Vec3d spherePos2 = visualTarget2.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local2 = Vec3f_local(float(spherePos2.x()),float(spherePos2.y()),float(spherePos2.z()));
		
		spehere_poss.push_back(sherePos_local2);
#endif

		Vec3f_local returnedImagePupil[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];
		Vec3f_local returnedImageLens[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];
		Vec3f_local returnedImageFoveation[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];
		//raytracer(sherePos_local, SPHERE_SIZE, Vec3f_local(eye.x(),eye.y(), eye.z()), Vec3f_local(ori_leye.x()*gDTR,0,ori_leye.z()*gDTR),returnedImage,0);
		raytracer(spehere_poss, SPHERE_SIZE, Vec3f_local(eye.x(),eye.y(), eye.z()), Vec3f_local(ori_leye.x()*gDTR,0,ori_leye.z()*gDTR),returnedImagePupil,returnedImageLens,returnedImageFoveation,0);

		int repeat = (int)(displayTimeStep/simulationTimeStep) + 1;
		for(int i=0;i<repeat;++i){
			SysLEye->clearAllForcesWithoutMuscle();
			SysLEye->updateExtForce();
			SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
			simCount++;
			curTime += simulationTimeStep;
		}
		compute_pupil_size();
		SysLEye->adjust_pupil(pupil_size);
		compute_lens_size();
		SysLEye->adjust_cornea();
		
		cnt++;
		if(cnt%10==0)
			cout << "iteration = " << cnt << endl;
		start_recording_pupil = true;
	}
	start_recording_pupil = false;
	finputpupil.close();
	finputpupilact.close();
	foutputpupil.close();
	foutputpupilballpos.close();
	foutputpupilrandomlensact.close();
	foutputpupilutility.close();
	foutputpupilsize.close();
	finish();
	exit(0);
}

void generateTrainingDatasetLens()
{
	unsigned int i;
	int cnt=0;
#ifdef FULL_LENS_DATASET_GENERATION
	int iterations = 10000;
#else
	int iterations = 100;
#endif
	final_record_iteration_lens = false;

	//500000
	while(cnt <= iterations)
	{
		if(cnt == iterations) {
			final_record_iteration_lens = true;
		}
		//move_visual_target_lens_training(visualTarget.get(), curTime);
		move_visual_target_lens_training_new(visualTarget.get(), curTime);
		gVec3 ori_leye = getcurrent_ori_leye();
		gLink* base = SysLEye->findLink("Eyeball");
		gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
		gVec3 localTargetPos = targetWorldPos-base->frame().trn();
		targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());
		osg::Vec3 eye(base->frame().trn().x(),base->frame().trn().y(),base->frame().trn().z());
		
		/*osg::Vec3d spherePos = visualTarget.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local = Vec3f_local(float(spherePos.x()),float(spherePos.y()),float(spherePos.z()));
		Vec3f_local returnedImage[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];*/

		osg::Vec3d spherePos1 = visualTarget.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local1 = Vec3f_local(float(spherePos1.x()),float(spherePos1.y()),float(spherePos1.z()));

		vector<Vec3f_local> spehere_poss;
		spehere_poss.push_back(sherePos_local1);

#ifdef SECOND_VISUAL_TARGET

		osg::Vec3d spherePos2 = visualTarget2.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local2 = Vec3f_local(float(spherePos2.x()),float(spherePos2.y()),float(spherePos2.z()));
		
		spehere_poss.push_back(sherePos_local2);
#endif

		Vec3f_local returnedImagePupil[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];
		Vec3f_local returnedImageLens[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];
		Vec3f_local returnedImageFoveation[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];
		//raytracer(sherePos_local, SPHERE_SIZE, Vec3f_local(eye.x(),eye.y(), eye.z()), Vec3f_local(ori_leye.x()*gDTR,0,ori_leye.z()*gDTR),returnedImage,0);
		raytracer(spehere_poss, SPHERE_SIZE, Vec3f_local(eye.x(),eye.y(), eye.z()), Vec3f_local(ori_leye.x()*gDTR,0,ori_leye.z()*gDTR),returnedImagePupil,returnedImageLens,returnedImageFoveation,0);

		int repeat = (int)(displayTimeStep/simulationTimeStep) + 1;
		for(int i=0;i<repeat;++i){
			SysLEye->clearAllForcesWithoutMuscle();
			SysLEye->updateExtForce();
			SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
			simCount++;
			curTime += simulationTimeStep;
		}
		compute_pupil_size();
		SysLEye->adjust_pupil(pupil_size);
		compute_lens_size();
		SysLEye->adjust_cornea();
		
		cnt++;
		if(cnt%10==0)
			cout << "iteration = " << cnt << endl;
		start_recording_lens = true;
	}
	start_recording_lens = false;
	finputlens.close();
	//finputlenswidth.close();
	finputlensact.close();
	foutputlens.close();
	foutputlensballpos.close();
	foutputpupilrandompupilact.close();
	finish();
	exit(0);
}

void generateTrainingDatasetFoveation()
{
	unsigned int i;
	int cnt=0;
#ifdef FULL_FOVEATION_DATASET_GENERATION
	int iterations = 10000;
#else
	int iterations = 100;
#endif

	/*clock_t tStart = clock();
	double priorTimeStamp = (double) tStart;*/

	while(cnt <= iterations)
	{
		move_visual_target_foveation_training(visualTarget.get(), curTime);
		gVec3 ori_leye = getcurrent_ori_leye();
		gLink* base = SysLEye->findLink("Eyeball");
		gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
		gVec3 localTargetPos = targetWorldPos-base->frame().trn();
		targeting_location.set(localTargetPos.x(),localTargetPos.y(),localTargetPos.z());
		osg::Vec3 eye(base->frame().trn().x(),base->frame().trn().y(),base->frame().trn().z());

		osg::Vec3d spherePos1 = visualTarget.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local1 = Vec3f_local(float(spherePos1.x()),float(spherePos1.y()),float(spherePos1.z()));

		vector<Vec3f_local> spehere_poss;
		spehere_poss.push_back(sherePos_local1);

		Vec3f_local returnedImagePupil[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];
		Vec3f_local returnedImageLens[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];
		Vec3f_local returnedImageFoveation[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];
		//raytracer(sherePos_local, SPHERE_SIZE, Vec3f_local(eye.x(),eye.y(), eye.z()), Vec3f_local(ori_leye.x()*gDTR,0,ori_leye.z()*gDTR),returnedImage,0);
		setRandomPupilAndLensLengths();
		raytracer(spehere_poss, SPHERE_SIZE, Vec3f_local(eye.x(),eye.y(), eye.z()), Vec3f_local(ori_leye.x()*gDTR,0,ori_leye.z()*gDTR),returnedImagePupil,returnedImageLens,returnedImageFoveation,0);

		/*int repeat = (int)(displayTimeStep/simulationTimeStep) + 1;
		for(int i=0;i<repeat;++i){
			SysLEye->clearAllForcesWithoutMuscle();
			SysLEye->updateExtForce();
			SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
			simCount++;
			curTime += simulationTimeStep;
		}
		compute_pupil_size();
		SysLEye->adjust_pupil(pupil_size);
		compute_lens_size();
		SysLEye->adjust_cornea();*/

		cnt++;
		if(cnt%10==0) {
			cout << "iteration = " << cnt << endl;
			/*double curTime = (double)(clock());
			cout << "time taken: " << (curTime-priorTimeStamp) / CLOCKS_PER_SEC << endl;
			priorTimeStamp = curTime;*/
		}
		start_recording_foveation = true;
	}
	start_recording_foveation = false;
	finputperception.close();
	foutputperception.close();
	foutputperceptionballpos.close();
	foutputpupilrandompupilactperception.close();
	foutputpupilrandomlensactperception.close();
	finish();
	exit(0);
}


void doInverseDynamicMuscleControlLeyeAndOutputDataThatIsLessExtensiveThanTrainingDataButStillUsefulForPlotting(gVec3 v)
{
	//static ofstream foutput("outputLeyeActivationsForPlotting.csv", ios::app);
	static ofstream foutputori("outputOriLeye.csv", ios::app);
	static ofstream foutputvel("outputVelLeye.csv", ios::app);
	static ofstream foutputacc("outputAccLeye.csv", ios::app);

	v.normalize();
	doInverseDynamicMuscleControlLeye(v);

	gVec3 ori_leye = getcurrent_ori_leye();

	foutputori << ori_leye.x() << ",";
	foutputori << ori_leye.y() << ",";
	foutputori << ori_leye.z() << "," << endl;
	foutputori.flush();

	foutputvel << abs(SysLEye->eyeball->frameVel().rot().x()) << ",";
	foutputvel << abs(SysLEye->eyeball->frameVel().rot().y()) << ",";
	foutputvel << abs(SysLEye->eyeball->frameVel().rot().z()) << "," << endl;
	foutputvel.flush();

	foutputacc << abs(SysLEye->eyeball->frameAcc().rot().x()) << ",";
	foutputacc << abs(SysLEye->eyeball->frameAcc().rot().y()) << ",";
	foutputacc << abs(SysLEye->eyeball->frameAcc().rot().z()) << "," << endl;
	foutputvel.flush();

	//for(int i=0; i<SysLEye->numMuscles()-1; i++){
	//	foutput << GV_get(SysLEye->actLevel(), i) << ",";
	//}
	//foutput << GV_get(SysLEye->actLevel(), SysLEye->numMuscles()-1) << endl;
	//foutput.flush();
}

void resetToInitialPoseLeye()
{	
	SysLEye->clearAllForces();
	GV_set_zero(SysLEye->acc());
	GV_set_zero(SysLEye->vel()); 
	SysLEye->setZeroCoord();

	SysLEye->updateKVariablesUptoAcc();

	compute_pupil_size();
	SysLEye->adjust_pupil(pupil_size);
	compute_lens_size();
	SysLEye->adjust_cornea();
}

void doInverseDynamicMuscleControlLeye(gVec3 v)
{
#ifdef ONLINE_FOVEATION
	double theta_final = eye_cyl;
	double phi_final = eye_cxl;
#else
	double theta_final = theta_vec_conversion(v);
	double phi_final = -phi_vec_conversion(v);
#endif

	//idcyl = theta_final;
	//idcxl = phi_final;

	bool target_leye_set = setTargetJointAngleLEye(theta_final, phi_final);

	if(!target_leye_set){
		cout << "***IK not solvable" << endl;
		leye_IK_solved = false;
		resetToInitialPoseLeye();
		return;
	}

	else if(leye_IK_solved || new_target_ball)
	{
		leye_IK_solved = true;

		// IK 
		//SysLEye->set_jointFrom_D_e();
		//SysLEye->updateKVariablesUptoPos();

		// Acc - DOES NOT WORK WTF
		//GV_set_zero(SysLEye->acc());
		//setDesiredAccHNTALLeye(displayTimeStep/3, 0.1);
		//setDesiredAccHNTALLeye(displayTimeStep/3, 0.1);
		//setTieredAccLeye(2.0);
		//SysLEye->timeIntegrateExplicitEuler(displayTimeStep);
		//SysLEye->updateKVariablesUptoVel();

		// Torque
		//GV_set_zero(SysLEye->acc());
		///*setDesiredAccHNTALLeye(displayTimeStep/3, 0.1);
		//SysLEye->sol_invDynForMuscleDrivenJointForBackwardEuler(simulationTimeStep);
		//setDesiredAccHNTALLeye(displayTimeStep/5.0, 0.0);
		//SysLEye->sol_invDynForMuscleDrivenJointForBackwardEuler(displayTimeStep);*/
		setDesiredAccHNTALLeye(displayTimeStep, 0.0);
		SysLEye->sol_invDynForMuscleDrivenJointForBackwardEuler(displayTimeStep);
	
		//calculateMuscleForceLEye();
		//calculateMuscleForceLEyeNew();

		for(int i = 6; i < 14; i++) {
			//SysLEye->muscle(i)->setActLevel(0.5*sin(1.0*frame/20.0)+0.5);
			SysLEye->muscle(i)->setActLevel(1.0);
		}

		int repeat = (int)(displayTimeStep/simulationTimeStep);// + 1;
		for(int i=0;i<repeat;++i){
			//simulation
			SysLEye->clearAllForcesWithoutMuscle();
			
			SysLEye->updateExtForce();
			SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
			//simulation
			simCount++;

			curTime += simulationTimeStep;
		}

		compute_pupil_size();
		SysLEye->adjust_pupil(pupil_size);
		compute_lens_size();
		SysLEye->adjust_cornea();
	}
	
}

void doInverseDynamicMuscleControlLeyeNew(gVec3 v)
{
#ifdef ONLINE_FOVEATION
	double theta_final = eye_cyl;
	double phi_final = eye_cxl;
#else
	double theta_final = theta_vec_conversion(v);
	double phi_final = -phi_vec_conversion(v);
#endif

	gVec3 ori_leye = getcurrent_ori_leye();
	idcyl = theta_final - ori_leye.z()*gDTR;
	idcxl = phi_final - ori_leye.x()*gDTR;

	bool failedMuscleFlag = false;
	bool target_leye_set = setTargetJointAngleLEye(theta_final, phi_final);

	if(!target_leye_set){
		cout << "***IK not solvable" << endl;
		leye_IK_solved = false;
		resetToInitialPoseLeye();
		return;
	}

	else if((leye_IK_solved && leye_muscle_solved) || new_target_ball)
	{
		leye_IK_solved = true;
		leye_muscle_solved = true;

		// IK 
		/*SysLEye->set_jointFrom_D_e();
		SysLEye->updateKVariablesUptoPos();*/

		// Acc
		/*setDesiredAccHNTALLeye(displayTimeStep, 0.0);
		SysLEye->timeIntegrateExplicitEuler(displayTimeStep);
		SysLEye->updateKVariablesUptoVel();*/

		// Torque
		//SysLEye->clearAllForces();
		//SysLEye->updateExtForce();

		//setDesiredAccHNTALLeye(displayTimeStep, 0.0);
		//SysLEye->sol_invDynForMuscleDrivenJoint();

		////cout << "ID Acc: " << endl;
		////for(int i = 0 ; i < SysLEye->dof() ; i++)
		////{
		////	cout << GV_get(SysLEye->acc(),i) << endl;
		////}

		////cout << endl;

		////cout << "ID Torque: " << endl;
		////for(int i = 0 ; i < SysLEye->dof() ; i++)
		////{
		////	cout << GV_get(SysLEye->force(),i) << endl;
		////}

		////cout << endl;

		//SysLEye->computeAccelerationFeatherstone();

		////cout << "Actual Acc: " << endl;
		////for(int i = 0 ; i < SysLEye->dof() ; i++)
		////{
		////	cout << GV_get(SysLEye->acc(),i) << endl;
		////}

		//GV* acc = GV_alloc(SysLEye->dof());
		//GV_memcpy(acc,SysLEye->acc());

		//int repeat = (int)(displayTimeStep/simulationTimeStep);
		////if(frame < 2) {
		//for(int i=0;i<repeat;++i){
		//	GV_memcpy(SysLEye->acc(),acc);
		//	SysLEye->nextStepImplicitEuler1stOrderHigh(simulationTimeStep);
		//}
		////}

		// Muscle
		bool useMuscleActivations = true;
		double gamma = 0.0;

		// comment out if else block for normal, they are only for gravity
		/*static int cnt = 0;
		if(cnt < 500) {
			cnt++;
		}
		else {*/
		do {
			SysLEye->clearAllForcesWithoutCilliaryAndSphincterMuscle();
			SysLEye->updateExtForce();

			//setDesiredAccHNTALLeye(displayTimeStep, gamma);
			setMaxVelAccLeye(displayTimeStep, gamma);
			SysLEye->sol_invDynForMuscleDrivenJoint();

			for(int i = 0; i < 6; i++) {
				GV_set(SysLEye->actLevel(),i,0);
			}
			useMuscleActivations = calculateMuscleForceLEyeNew();
			if(!useMuscleActivations) {
				gamma += 0.01;
				cout << "Failed to find solution. Retrying with gamma " << gamma << endl;
				if(gamma >= 0.9) { // Unstable state - force reset
					failedMuscleFlag = true;
					break;
					cout << "Reached unstable state and unable to continue simulation. Resetting state" << endl;
				}
			}
		} while(!useMuscleActivations);
		
		
		if(failedMuscleFlag) {
			leye_muscle_solved = false;
			resetToInitialPoseLeye();
			return;
		}
		else {
			//double act = 0.5*sin(1.0*frame/20.0)+0.5;
			/*double act = 0;
			for(int i = 6; i < 14; i++) {
				SysLEye->muscle(i)->setActLevel(act);
			}
			for(int i = 14; i < 22; i++) {
				SysLEye->muscle(i)->setActLevel(act);
			}
			for(int i = 38; i < 54; i++) {
				SysLEye->muscle(i)->setActLevel(0.038*(1-act));
			}*/

			SysLEye->clearAllForcesWithoutMuscle();
			SysLEye->updateExtForce();
			SysLEye->computeAccelerationFeatherstone();

			/*for(int i = 3; i < SysLEye->dof(); i++) {
				GV_set(SysLEye->acc(),i,0.0);
			}*/

			int repeat = (int)(displayTimeStep/simulationTimeStep);
			for(int i=0;i<repeat;++i){
				SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
			}

			compute_pupil_size();
			SysLEye->adjust_pupil(pupil_size);
			compute_lens_size();
			SysLEye->adjust_cornea();
		}
	}
	
	//cout << "frame " << frame << endl;
}




void doInverseDynamicMuscleControlLeyeNew_fixaton(gVec3 v, float time)
{
#ifdef ONLINE_FOVEATION
	double theta_final = eye_cyl;
	double phi_final = eye_cxl;
#else
	double theta_final = theta_vec_conversion(v);
	double phi_final = -phi_vec_conversion(v);
#endif

	gVec3 ori_leye = getcurrent_ori_leye();
	idcyl = theta_final - ori_leye.z()*gDTR;
	idcxl = phi_final - ori_leye.x()*gDTR;

	bool failedMuscleFlag = false;
	bool target_leye_set = setTargetJointAngleLEye(theta_final, phi_final);

	if(!target_leye_set){
		//cout << "***IK not solvable" << endl;
		leye_IK_solved = false;
		resetToInitialPoseLeye();
		return;
	}

	else if((leye_IK_solved && leye_muscle_solved) || new_target_ball)
	{
		leye_IK_solved = true;
		leye_muscle_solved = true;

		// IK 
		SysLEye->set_jointFrom_D_e();
		SysLEye->updateKVariablesUptoPos();

		// Acc
		/*setDesiredAccHNTALLeye(displayTimeStep, 0.0);
		SysLEye->timeIntegrateExplicitEuler(displayTimeStep);
		SysLEye->updateKVariablesUptoVel();*/

		// Torque
		//SysLEye->clearAllForces();
		//SysLEye->updateExtForce();

		//setDesiredAccHNTALLeye(displayTimeStep, 0.0);
		//SysLEye->sol_invDynForMuscleDrivenJoint();

		////cout << "ID Acc: " << endl;
		////for(int i = 0 ; i < SysLEye->dof() ; i++)
		////{
		////	cout << GV_get(SysLEye->acc(),i) << endl;
		////}

		////cout << endl;

		////cout << "ID Torque: " << endl;
		////for(int i = 0 ; i < SysLEye->dof() ; i++)
		////{
		////	cout << GV_get(SysLEye->force(),i) << endl;
		////}

		////cout << endl;

		//SysLEye->computeAccelerationFeatherstone();

		////cout << "Actual Acc: " << endl;
		////for(int i = 0 ; i < SysLEye->dof() ; i++)
		////{
		////	cout << GV_get(SysLEye->acc(),i) << endl;
		////}

		//GV* acc = GV_alloc(SysLEye->dof());
		//GV_memcpy(acc,SysLEye->acc());

		//int repeat = (int)(displayTimeStep/simulationTimeStep);
		////if(frame < 2) {
		//for(int i=0;i<repeat;++i){
		//	GV_memcpy(SysLEye->acc(),acc);
		//	SysLEye->nextStepImplicitEuler1stOrderHigh(simulationTimeStep);
		//}
		////}

		// Muscle
		//bool useMuscleActivations = true;
		//double gamma = 0.0;

		//// comment out if else block for normal, they are only for gravity
		//static int cnt = 0;
		///*if(cnt < 500) {
		//	cnt++;
		//}
		//else {*/
		//do {
		//	SysLEye->clearAllForcesWithoutCilliaryAndSphincterMuscle();
		//	SysLEye->updateExtForce();

		//	//setDesiredAccHNTALLeye(displayTimeStep, gamma);
		//	setMaxVelAccLeye(time, gamma);
		//	SysLEye->sol_invDynForMuscleDrivenJoint();

		//	for(int i = 0; i < 6; i++) {
		//		GV_set(SysLEye->actLevel(),i,0);
		//	}
		//	useMuscleActivations = calculateMuscleForceLEyeNew();
		//	if(!useMuscleActivations) {
		//		gamma += 0.01;
		//		cout << "Failed to find solution. Retrying with gamma " << gamma << endl;
		//		if(gamma >= 0.9) { // Unstable state - force reset
		//			failedMuscleFlag = true;
		//			break;
		//			cout << "Reached unstable state and unable to continue simulation. Resetting state" << endl;
		//		}
		//	}
		//} while(!useMuscleActivations);
		//}
		//
		//if(failedMuscleFlag) {
		//	leye_muscle_solved = false;
		//	resetToInitialPoseLeye();
		//	return;
		//}
		//else {
		//	//double act = 0.5*sin(1.0*frame/20.0)+0.5;
		//	/*double act = 0;
		//	for(int i = 6; i < 14; i++) {
		//		SysLEye->muscle(i)->setActLevel(act);
		//	}
		//	for(int i = 14; i < 22; i++) {
		//		SysLEye->muscle(i)->setActLevel(act);
		//	}
		//	for(int i = 38; i < 54; i++) {
		//		SysLEye->muscle(i)->setActLevel(0.038*(1-act));
		//	}*/

		//	SysLEye->clearAllForcesWithoutMuscle();
		//	SysLEye->updateExtForce();
		//	SysLEye->computeAccelerationFeatherstone();

		//	/*for(int i = 3; i < SysLEye->dof(); i++) {
		//		GV_set(SysLEye->acc(),i,0.0);
		//	}*/

		//	int repeat = (int)(displayTimeStep/simulationTimeStep);
		//	for(int i=0;i<repeat;++i){
		//		SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
		//	}

		//	/*compute_pupil_size();
		//	SysLEye->adjust_pupil(pupil_size);
		//	compute_lens_size();
		//	SysLEye->adjust_cornea();*/
		//}
	}
	
	//cout << "frame " << frame << endl;
}



void doInverseKinematicsControlLeyeNew_fixaton(gVec3 v, float time)
{
#ifdef ONLINE_FOVEATION
	double theta_final = eye_cyl;
	double phi_final = eye_cxl;
#else
	double theta_final = theta_vec_conversion(v);
	double phi_final = -phi_vec_conversion(v);
#endif

	gVec3 ori_leye = getcurrent_ori_leye();
	idcyl = theta_final - ori_leye.z()*gDTR;
	idcxl = phi_final - ori_leye.x()*gDTR;

	bool failedMuscleFlag = false;
	bool target_leye_set = setTargetJointAngleLEye(theta_final, phi_final);

	if(!target_leye_set){
		//cout << "***IK not solvable" << endl;
		leye_IK_solved = false;
		resetToInitialPoseLeye();
		return;
	}
	leye_IK_solved = true;
	leye_muscle_solved = true;

	// IK 
	SysLEye->set_jointFrom_D_e();
	SysLEye->updateKVariablesUptoPos();

	compute_pupil_size();
	SysLEye->adjust_pupil(pupil_size);
	compute_lens_size();
	SysLEye->adjust_cornea();

	//cout << "frame " << frame << endl;
}


void setRandomPupilAndLensLengths()
{
	double pupilRadius = randomNumber(1.69055,4.50033);
	double lensWidth = randomNumber(1.26359,5.37659);

	SysLEye->adjust_pupil(pupilRadius);
	SysLEye->adjust_lens(lensWidth);
	SysLEye->adjust_cornea();
}

void doForwardDynamicMuscleControlLeye(int muscle_index, double actLevel) 
{
	
	SysLEye->muscle(muscle_index)->setActLevel(actLevel);

	//simulationTimeStepExplicitEuler
	int repeat = (int)(displayTimeStep/simulationTimeStep) + 1;
	for(int i=0;i<repeat;++i){
		SysLEye->clearAllForcesWithoutMuscle();
		SysLEye->updateExtForce();
		
		SysLEye->nextStepExplicitEulerFeatherstone(simulationTimeStep);
		//SysLEye->adjust_pupil(pupil_size);

		curTime += simulationTimeStep;
		simCount++;
	}
	compute_pupil_size();
	SysLEye->adjust_pupil(pupil_size);
	compute_lens_size();
	SysLEye->adjust_cornea();

}

void testGeneratedDatasetFromTrainingLeyeMotor()
{
	static bool bRead = false;
	int num_eoc_muscles = 6;
	static int cur = 0;
	int i=0;
	int p=0;
	int pd = 0;
	int q=0;
	int qd = 0;
	static int totalRow = 0;
	static int totalRowDelta = 0;
	static int totalRowReset = 0;
	float base [6];
	if(!bRead){
		ifstream ifs(".\\motorData\\outputTestDeltaMotorFixaton.csv", ios::app);
		ifstream finput(".\\motorData\\inputTestMotorFixaton.csv", ios::app);
		ifstream fresets(".\\motorData\\resetsFixaton.csv", ios::app);
		if(!ifs){
			cout << "Could not open delta file. Exiting" << endl;
			return;
		}
		else if(!finput){
			cout << "Could not open input file. Exiting" << endl;
			return;
		}
		string str;
		string inp;
		string res;
		while(getline(finput,inp)){
			cout << "Reading line " << totalRow << ": " << inp << endl;
			string token;
			istringstream stream(inp);
			while(getline(stream,token,',')){
				double temp=stod(token);
				muscle_activation_data_leye[totalRow][q] = temp;
				q++;
				if(q >= num_eoc_muscles+2){
					q = 0;
				}
			}
			totalRow++;
			if(totalRow >= 1){
				break;
			}
	    }
		while(getline(ifs,str)){
			cout << "Reading delta line " << totalRowDelta << ": " << str << endl;
			string token;
			istringstream stream(str);
			while(getline(stream,token,',')){
				double temp=stod(token);
				muscle_activation_data_delta_leye[totalRowDelta][qd] = temp;
				qd++;
				if(qd >= num_eoc_muscles){
					qd = 0;
				}
			}
			totalRowDelta++;
			if(totalRowDelta >= 5000){
				break;
			}
	    }
		while(getline(fresets,res)){
			cout << "Reading reset line " << totalRowReset << ": " << res << endl;
			resets_leye[totalRowReset] = stoi(res);
			totalRowReset++;
			if(totalRowReset >= 5000){
				break;
			}
	    }
		bRead = true;
		for(int t=0; t < num_eoc_muscles; t++){
			base[t] = muscle_activation_data_leye[0][t+2];
		}
		for(int t=0; t < num_eoc_muscles; t++){
			SysLEye->setActLevel(t,base[t]);
		}
	}

	i = 0;
	if(resets_leye[cur] == 1) {
		cout << "FORCED EYE RESET" << endl;
		resetToInitialPoseLeye();
	}

	/*cout << "eye acceleration: " << SysLEye->eyeball->frameAcc().rot().magnitude() << endl;
	cout << "eye velocity: " << SysLEye->eyeball->frameVel().rot().magnitude() << endl;*/

	//cout << std::fixed;
	for(int t=0; t < num_eoc_muscles; t++){
		//cout << "act and delta: " << setprecision(15) << SysLEye->muscle(t)->actLevel() << " " << muscle_activation_data_delta_leye[cur][t] << endl;
		long double currAccLevel = SysLEye->muscle(t)->actLevel();
		SysLEye->setActLevel(t,currAccLevel + (long double) muscle_activation_data_delta_leye[cur][t]);
	}
	cur++;
	cout << "this is interation " << cur << endl;
	if(cur>totalRowDelta){
		cout << "end of iteration. rewind to the top" << endl;
		bPlay = false;
	}

	static FILE* velocities = fopen("vels.csv","w");
	fprintf(velocities,"%g\n",SysLEye->eyeball->frameVel().rot().magnitude());

	SysLEye->clearAllForcesWithoutMuscle();
	SysLEye->updateExtForce();
	SysLEye->computeAccelerationFeatherstone();

	/*for(int i = 3; i < SysLEye->dof(); i++) {
		GV_set(SysLEye->acc(),i,0.0);
	}*/

	int repeat = (int)(displayTimeStep/simulationTimeStep);
	for(int i=0;i<repeat;++i){
		SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
	}

	compute_pupil_size();
	SysLEye->adjust_pupil(pupil_size);
	compute_lens_size();
	SysLEye->adjust_cornea();
	
	////GV_set_zero(SysLEye->acc());
	//SysLEye->clearAllForcesWithoutMuscle();
	//SysLEye->updateExtForce();
	////SysLEye->computeAccelerationFeatherstone();

	////for(int i = 3; i < SysLEye->dof(); i++) {
	////	GV_set(SysLEye->acc(),i,0.0); // clear acceleration and velocities for non EOC joints
	////	GV_set(SysLEye->vel(),i,0.0);
	////}

	//static GV* acc = GV_alloc(SysLEye->dof());
	//GV_memcpy(acc,SysLEye->acc());

	//int repeat = (int)(displayTimeStep/simulationTimeStep);
	//for(int i=0;i<repeat;++i){
	//	GV_memcpy(SysLEye->acc(),acc);
	//	SysLEye->nextStepImplicitEuler1stOrderHigh(simulationTimeStep);
	//}

	//compute_pupil_size();
	//SysLEye->adjust_pupil(pupil_size);
	//compute_lens_size();
	//SysLEye->adjust_cornea();

	/*for(i=0; i<repeat; i++)
	{
		SysLEye->clearAllForcesWithoutMuscle();
		SysLEye->updateExtForce();
		SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
		curTime += simulationTimeStep;
		simCount++;
	}*/
}

void testGeneratedDatasetFromTrainingLeyePupil()
{
	int numTestingSamples = 99;
	static bool bRead = false;
	static int cur = 0;
	static int totalRow = 0;
	static int totalRowDelta = 0;
	static int totalRowLensDelta = 0;
	static int totalRowBallPos = 0;
	static int totalRowImg = 0;

	static double image_leye_pupil_temp[1500][RAYTRACE_WIDTH*RAYTRACE_HEIGHT];

	if(!bRead){
		ifstream ifs(dirPrefix + "\\pupilData\\output_iris_sphincter_delta_act.csv");
		ifstream finput(dirPrefix + "\\pupilData\\iris_sphincter_act.csv");
		ifstream finputball(dirPrefix + "\\pupilData\\ballpos.csv");
		ifstream finputlenspupil(dirPrefix + "\\pupilData\\randomlensact.csv");
		ifstream finputimg(dirPrefix + "\\pupilData\\image_x_act_iris_sphincter.csv");
		if(!ifs){
			cout << "Could not open delta file. Exiting" << endl;
			return;
		}
		else if(!finput){
			cout << "Could not open input file. Exiting" << endl;
			return;
		}
		else if(!finputball){
			cout << "Could not open ball file. Exiting" << endl;
			return;
		}
		else if(!finputlenspupil){
			cout << "Could not open lens file. Exiting" << endl;
			return;
		}
		else if(!finputimg){
			cout << "Could not open image file. Exiting" << endl;
			return;
		}
		string str;
		string inp;
		string bp;
		string lens;
		string img;
		while(getline(finput,inp)){
			cout << "Reading line " << totalRow << ": " << inp << endl;
			string token;
			istringstream stream(inp);
			muscle_activation_data_leye_pupil_start = stof(inp);
			totalRow++;
			if(totalRow >= 1){
				break;
			}
	    }
		while(getline(ifs,str)){
			cout << "Reading delta line " << totalRowDelta << ": " << str << endl;
			string token;
			istringstream stream(str);
			muscle_activation_data_delta_leye_pupil[totalRowDelta] = stof(str);
			totalRowDelta++;
			if(totalRowDelta >= numTestingSamples){
				break;
			}
	    }
		while(getline(finputlenspupil,lens)){
			cout << "Reading lens line " << totalRowLensDelta << ": " << lens << endl;
			string token;
			istringstream stream(lens);
			random_lens_cilliary_act[totalRowLensDelta] = stof(lens);
			totalRowLensDelta++;
			if(totalRowLensDelta >= numTestingSamples){
				break;
			}
	    }
		while(getline(finputball,bp)){
			cout << "Reading ball pos line " << totalRowBallPos << ": " << bp << endl;
			string token;
			istringstream stream(bp);
			int qd = 0;
			while(getline(stream,token,',')){
				double temp=stof(token);
				ballpos_leye_pupil[totalRowBallPos][qd] = temp;
				qd++;
				if(qd >= 4){
					qd = 0;
				}
			}
			totalRowBallPos++;
			if(totalRowBallPos >= numTestingSamples){
				break;
			}
	    }
		while(getline(finputimg,img)) {
			cout << "Reading onv line " << totalRowImg << endl;
			string token;
			istringstream stream(img);
			int qd = 0;
			while(getline(stream,token,',')) {
				double temp=stof(token);
				image_leye_pupil_temp[totalRowImg][qd] = temp;
				qd++;
				if(qd >= RAYTRACE_WIDTH*RAYTRACE_HEIGHT){
					qd = 0;
					break;
				}
			}
			totalRowImg++;
			if(totalRowImg >= numTestingSamples){
				break;
			}
		}
		bRead = true;
		for(int i = 6; i < 14; i++) {
			SysLEye->muscle(i)->setActLevel(muscle_activation_data_leye_pupil_start);
		}
	}

	for(int t = 6; t < 14; t++) {
		double currAccLevel = SysLEye->muscle(t)->actLevel();
		SysLEye->muscle(t)->setActLevel(currAccLevel + muscle_activation_data_delta_leye_pupil[cur]);
	}
	for(int i = 14; i < 22; i++) {
		SysLEye->muscle(i)->setActLevel(random_lens_cilliary_act[cur]);
	}
	for(int i = 38; i < 54; i++) {
		SysLEye->muscle(i)->setActLevel(0.072*(1-random_lens_cilliary_act[cur]));
	}

	for(int k = 0; k < RAYTRACE_WIDTH*RAYTRACE_HEIGHT; k++) {
		image_leye_pupil[k] = image_leye_pupil_temp[cur][k];
	}

	vis_target_ec = ballpos_leye_pupil[cur][3];
	osg::Matrixd mat;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	
	osg::Vec3 v(ballpos_leye_pupil[cur][0], ballpos_leye_pupil[cur][1], ballpos_leye_pupil[cur][2]);
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	visualTarget.get()->setMatrix(mat);

	cur++;
	cout << "this is iteration " << cur << endl;
	if(cur>totalRowDelta){
		cout << "end of iteration. rewind to the top" << endl;
		bPlay = false;
	}

	int repeat = (int)(displayTimeStep/simulationTimeStep) + 1;
	for(int i=0; i<repeat; i++)
	{
		SysLEye->clearAllForcesWithoutMuscle();
		SysLEye->updateExtForce();
		SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
		curTime += simulationTimeStep;
		simCount++;
	}
	compute_pupil_size();
	SysLEye->adjust_pupil(pupil_size);
	compute_lens_size();
	SysLEye->adjust_cornea();
}

void testGeneratedDatasetFromTrainingLeyeLens()
{
	int numTestingSamples = 99;
	static bool bRead = false;
	static int cur = 0;
	static int totalRow = 0;
	static int totalRowDelta = 0;
	static int totalRowBallPos = 0;
	static int totalRowPupilAct = 0;
	static int totalRowImg = 0;

	static double image_leye_lens_temp[1500][RAYTRACE_WIDTH*RAYTRACE_HEIGHT];

	//muscle_activation_data_leye_lens_start = 0.0;
	if(!bRead){
		ifstream ifs(dirPrefix + "\\lensData\\output_lens_cilliary_delta_act.csv");
		ifstream finput(dirPrefix + "\\lensData\\cilliary_act.csv");
		ifstream finputball(dirPrefix + "\\lensData\\ballpos.csv");
		ifstream finputpupil(dirPrefix + "\\lensData\\randompupilact.csv");
		ifstream finputimg(dirPrefix + "\\lensData\\image_x_act_cilliary.csv");
		if(!ifs){
			cout << "Could not open delta file. Exiting" << endl;
			return;
		}
		else if(!finput){
			cout << "Could not open input file. Exiting" << endl;
			return;
		}
		else if(!finputball){
			cout << "Could not open ball file. Exiting" << endl;
			return;
		}
		else if(!finputpupil){
			cout << "Could not open random pupil act. Exiting" << endl;
			return;
		}
		else if(!finputimg){
			cout << "Could not open image file. Exiting" << endl;
			return;
		}
		string str;
		string inp;
		string bp;
		string pupil;
		string img;
		while(getline(finput,inp)){
			cout << "Reading line " << totalRow << ": " << inp << endl;
			string token;
			istringstream stream(inp);
			muscle_activation_data_leye_lens_start = stof(inp);
			totalRow++;
			if(totalRow >= 1){
				break;
			}
	    }
		while(getline(ifs,str)){
			cout << "Reading delta line " << totalRowDelta << ": " << str << endl;
			string token;
			istringstream stream(str);
			muscle_activation_data_delta_leye_lens[totalRowDelta] = stof(str);
			totalRowDelta++;
			if(totalRowDelta >= numTestingSamples){
				break;
			}
	    }
		while(getline(finputpupil,pupil)){
			cout << "Reading pupil line " << totalRowPupilAct << ": " << pupil << endl;
			string token;
			istringstream stream(pupil);
			random_pupil_sphincter_act[totalRowPupilAct] = stof(pupil);
			totalRowPupilAct++;
			if(totalRowPupilAct >= numTestingSamples){
				break;
			}
	    }
		while(getline(finputball,bp)){
			cout << "Reading ball pos line " << totalRowBallPos << ": " << bp << endl;
			string token;
			istringstream stream(bp);
			int qd = 0;
			while(getline(stream,token,',')){
				double temp=stof(token);
				ballpos_leye_lens[totalRowBallPos][qd] = temp;
				qd++;
				if(qd >= 4){
					qd = 0;
				}
			}
			totalRowBallPos++;
			if(totalRowBallPos >= numTestingSamples){
				break;
			}
	    }
		while(getline(finputimg,img)) {
			cout << "Reading onv line " << totalRowImg << endl;
			string token;
			istringstream stream(img);
			int qd = 0;
			while(getline(stream,token,',')) {
				double temp=stof(token);
				image_leye_lens_temp[totalRowImg][qd] = temp;
				qd++;
				if(qd >= RAYTRACE_WIDTH*RAYTRACE_HEIGHT){
					qd = 0;
					break;
				}
			}
			totalRowImg++;
			if(totalRowImg >= numTestingSamples){
				break;
			}
		}
		bRead = true;
		for(int i = 14; i < 22; i++) {
			SysLEye->muscle(i)->setActLevel(muscle_activation_data_leye_lens_start);
		}
		for(int i = 38; i < 54; i++) {
			SysLEye->muscle(i)->setActLevel(0.072*(1-muscle_activation_data_leye_lens_start));
		}
	}

	double currAccLevel = SysLEye->muscle(14)->actLevel();
	for(int t = 14; t < 22; t++) {
		SysLEye->muscle(t)->setActLevel(currAccLevel + muscle_activation_data_delta_leye_lens[cur]);
	}
	for(int t = 38; t < 54; t++) {
		SysLEye->muscle(t)->setActLevel(0.072*(1-(currAccLevel + muscle_activation_data_delta_leye_lens[cur])));
	}

	for(int i = 6; i < 14; i++) {
		SysLEye->muscle(i)->setActLevel(random_pupil_sphincter_act[cur]);
	}

	for(int k = 0; k < RAYTRACE_WIDTH*RAYTRACE_HEIGHT; k++) {
		image_leye_lens[k] = image_leye_lens_temp[cur][k];
	}

	vis_target_ec = ballpos_leye_lens[cur][3];
	osg::Matrixd mat;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	
	osg::Vec3 v(ballpos_leye_lens[cur][0], ballpos_leye_lens[cur][1], ballpos_leye_lens[cur][2]);
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	visualTarget.get()->setMatrix(mat);

	cur++;
	cout << "this is interation " << cur << endl;
	if(cur>totalRowDelta){
		cout << "end of iteration. rewind to the top" << endl;
		bPlay = false;
	}
	
	int repeat = (int)(displayTimeStep/simulationTimeStep) + 1;
	for(int i=0; i<repeat; i++)
	{
		SysLEye->clearAllForcesWithoutMuscle();
		SysLEye->updateExtForce();
		SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
		curTime += simulationTimeStep;
		simCount++;
	}
	compute_pupil_size();
	SysLEye->adjust_pupil(pupil_size);
	compute_lens_size();
	SysLEye->adjust_cornea();

}

void testGeneratedDatasetFromTrainingLeyeFoveation() {

	int numTestingSamples = 99;
	static bool bRead = false;
	static int cur = 0;
	static int totalRowImg = 0;
	static int totalRowEtp = 0;
	static int totalRowBallPos = 0;
	static int totalRowPupilAct = 0;
	static int totalRowLensAct = 0;

	static double image_leye_foveation_temp[1500][RAYTRACE_WIDTH*RAYTRACE_HEIGHT];

	if(!bRead){
		ifstream finputimg(dirPrefix + "\\perceptionData\\image_x.csv");
		ifstream foutput(dirPrefix + "\\perceptionData\\output_etheta_ephi.csv");
		ifstream finputball(dirPrefix + "\\perceptionData\\ballpos.csv");
		ifstream finputpupil(dirPrefix + "\\perceptionData\\randompupilact.csv");
		ifstream finputlens(dirPrefix + "\\perceptionData\\randomlensact.csv");
		
		if(!finputimg){
			cout << "Could not open image file. Exiting" << endl;
			return;
		}
		else if(!foutput){
			cout << "Could not open output file. Exiting" << endl;
			return;
		}
		else if(!finputball){
			cout << "Could not open ball file. Exiting" << endl;
			return;
		}
		else if(!finputpupil){
			cout << "Could not open random pupil act. Exiting" << endl;
			return;
		}
		else if(!finputlens){
			cout << "Could not open random pupil act. Exiting" << endl;
			return;
		}
		string img;
		string etp;
		string bp;
		string pupil;
		string lens;
		
		while(getline(finputimg,img)) {
			cout << "Reading onv line " << totalRowImg << endl;
			string token;
			istringstream stream(img);
			int qd = 0;
			while(getline(stream,token,',')) {
				double temp=stof(token);
				image_leye_foveation_temp[totalRowImg][qd] = temp;
				qd++;
				if(qd >= RAYTRACE_WIDTH*RAYTRACE_HEIGHT){
					qd = 0;
					break;
				}
			}
			totalRowImg++;
			if(totalRowImg >= numTestingSamples){
				break;
			}
		}
		while(getline(foutput,etp)) {
			cout << "Reading output line " << totalRowEtp << endl;
			string token;
			istringstream stream(etp);
			int qd = 0;
			while(getline(stream,token,',')) {
				double temp=stof(token);
				etheta_ephi_perception[totalRowEtp][qd] = temp;
				qd++;
				if(qd >= 2){
					qd = 0;
					break;
				}
			}
			totalRowEtp++;
			if(totalRowEtp >= numTestingSamples){
				break;
			}
		}
		while(getline(finputpupil,pupil)){
			cout << "Reading pupil line " << totalRowPupilAct << ": " << pupil << endl;
			string token;
			istringstream stream(pupil);
			random_pupil_sphincter_act_foveation[totalRowPupilAct] = stof(pupil);
			totalRowPupilAct++;
			if(totalRowPupilAct >= numTestingSamples){
				break;
			}
	    }
		while(getline(finputlens,lens)){
			cout << "Reading lens line " << totalRowLensAct << ": " << lens << endl;
			string token;
			istringstream stream(lens);
			random_lens_cilliary_act_foveation[totalRowLensAct] = stof(lens);
			totalRowLensAct++;
			if(totalRowLensAct >= numTestingSamples){
				break;
			}
	    }
		while(getline(finputball,bp)){
			cout << "Reading ball pos line " << totalRowBallPos << ": " << bp << endl;
			string token;
			istringstream stream(bp);
			int qd = 0;
			while(getline(stream,token,',')){
				double temp=stof(token);
				ballpos_leye_foveation[totalRowBallPos][qd] = temp;
				qd++;
				if(qd >= 4){
					qd = 0;
				}
			}
			totalRowBallPos++;
			if(totalRowBallPos >= numTestingSamples){
				break;
			}
	    }
		bRead = true;
	}

	SysLEye->adjust_pupil(random_pupil_sphincter_act_foveation[cur]);
	SysLEye->adjust_lens(random_lens_cilliary_act_foveation[cur]);

	for(int k = 0; k < RAYTRACE_WIDTH*RAYTRACE_HEIGHT; k++) {
		image_leye_perception[k] = image_leye_foveation_temp[cur][k];
	}

	vis_target_ec = ballpos_leye_foveation[cur][3];
	osg::Matrixd mat;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	
	osg::Vec3 v(ballpos_leye_foveation[cur][0], ballpos_leye_foveation[cur][1], ballpos_leye_foveation[cur][2]);
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	visualTarget.get()->setMatrix(mat);
		
	gLink* base = SysLEye->findLink("Eyeball");
	gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
	gVec3 localTargetPos = targetWorldPos-base->frame().trn();
	double theta = theta_vec_conversion(localTargetPos);
	double phi = -phi_vec_conversion(localTargetPos);

	cout << "Etha (reocrded), Etheta (live), diff: " << etheta_ephi_perception[cur][0] << " " << theta << " " << abs(etheta_ephi_perception[cur][0] - theta) << endl;
	cout << "Ephi (reocrded), Ephi (live), diff: " << etheta_ephi_perception[cur][1] << " " << phi << " " << abs(etheta_ephi_perception[cur][1] - phi) << endl;

	cur++;
	cout << "this is interation " << cur << endl;
	if(cur>totalRowImg){
		cout << "end of iteration. rewind to the top" << endl;
		bPlay = false;
	}
}

void doMuscleControlKerasLEye(gVec3 v)
{
	int num_eoc_muscles = 6;

	gVec3 ori_leye = getcurrent_ori_leye();
	double target_theta, target_phi;

#ifdef ONLINE_FOVEATION
	target_theta = eye_cyl;
	target_phi = eye_cxl;
#else
	target_theta = theta_vec_conversion(v);
	target_phi = -phi_vec_conversion(v);
#endif

	cout << "current theta and phi is " << ori_leye.z()*gDTR << ", " << ori_leye.x()*gDTR << endl;
	cout << "target theta and phi is " << target_theta << ", " << target_phi << endl;
	double error_theta = target_theta - ori_leye.z()*gDTR;
	double error_phi = target_phi - ori_leye.x()*gDTR;

	double input[8];

	input[0] = error_theta;
	input[1] = error_phi;

	cout << input[0] << ", " << input[1] << endl;

	static GV *prev_activationLeye = 0;
	
	if(!prev_activationLeye) prev_activationLeye = GV_alloc(num_eoc_muscles);
	 
	for(int j = 0; j < num_eoc_muscles; j++) {
		input[j+2] = GV_get(SysLEye->actLevel(), j);
		GV_set(prev_activationLeye,j, GV_get(SysLEye->actLevel(), j));
	}
	
	vector<float> vectorInput;
	for(int i = 0; i < num_eoc_muscles+2; i++) {
		vectorInput.push_back(input[i]);
	}
	sampleLEye->set_data(vectorInput);
	vector<float> returned = mLEye->compute_output(sampleLEye);
	float *aLevel = &returned[0];

	double param=1.0;
	for(int j=0; j < num_eoc_muscles; j++){
		double newAct = (double)aLevel[j]*stdValueLEye[j]+meanValueLEye[j] + GV_get(prev_activationLeye, j);
		cout << "prev, delta: " << GV_get(prev_activationLeye, j) << " " << newAct << endl;
		SysLEye->setActLevel(j,gMin(1.0, gMax(param*newAct,0)));
	}

	/*int repeat = (int)(displayTimeStep/simulationTimeStep) + 1;
	for(int i=0;i<repeat;++i){	
		SysLEye->clearAllForcesWithoutMuscle();
		SysLEye->updateExtForce();
		SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
		curTime += simulationTimeStep;
		simCount++;
	}
	compute_pupil_size();
	SysLEye->adjust_pupil(pupil_size);
	compute_lens_size();
	SysLEye->adjust_cornea();*/

	SysLEye->clearAllForcesWithoutMuscle();
	SysLEye->updateExtForce();
	SysLEye->computeAccelerationFeatherstone();

	for(int i = 3; i < SysLEye->dof(); i++) {
		GV_set(SysLEye->acc(),i,0.0);
		GV_set(SysLEye->vel(),i,0.0);
		GV_set(SysLEye->force(),i,0.0);
	}

	int repeat = (int)(displayTimeStep/simulationTimeStep);
	for(int i=0;i<repeat;++i){
		SysLEye->nextStepImplicitEuler1stOrderHighLEye(simulationTimeStep);
	}

	/*compute_pupil_size();
	SysLEye->adjust_pupil(pupil_size);
	compute_lens_size();
	SysLEye->adjust_cornea();*/

	// Everything below this just records data for plotting
	/*ofstream foutput("kerasOutputLeyeActivationsForPlotting.csv", ios::app);
	ofstream foutputori("outputOriLeye.csv", ios::app);
	ofstream foutputvel("outputVelLeye.csv", ios::app);
	ofstream foutputacc("outputAccLeye.csv", ios::app);

	static double prev_vel[2] ={0.0, 0.0};

	for(int i=0; i<num_eoc_muscles-1; i++){
		foutput << GV_get(SysLEye->actLevel(), i) << ",";
	}
	foutput << GV_get(SysLEye->actLevel(), num_eoc_muscles-1) << endl;
	foutput.flush();

	ori_leye = getcurrent_ori_leye();

	foutputori << ori_leye.x() << ",";
	foutputori << ori_leye.z() << "," << endl;
	foutputori.flush();

	gBallLink* l = (gBallLink*)(SysLEye->eyeball);

	double velx = l->genVel(0);
	double velz = l->genVel(2);

	foutputvel << velx << ",";
	foutputvel << velz << "," << endl;
	foutputvel.flush();

	double accx = velx - prev_vel[0];
	double accz = velz - prev_vel[1];

	foutputacc << accx << ",";
	foutputacc << accz << "," << endl;
	foutputacc.flush();

	prev_vel[0] = velx;
	prev_vel[1] = velz;*/
}

void create_bounce_board(void)
{
	visualBounceBoard1 = new osg::MatrixTransform;
	
	osg::Box* unitCube = new osg::Box(osg::Vec3(750,-920,300), 550,550,10);

	// Declare an instance of the shape drawable class and initialize 
	// it with the unitCube shape we created above.
	// This class is derived from 'drawable' so instances of this
	// class can be added to Geode instances.
	osg::ShapeDrawable* unitCubeDrawable = new osg::ShapeDrawable(unitCube);
	// Declare a instance of the geode class: 
	osg::Geode* basicShapesGeode = new osg::Geode();
	// Add the unit cube drawable to the geode:
	basicShapesGeode->addDrawable(unitCubeDrawable);
	visualBounceBoard1.get()->addChild(basicShapesGeode);

	if(sceneGroup) sceneGroup->addChild(visualBounceBoard1.get());

	osg::Matrixd rotMat;
	rotMat.makeRotate(0, osg::X_AXIS, -30*gDTR, osg::Y_AXIS, 0,osg::Z_AXIS);
	visualBounceBoard1->setMatrix(rotMat);

	visualBounceBoard2 = new osg::MatrixTransform;
	osg::Box* unitCube2 = new osg::Box(osg::Vec3(-600,-850, 5), 650,650,10);

	// Declare an instance of the shape drawable class and initialize 
	// it with the unitCube shape we created above.
	// This class is derived from 'drawable' so instances of this
	// class can be added to Geode instances.
	osg::ShapeDrawable* unitCubeDrawable2 = new osg::ShapeDrawable(unitCube2);
	// Declare a instance of the geode class: 
	osg::Geode* basicShapesGeode2 = new osg::Geode();
	// Add the unit cube drawable to the geode:
	basicShapesGeode2->addDrawable(unitCubeDrawable2);
	visualBounceBoard2.get()->addChild(basicShapesGeode2);

	if(sceneGroup) sceneGroup->addChild(visualBounceBoard2.get());

	rotMat.makeRotate(-20*gDTR, osg::X_AXIS, 25*gDTR, osg::Y_AXIS, 0,osg::Z_AXIS);
	visualBounceBoard2->setMatrix(rotMat);



	visualBounceBoard3 = new osg::MatrixTransform;
	osg::Box* unitCube3 = new osg::Box(osg::Vec3(0,-1650,-120), 550,550,10);
	osg::ShapeDrawable* unitCubeDrawable3 = new osg::ShapeDrawable(unitCube3);
	osg::Geode* basicShapesGeode3 = new osg::Geode();
	basicShapesGeode3->addDrawable(unitCubeDrawable3);
	visualBounceBoard3.get()->addChild(basicShapesGeode3);
	if(sceneGroup) sceneGroup->addChild(visualBounceBoard3.get());
	osg::Matrixd rotMat3;
	rotMat3.makeRotate(-30*gDTR, osg::X_AXIS, 20*gDTR, osg::Y_AXIS, 0,osg::Z_AXIS);
	visualBounceBoard3->setMatrix(rotMat3);



}

void create_visual_target(void)
{
	visualTarget = new osg::MatrixTransform;
	visualTarget.get()->setDataVariance(osg::Object::DYNAMIC);
	//use this if we want to have some other object other than sphere. we can import 3ds models maybe baseball
	//visualTarget.get()->addChild( osgDB::readNodeFile("prop/legoman2.3ds"));
	
#ifdef CONTROL_LEGS
	
	osg::Sphere* unitCube = new osg::Sphere( osg::Vec3(0,0,0), SPHERE_SIZE);
#else
	osg::Sphere* unitCube = new osg::Sphere( osg::Vec3(0,0,0), SPHERE_SIZE);
#endif
	// Declare an instance of the shape drawable class and initialize 
	// it with the unitCube shape we created above.
	// This class is derived from 'drawable' so instances of this
	// class can be added to Geode instances.
	osg::ShapeDrawable* unitCubeDrawable = new osg::ShapeDrawable(unitCube);
	// Declare a instance of the geode class: 
	osg::Geode* basicShapesGeode = new osg::Geode();
	// Add the unit cube drawable to the geode:
	basicShapesGeode->addDrawable(unitCubeDrawable);

#ifdef CONTROL_LEGS
	visualTarget.get()->addChild( osg::ref_ptr<osg::Node>(osgDB::readNodeFile("props\\Soccer\\soccer.3DS")));
#else
	
	//visualTarget.get()->addChild( osg::ref_ptr<osg::Node>(osgDB::readNodeFile("props\\Drone\\drone_blue.osg")));
	visualTarget.get()->addChild(basicShapesGeode);
#endif
	//gLink* base = SysNeck->root();
	//base = SysNeck->findLink("Skull"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Skull");
	gVec3 baseWorldPos =base->frame().trn();
	
	//initial position
	osg::Matrixd mat;

#ifdef CONTROL_LEGS
	
	mat.makeTranslate(baseWorldPos.x()-30,baseWorldPos.y()-650,baseWorldPos.z()-1500);
#else
	mat.makeTranslate(baseWorldPos.x()-30,baseWorldPos.y()-500,baseWorldPos.z()+50);
#endif
	visualTarget.get()->setMatrix(mat);


	if(sceneGroup) sceneGroup->addChild(visualTarget.get());

	//randomMotion.set_bbox(Vec3(-30,-5,25),Vec3(30,5,30));
	//randomMotion.set_bbox(Vec3(-7,-7,25),Vec3(7,7,30));
	randomMotion.set_bbox(osg::Vec3(baseWorldPos.x()+3,baseWorldPos.y()+4,baseWorldPos.z()-1),osg::Vec3(baseWorldPos.x()+3,baseWorldPos.y()-4,baseWorldPos.z()+1));
	//randomMotion.set_bbox(osg::Vec3(baseWorldPos.x()-300,baseWorldPos.y()-100,baseWorldPos.z()-150),osg::Vec3(baseWorldPos.x()+300,baseWorldPos.y()-450,baseWorldPos.z()+150));
	//randomMotion.set_bbox(osg::Vec3(baseWorldPos.x()+100,baseWorldPos.y()-100,baseWorldPos.z()-1150),osg::Vec3(baseWorldPos.x()+500,baseWorldPos.y()-250,baseWorldPos.z()-850));
#ifdef INSTANT_MODE
	randomMotion.set_mode(RandomMotion::INSTANT);
	randomMotion.set_rangeSpeed(20,50);
#else
	randomMotion.set_mode(RandomMotion::LINEAR);
	randomMotion.set_rangeSpeed(1,30);
#endif

	//randomMotion.set_mode(RandomMotion::LINEAR);
	randomMotion.set_noisy(true);
	randomMotion.set_rangeNoiseSpeed(0,0.4);
	randomMotion.set_noisyDistance(2);
	randomMotion.set_initialPosition(mat.getTrans());
}

void create_visual_target2(void)
{
	visualTarget = new osg::MatrixTransform;
	visualTarget.get()->setDataVariance(osg::Object::DYNAMIC);
#ifdef SECOND_VISUAL_TARGET
	visualTarget2 = new osg::MatrixTransform;
	visualTarget2.get()->setDataVariance(osg::Object::DYNAMIC);
#endif
	//use this if we want to have some other object other than sphere. we can import 3ds models maybe baseball
	//visualTarget.get()->addChild( osgDB::readNodeFile("prop/legoman2.3ds"));

#ifdef CONTROL_LEGS

	osg::Sphere* unitCube = new osg::Sphere( osg::Vec3(0,0,0), SPHERE_SIZE);
#else
	osg::Sphere* unitCube = new osg::Sphere( osg::Vec3(0,0,0), SPHERE_SIZE);
#endif
	// Declare an instance of the shape drawable class and initialize 
	// it with the unitCube shape we created above.
	// This class is derived from 'drawable' so instances of this
	// class can be added to Geode instances.
	osg::ShapeDrawable* unitCubeDrawable = new osg::ShapeDrawable(unitCube);
	// Declare a instance of the geode class: 
	osg::Geode* basicShapesGeode = new osg::Geode();
	// Add the unit cube drawable to the geode:
	basicShapesGeode->addDrawable(unitCubeDrawable);

	osg::ref_ptr<osg::Material> mater = new osg::Material;
	mater->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0, 0.0, 0.0, 1.0));
	mater->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0.0, 0.0, 0.0, 1.0));
	mater->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.0, 0.0, 0.0, 1.0));
	mater->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0, 0.0, 0.0, 1.0));
	basicShapesGeode->getOrCreateStateSet()->setAttributeAndModes(mater.get(), osg::StateAttribute::ON);

	/*vis_target_ec = 0.55;
	osg::ref_ptr<osg::Material> mater = new osg::Material;
	mater->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	mater->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	mater->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	mater->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	basicShapesGeode->getOrCreateStateSet()->setAttributeAndModes(mater.get(), osg::StateAttribute::ON);
	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));*/


#ifdef CONTROL_LEGS
	visualTarget.get()->addChild( osg::ref_ptr<osg::Node>(osgDB::readNodeFile("props\\Soccer\\soccer.3DS")));
#else
	visualTarget.get()->addChild(basicShapesGeode);

	#ifdef SECOND_VISUAL_TARGET
		visualTarget2.get()->addChild(basicShapesGeode);
	#endif
#endif

	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball");
	gVec3 baseWorldPos =base->frame().trn();
	
	osg::Matrixd mat;

#ifdef CONTROL_LEGS

	mat.makeTranslate(baseWorldPos.x()-30,baseWorldPos.y()-650,baseWorldPos.z()-1500);
#else
	//mat.makeTranslate(baseWorldPos.x(), baseWorldPos.y()+yplane, baseWorldPos.z()); // use this
	mat.makeTranslate(baseWorldPos.x(), baseWorldPos.y()-445, baseWorldPos.z()); // use this
#endif
	visualTarget.get()->setMatrix(mat);
	if(sceneGroup) {
		sceneGroup->addChild(visualTarget.get());
	}
#ifdef SECOND_VISUAL_TARGET
	visualTarget2.get()->setMatrix(mat);
	if(sceneGroup) {
		sceneGroup->addChild(visualTarget2.get());
	}
#endif

	//randomMotion.set_bbox(Vec3(-30,-5,25),Vec3(30,5,30));
	//randomMotion.set_bbox(Vec3(-7,-7,25),Vec3(7,7,30));
	randomMotion.set_bbox(osg::Vec3(baseWorldPos.x()+3,baseWorldPos.y()+4,baseWorldPos.z()-1),osg::Vec3(baseWorldPos.x()+3,baseWorldPos.y()-4,baseWorldPos.z()+1));
	randomMotion2.set_bbox(osg::Vec3(baseWorldPos.x()+3,baseWorldPos.y()+4,baseWorldPos.z()-1),osg::Vec3(baseWorldPos.x()+3,baseWorldPos.y()-4,baseWorldPos.z()+1));
#ifdef INSTANT_MODE
	randomMotion.set_mode(RandomMotion::INSTANT);
	randomMotion.set_rangeSpeed(20,50);
#else
	randomMotion.set_mode(RandomMotion::LINEAR);
	randomMotion.set_rangeSpeed(1,30);

	randomMotion2.set_mode(RandomMotion::LINEAR);
	randomMotion2.set_rangeSpeed(1,30);
#endif

	//randomMotion.set_mode(RandomMotion::LINEAR);
	randomMotion.set_noisy(true);
	randomMotion.set_rangeNoiseSpeed(0,0.4);
	randomMotion.set_noisyDistance(2);
	randomMotion.set_initialPosition(mat.getTrans());

	randomMotion2.set_noisy(true);
	randomMotion2.set_rangeNoiseSpeed(0,0.4);
	randomMotion2.set_noisyDistance(2);
	randomMotion2.set_initialPosition(mat.getTrans());
}

void create_eye_gaze_hitbox(void)
{
	visualEyeGazeHitbox = new osg::MatrixTransform;
	
	float xlen = 700.0;
	float ylen = 2.0;
	float zlen = 700.0;
	//osg::Box* hitcube = new osg::Box(osg::Vec3(35.0,-600.0, 1848.0), xlen, ylen, zlen);

	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	Vec3f_local minb(35-(xlen/2),baseWorldPos.y()+yplane+(ylen/2),1848-(zlen/2));
	Vec3f_local maxb(35+(xlen/2),baseWorldPos.y()+yplane-(ylen/2),1848+(zlen/2));

	eyeTraceHitBox = Box_local(minb,maxb);

	// Declare an instance of the shape drawable class and initialize 
	// it with the unitCube shape we created above.
	// This class is derived from 'drawable' so instances of this
	// class can be added to Geode instances.
	/*osg::ShapeDrawable* hitcubeDrawable = new osg::ShapeDrawable(hitcube);
	// Declare a instance of the geode class: 
	osg::Geode* basicShapesGeode = new osg::Geode();
	// Add the unit cube drawable to the geode:
	basicShapesGeode->addDrawable(hitcubeDrawable);

	osg::ref_ptr<osg::Material> mater = new osg::Material;
	mater->setAlpha(osg::Material::FRONT_AND_BACK, 0.0);
	basicShapesGeode->getOrCreateStateSet()->setAttributeAndModes(mater.get(), osg::StateAttribute::ON);

	visualEyeGazeHitbox.get()->addChild(basicShapesGeode);*/

	if(sceneGroup) sceneGroup->addChild(visualEyeGazeHitbox.get());
}

void create_eye_ball_left(void)
{
	leyeGroup = new osg::Group;
	/*gLink* base = SysNeck->findLink("Skull");
	eyeBallLeft = new osg::MatrixTransform;*/

#ifdef CONTROL_LEGS
			eye_cyl = 0;
			eye_cxl = 30*gDTR;
#else
			//eye_cyl = 45*gDTR;
			eye_cyl = 0*gDTR;
			eye_cxl = 0*gDTR;
#endif

	SysLEye = new HuMuLEye();
	SysMDLeye = dynamic_cast<gSystemMD*>(SysLEye);
	//SolverLeye = new gSystemSolver(SysLEye);
	VisLEye = new gVisSystemMD(SysLEye);
	VoluntaryLEye = new gSystemMDCtrlVol(SysLEye);
	ReflexLEye = new gSystemMDCtrlReflex(SysLEye,VoluntaryLEye);
	gSystemMDLoader loader4;
	loader4.load_model("./PAT_organ_leye.txt","./PAT_muscle_leye.txt",SysLEye,VisLEye);
	SysLEye->init_IK();
	SysLEye->init_eye_organ();
	SysLEye->init_muscleList();
	SysLEye->rearrange_muscleList();
	//SysLEye->init_muscle_weights("");
	//SysLEye->init_pcsa_weights("");
	SysLEye->init_jointMuscleMapForBoneM(1e-3);
	SysLEye->updateKVariablesUptoVel();
	//int muscleUb =  SysLEye->muscle_leye.size();
	int muscleUb = 6; // EOC muscles only
	int jointUb = SysLEye->jsz_leye;
	cout << muscleUb << "," << jointUb << endl;
	PF_ubLeye = ( muscleUb == 0 || jointUb == 0 ) ? NULL :GM_alloc(jointUb, muscleUb);
	VisLEye->update();
	compute_pupil_size();
	SysLEye->init_pupil(pupil_size, 13.3);
#ifdef UNIFORM_LENS
	SysLEye->init_lens(9.0, MIN_LENS_WIDTH, 10);
#else
	SysLEye->init_lens(6.5, 16.0, MIN_LENS_WIDTH, 10);
#endif
	SysLEye->init_cornea(8.5, 10.5, 10.2); // can change the geometry of the cornea here
	leyeGroup->addChild(VisLEye->node().get());
	leyeGroup->addChild(SysLEye->VisPupil.get());
	leyeGroup->addChild(SysLEye->VisLens.get());
	leyeGroup->addChild(SysLEye->VisCornea.get());
	leyeGroup->addChild(SysLEye->VisEyeball.get()); // for side view only
	//leyeGroup->addChild(eyeBallLeft.get());
	sceneGroup->addChild(leyeGroup);
	
	lens_arc_increment = SysLEye->lens.sphere2.radius * 0.5 * gDTR;

	//SysLEye->init_eye_legacy("props\\Eyeball\\eyeball3_small.osg", eyeBallLeft, base);
	SysLEye->updateKVariablesUptoAcc();
	VoluntaryLEye->init();
	ReflexLEye->init();

}


void re_create_eye_ball_left(void)
{
	
#ifdef CONTROL_LEGS
			eye_cyl = 0;
			eye_cxl = 30*gDTR;
#else
			//eye_cyl = 45*gDTR;
			eye_cyl = 0*gDTR;
			eye_cxl = 0*gDTR;
#endif


	delete SysLEye;
	delete VisLEye;
	delete VoluntaryLEye;
	delete ReflexLEye;

	SysLEye = new HuMuLEye();
	SysMDLeye = dynamic_cast<gSystemMD*>(SysLEye);
	//SolverLeye = new gSystemSolver(SysLEye);
	VisLEye = new gVisSystemMD(SysLEye);
	VoluntaryLEye = new gSystemMDCtrlVol(SysLEye);
	ReflexLEye = new gSystemMDCtrlReflex(SysLEye,VoluntaryLEye);
	gSystemMDLoader loader4;
	loader4.load_model("./PAT_organ_leye.txt","./PAT_muscle_leye.txt",SysLEye,VisLEye);
	SysLEye->init_IK();
	SysLEye->init_eye_organ();
	SysLEye->init_muscleList();
	SysLEye->rearrange_muscleList();
	//SysLEye->init_muscle_weights("");
	//SysLEye->init_pcsa_weights("");
	SysLEye->init_jointMuscleMapForBoneM(1e-3);
	SysLEye->updateKVariablesUptoVel();
	//int muscleUb =  SysLEye->muscle_leye.size();
	int muscleUb = 6; // EOC muscles only
	int jointUb = SysLEye->jsz_leye;
	cout << muscleUb << "," << jointUb << endl;
	PF_ubLeye = ( muscleUb == 0 || jointUb == 0 ) ? NULL :GM_alloc(jointUb, muscleUb);
	VisLEye->update();
	compute_pupil_size();
	SysLEye->init_pupil(pupil_size, 13.3);
#ifdef UNIFORM_LENS
	SysLEye->init_lens(9.0, MIN_LENS_WIDTH, 10);
#else
	SysLEye->init_lens(6.5, 16.0, MIN_LENS_WIDTH, 10);
#endif
	SysLEye->init_cornea(8.5, 10.5, 10.2); // can change the geometry of the cornea here
	
	lens_arc_increment = SysLEye->lens.sphere2.radius * 0.5 * gDTR;

	//SysLEye->init_eye_legacy("props\\Eyeball\\eyeball3_small.osg", eyeBallLeft, base);
	SysLEye->updateKVariablesUptoAcc();
	VoluntaryLEye->init();
	ReflexLEye->init();

}

void move_visual_target(osg::MatrixTransform* trans_visual_target, double time)
{
	osg::Matrixd mat;
	bool new_target = randomMotion.sol_pos();

	// set the doll's orientation as facing against the head
	osg::Vec3 v(randomMotion.get_curPos());
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);	
}

void move_draw(gVec3 prev, gVec3 after)
{
	osg::Vec3Array* lineVertices = new osg::Vec3Array;
	osg::Vec4Array* colors_line = new osg::Vec4Array;

	lineVertices->setName("VertexHitTraceLeft");

	gVec3 diff = after - prev;

	bool smallxzdiff = sqrt(diff.x()*diff.x()+diff.z()*diff.z()) < 4.0;

	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	osg::Vec3 prevosg = osg::Vec3(prev.x(),baseWorldPos.y()+yplane,prev.z());
	osg::Vec3 afterosg = osg::Vec3(after.x(),baseWorldPos.y()+yplane,after.z());

	osg::Vec3 afterosgcross11 = osg::Vec3(after.x()-1.0,baseWorldPos.y()+yplane,after.z()+1.0);
	osg::Vec3 afterosgcross12 = osg::Vec3(after.x()+1.0,baseWorldPos.y()+yplane,after.z()-1.0);

	osg::Vec3 afterosgcross21 = osg::Vec3(after.x()-1.0,baseWorldPos.y()+yplane,after.z()-1.0);
	osg::Vec3 afterosgcross22 = osg::Vec3(after.x()+1.0,baseWorldPos.y()+yplane,after.z()+1.0);

	lineVertices->push_back( prevosg );
	lineVertices->push_back( afterosg );

	lineVertices->push_back( afterosgcross11 );
	lineVertices->push_back( afterosgcross12 );

	lineVertices->push_back( afterosgcross21 );
	lineVertices->push_back( afterosgcross22 );

	if(exportMotion)
	{
		for(int i = 0; i < 6; i+=2) {
			osg::Vec3f startpoint = lineVertices->at(i);
			osg::Vec3f endpoint = lineVertices->at(i+1);
			fprintf(fileRaytracePoints,"%g %g %g %g %g %g\n", startpoint.x(), startpoint.y(), startpoint.z(), endpoint.x(), endpoint.y(), endpoint.z());
		}
	}
	
	colors_line->push_back(osg::Vec4(135/255.0,206/255.0,250/255.0,1));
	colors_line->push_back(osg::Vec4(135/255.0,206/255.0,250/255.0,1));

	colors_line->push_back(osg::Vec4(135/255.0,206/255.0,250/255.0,1));
	colors_line->push_back(osg::Vec4(135/255.0,206/255.0,250/255.0,1));

	colors_line->push_back(osg::Vec4(135/255.0,206/255.0,250/255.0,1));
	colors_line->push_back(osg::Vec4(135/255.0,206/255.0,250/255.0,1));
	
	osg::ref_ptr<osg::Geometry> geom_active = new osg::Geometry;
	geom_active->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, lineVertices->size()));
	geom_active->setUseDisplayList(false);

	geom_active->setVertexArray(lineVertices);
	geom_active->setColorArray(colors_line);
	geom_active->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	osg::ref_ptr<osg::Geode> geode_active = new osg::Geode;
	geode_active->addDrawable(geom_active.get());

	osg::StateSet* state_active = geode_active->getOrCreateStateSet();
	state_active->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	state_active->setMode(GL_BLEND, osg::StateAttribute::ON);
	state_active->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
	osg::LineWidth* lw_active = new osg::LineWidth;
	lw_active->setWidth(300);
	
	state_active->setAttribute(lw_active, osg::StateAttribute::ON);
	osg::BlendFunc* blendfunc_active = new osg::BlendFunc();
	state_active->setAttributeAndModes(blendfunc_active, osg::StateAttribute::ON);
	
	sceneGroup->addChild( geode_active );
}

void move_visual_target_eye_training(osg::MatrixTransform* trans_visual_target, double time)
{	
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();
	
	//randomMotion.set_bbox(osg::Vec3(baseWorldPos.x()-250,baseWorldPos.y()-4000,baseWorldPos.z()-250),osg::Vec3(baseWorldPos.x()+250,baseWorldPos.y()-400,baseWorldPos.z()+250));
	//randomMotion.set_bbox(osg::Vec3(baseWorldPos.x()-120,baseWorldPos.y()-500,baseWorldPos.z()-130),osg::Vec3(baseWorldPos.x()+120,baseWorldPos.y()-400,baseWorldPos.z()+130));
	randomMotion.set_bpyramid(osg::Vec3(baseWorldPos.x(),baseWorldPos.y()-250,baseWorldPos.z()),osg::Vec3(baseWorldPos.x()-300,baseWorldPos.y()-1200,baseWorldPos.z()-300)); // use this
	randomMotion.set_mode(RandomMotion::INSTANT);
	//randomMotion.set_rangeSpeed(1*100,1.1*100);
	//randomMotion.set_rangeSpeed(4,4.1); // USE OR REVERT TO THIS
	//randomMotion.set_rangeSpeed(5,5.1); // USE OR REVERT TO THIS
	randomMotion.set_rangeSpeed(1*50,1.1*50);

	osg::Matrixd mat;
	new_target_ball = randomMotion.sol_pos();
	//if(new_target_ball){
	//	//vis_target_ec = (0.99-0.11)*rand()/((float)RAND_MAX)+0.11;
	//	vis_target_ec = 0.44;
	//	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	//	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	//	trans_visual_target->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);
	//	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	//	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	//	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	//	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	//}
	// set the doll's orientation as facing against the head
	osg::Vec3 v(randomMotion.get_curPos());
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);	
}

void move_visual_target_saccade_fixed(osg::MatrixTransform* trans_visual_target, double time)
{	
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	/*vis_target_ec = 0.7;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);*/

	static int tot_count = 1;
	static int tot_count_vis = 1;
	float xbound = 120;
	float zbound = 130;

	// Target positions
	vector<osg::Vec3> targetPoints;
	vector<double> targetIntensities;

	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()+yplane, baseWorldPos.z()));
	targetIntensities.push_back(0.7);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x()-xbound/1.3, baseWorldPos.y()+yplane, baseWorldPos.z()+zbound/2));
	targetIntensities.push_back(0.7);
	//targetIntensities.push_back(0.4);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x()+2*xbound/3, baseWorldPos.y()+yplane, baseWorldPos.z()-2*zbound/3));
	targetIntensities.push_back(0.4);
	//targetIntensities.push_back(0.8);
	/*targetPoints.push_back(osg::Vec3(baseWorldPos.x()+xbound, baseWorldPos.y()+yplane, baseWorldPos.z()-2*zbound/3));
	targetIntensities.push_back(0.4);*/
	targetPoints.push_back(osg::Vec3(baseWorldPos.x()-xbound/3.1, baseWorldPos.y()+yplane, baseWorldPos.z()-zbound/3));
	//targetPoints.push_back(osg::Vec3(baseWorldPos.x()-xbound/1.7, baseWorldPos.y()-400, baseWorldPos.z()+zbound/2));
	//targetPoints.push_back(osg::Vec3(baseWorldPos.x()-xbound/1.1, baseWorldPos.y()-400, baseWorldPos.z()));
	//targetIntensities.push_back(0.8);
	targetIntensities.push_back(0.6);

	/*targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()+yplane, baseWorldPos.z()));
	targetIntensities.push_back(0.99);*/

	targetPoints.push_back(osg::Vec3(baseWorldPos.x()-xbound/7, baseWorldPos.y()+yplane, baseWorldPos.z()+zbound/2));
	targetIntensities.push_back(0.2);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x()+xbound/1.2, baseWorldPos.y()+yplane, baseWorldPos.z()+zbound/1.3));
	targetIntensities.push_back(0.5);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x()+xbound/1.2, baseWorldPos.y()+yplane, baseWorldPos.z()-zbound/1.3));
	targetIntensities.push_back(0.6);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()+yplane, baseWorldPos.z()+zbound/1.2));
	targetIntensities.push_back(0.4);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x()-xbound/1.2, baseWorldPos.y()+yplane, baseWorldPos.z()));
	targetIntensities.push_back(0.9);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()+yplane, baseWorldPos.z()-zbound/1.1));
	targetIntensities.push_back(0.5);

	static int targetIndex = 0;
	static int cnt = 0;

	if(cnt < tot_count){
		new_target_ball = false;
		cnt++;
	}
	else {
		targetIndex = min(targetIndex + 1, targetPoints.size() - 1);
		new_target_ball = true;
		if(tot_count == 1) {
			//tot_count *= 80;
			tot_count *= 40;
		}
		/*if(targetIndex == 3) {
			tot_count = 100;
		}
		else {
			tot_count = 40;
		}*/
		cnt = 0;
	}

	static int visTargetIndex = 0;
	static int cntVis = 0;

	if(cntVis < tot_count_vis){
		cntVis++;
	}
	else {
		visTargetIndex = (visTargetIndex + 1) % targetIntensities.size();
		if(tot_count_vis == 1) {
			tot_count_vis = 100;
		}
		else {
			tot_count_vis = 80;
		}
		cntVis = 0;
	}

	osg::Vec3 target = targetPoints.at(targetIndex);
	//vis_target_ec = targetIntensities.at(targetIndex);
	vis_target_ec = targetIntensities.at(visTargetIndex);

	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	trans_visual_target->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);
	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	
	osg::Matrixd mat;
	mat.setTrans(target);
	trans_visual_target->setMatrix(mat);	
}

void move_visual_target_smooth_pursuit(osg::MatrixTransform* trans_visual_target, double time)
{
	/*vis_target_ec = 1.0;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);*/

	//vis_target_ec = 0.7;
	osg::Vec3 one_step;
	//static double one_step_int;
	static int tot_count = 30;
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	osg::Vec3 startPos(baseWorldPos.x(), baseWorldPos.y()+yplane, baseWorldPos.z());
	static osg::Vec3 cur_pos = startPos;

	float xbound = 120;
	float zbound = 130;

	// Target positions
	vector<osg::Vec3> targetPoints;
	vector<double> targetIntensities;

	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()+yplane, baseWorldPos.z()));
	targetIntensities.push_back(0.7);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x()+xbound, baseWorldPos.y()+yplane, baseWorldPos.z()-zbound));
	targetIntensities.push_back(0.4);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x()+xbound, baseWorldPos.y()+yplane, baseWorldPos.z()+zbound));
	targetIntensities.push_back(0.9);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x()-xbound, baseWorldPos.y()+yplane, baseWorldPos.z()+zbound));
	targetIntensities.push_back(0.3);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x()+xbound, baseWorldPos.y()+yplane, baseWorldPos.z()-zbound));
	targetIntensities.push_back(0.8);

	static int targetIndex = 0;
	static int cnt = 0;

	static double one_step_int = 0.7 / tot_count;
	
	if(cnt < tot_count){
		cnt++;
	}
	else {
		//osg::Vec3 oldTarget = targetPoints.at(targetIndex);
		double oldTargetIntensity = targetIntensities.at(targetIndex);
		targetIndex = min(targetIndex + 1, targetPoints.size() - 1);
		//osg::Vec3 newTarget = targetPoints.at(targetIndex);
		double newTargetIntensity = targetIntensities.at(targetIndex);
		if(tot_count == 30) {
			tot_count *= 3;
		}
		cnt = 0;
		cout << "intensities: " << oldTargetIntensity << " " << newTargetIntensity << " " << newTargetIntensity-oldTargetIntensity << endl;
		//one_step = (newTarget-oldTarget) / tot_count;
		one_step_int = (newTargetIntensity-oldTargetIntensity) / tot_count;
	}

	cout << "vis target ec: " << vis_target_ec << endl;

	osg::Vec3 target = targetPoints.at(targetIndex);
	double targetIntensity = targetIntensities.at(targetIndex);
	one_step = target-cur_pos;
	one_step /= tot_count;
	//one_step_int = targetIntensity-vis_target_ec;
	//one_step_int /= tot_count;
	cur_pos += one_step;
	vis_target_ec = (one_step_int > 0) ? min(targetIntensity, vis_target_ec+one_step_int) : max(targetIntensity, vis_target_ec+one_step_int);

	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	trans_visual_target->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);
	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	
	osg::Matrixd mat;
	mat.setTrans(cur_pos);
	trans_visual_target->setMatrix(mat);

}

void move_visual_target_smooth_lens_adjustment(osg::MatrixTransform* trans_visual_target, double time)
{
	/*vis_target_ec = 1.0;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);*/

	//vis_target_ec = 0.7;
	osg::Vec3 one_step;
	//static double one_step_int;
	static int tot_count = 50;
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	osg::Vec3 startPos(baseWorldPos.x(), baseWorldPos.y()-350, baseWorldPos.z());
	static osg::Vec3 cur_pos = startPos;

	// Target positions
	vector<osg::Vec3> targetPoints;
	vector<double> targetIntensities;

	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-350, baseWorldPos.z()));
	targetIntensities.push_back(0.6);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-650, baseWorldPos.z()));
	targetIntensities.push_back(0.6);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-900, baseWorldPos.z()));
	targetIntensities.push_back(0.6);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-500, baseWorldPos.z()));
	targetIntensities.push_back(0.6);
	/*targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()+yplane, baseWorldPos.z()));
	targetIntensities.push_back(0.8);*/

	static int targetIndex = 0;
	static int cnt = 0;

	static double one_step_int = 0.6 / tot_count;
	
	if(cnt < tot_count){
		cnt++;
	}
	else {
		//osg::Vec3 oldTarget = targetPoints.at(targetIndex);
		double oldTargetIntensity = targetIntensities.at(targetIndex);
		targetIndex = min(targetIndex + 1, targetPoints.size() - 1);
		//osg::Vec3 newTarget = targetPoints.at(targetIndex);
		double newTargetIntensity = targetIntensities.at(targetIndex);
		if(tot_count == 50) {
			tot_count += 2;
		}
		cnt = 0;
		cout << "intensities: " << oldTargetIntensity << " " << newTargetIntensity << " " << newTargetIntensity-oldTargetIntensity << endl;
		//one_step = (newTarget-oldTarget) / tot_count;
		one_step_int = (newTargetIntensity-oldTargetIntensity) / tot_count;
	}

	cout << "vis target ec: " << vis_target_ec << endl;
	if(tot_count == 50) {
		vis_target_ec = 0.6;
	}

	osg::Vec3 target = targetPoints.at(targetIndex);
	double targetIntensity = targetIntensities.at(targetIndex);
	one_step = target-cur_pos;
	one_step /= tot_count;
	//one_step_int = targetIntensity-vis_target_ec;
	//one_step_int /= tot_count;
	cur_pos += one_step;
	vis_target_ec = (one_step_int > 0) ? min(targetIntensity, vis_target_ec+one_step_int) : max(targetIntensity, vis_target_ec+one_step_int);

	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	trans_visual_target->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);
	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	
	osg::Matrixd mat;
	mat.setTrans(cur_pos);
	trans_visual_target->setMatrix(mat);

}

void move_visual_target_smooth_pursuit_lateral(osg::MatrixTransform* trans_visual_target, double time)
{
	vis_target_ec = 0.6;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);
	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));

	osg::Vec3 one_step;
	static int counter = 0;
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	osg::Vec3 startPos(baseWorldPos.x(), baseWorldPos.y()+yplane, baseWorldPos.z());
	static osg::Vec3 cur_pos = startPos;

	float xval = 60*sin(counter*gDTR);
	counter++;

	osg::Vec3 target = osg::Vec3(baseWorldPos.x()+xval, baseWorldPos.y()+yplane, baseWorldPos.z());
	one_step = target-cur_pos;
	//one_step /= tot_count;
	cur_pos += one_step;
	
	osg::Matrixd mat;
	mat.setTrans(cur_pos);
	trans_visual_target->setMatrix(mat);

}

void move_visual_target_oscillate_straight(osg::MatrixTransform* trans_visual_target, double time)
{
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	//vis_target_ec = 0.99;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

	double vis_target_ec2 = 0.99;
	osg::ref_ptr<osg::Material> mater22 = new osg::Material;
	mater22->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec2, vis_target_ec2, vis_target_ec2, 1.0));
	
#ifdef SECOND_VISUAL_TARGET
	visualTarget2.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater22.get(), osg::StateAttribute::ON);
#endif

#ifdef LENS_VIDEO
	sceneView->setClearColor(osg::Vec4(0.05, 0.05, 0.05, 1.0));
#else
	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
#endif

	osg::Vec3 one_step, one_step2;
	static int counter = 0;

	osg::Vec3 startPos(baseWorldPos.x(), baseWorldPos.y()-400, baseWorldPos.z());
	static osg::Vec3 cur_pos = startPos;
	static osg::Vec3 cur_pos2 = startPos;

	float yval = 200*sin(3*counter*gDTR);
	counter++;

	osg::Vec3 targetLoc(baseWorldPos.x(), baseWorldPos.y()-400+yval, baseWorldPos.z());

	osg::Vec3 target = osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-400+yval, baseWorldPos.z());
	osg::Vec3 target2 = osg::Vec3(baseWorldPos.x()-30, baseWorldPos.y()-400-yval, baseWorldPos.z());
	one_step = target-cur_pos;
	one_step2 = target2-cur_pos2;
	//one_step /= tot_count;
	cur_pos += one_step;
	cur_pos2 += one_step2;
	
	osg::Matrixd mat, mat2;
	osg::Quat q(atan2(cur_pos.x(),cur_pos.z()),osg::Y_AXIS, -atan2(cur_pos.y()-8,sqrt(cur_pos.x()*cur_pos.x()+cur_pos.z()*cur_pos.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat2.makeRotate(q);
	mat.setTrans(cur_pos);
	trans_visual_target->setMatrix(mat);

	osg::Quat q2(atan2(cur_pos2.x(),cur_pos2.z()),osg::Y_AXIS, -atan2(cur_pos2.y()-8,sqrt(cur_pos2.x()*cur_pos2.x()+cur_pos2.z()*cur_pos2.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat2.makeRotate(q2);
	mat2.setTrans(cur_pos2);

	
#ifdef SECOND_VISUAL_TARGET
	visualTarget2.get()->setMatrix(mat2);
#endif
}

bool move_visual_target_ball_thrown(osg::MatrixTransform* trans_visual_target, double time)
{
	bool bHit = false;
	osg::Matrixd mat;
	static int counter = 1;
	static int changeVal = 50;
	static bool firstTime = true;
	static int eclevel = 0;
	bool random = true;
	//bool random = false;
	osg::Vec3 pos;
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball");
	gVec3 baseWorldPos =base->frame().trn();

	//pos.set(baseWorldPos.x()+120, baseWorldPos.y()-500, baseWorldPos.z()-180);
	pos.set(baseWorldPos.x(), baseWorldPos.y()-500, baseWorldPos.z()); // use this
	//pos.set(baseWorldPos.x(), baseWorldPos.y()-100, baseWorldPos.z()); // use this
	new_target_ball = false;
	//pos.set(baseWorldPos.x()-150, baseWorldPos.y()-500, baseWorldPos.z()+180);

	double* ecsweep = new double[4];
	/*ecsweep[0] = 0.5;
	ecsweep[1] = 0.11;
	ecsweep[2] = 0.6;
	ecsweep[3] = 0.99;*/

	ecsweep[0] = 0.99;
	ecsweep[1] = 0.99;
	ecsweep[2] = 0.99;
	ecsweep[3] = 0.99;

	if (firstTime || counter >= changeVal)
	{
		if (random) { // switch color randomly
			int randVal = rand();
			vis_target_ec = max(0.1,double(randVal) / double (RAND_MAX) + 1.0 / double (RAND_MAX));
		}
		else { // vary in controlled manner
			vis_target_ec = ecsweep[eclevel];
			eclevel = (eclevel + 1) % 4;
		}
		firstTime = false;
		osg::ref_ptr<osg::Material> mater2 = new osg::Material;
		mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);
		counter = 0;

		/*sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
		sceneView->getLight()->setDiffuse(osg::Vec4(0.6*vis_target_ec,0.6*vis_target_ec,0.6*vis_target_ec,1.0));
		sceneView->getLight()->setSpecular(osg::Vec4(0.5*vis_target_ec,0.5*vis_target_ec,0.5*vis_target_ec,1.0));
		sceneView->getLight()->setAmbient(osg::Vec4(0.5*vis_target_ec,0.5*vis_target_ec,0.5*vis_target_ec,1.0));*/

		sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
		sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
		sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
		sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	}
	
	counter++;

	// set the doll's orientation as facing against the head
	osg::Vec3 v(pos);
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);	
	return bHit;
}

void move_visual_target_ball_straight(osg::MatrixTransform* trans_visual_target, double time)
{	
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	vis_target_ec = 0.99;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));

	static int tot_count = 1;

	// Target positions
	vector<osg::Vec3> targetPoints;

	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y(), baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-90.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-800.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-200.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-2000.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-300.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-350.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-400.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-450.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-500.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-550.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-600.0, baseWorldPos.z()));

	static int targetIndex = 0;
	static int cnt = 0;
	static int current_index = 0;

	if(cnt < tot_count){
		new_target_ball = false;
		cnt++;
	}
	else {
		targetIndex = min(targetIndex + 1, targetPoints.size() - 1);
		new_target_ball = true;
		if(tot_count == 1) {
			tot_count *= 60;
		}
		cnt = 0;
	}

	osg::Vec3 target = targetPoints.at(targetIndex);
	
	osg::Matrixd mat;
	osg::Vec3 v(target);
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);		
}

void move_visual_target_pupil_training(osg::MatrixTransform* trans_visual_target, double time)
{	
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	randomMotion.set_bpyramid(osg::Vec3(baseWorldPos.x(),baseWorldPos.y()-350,baseWorldPos.z()),osg::Vec3(baseWorldPos.x()-200,baseWorldPos.y()-1250,baseWorldPos.z()-200));
	randomMotion2.set_bpyramid(osg::Vec3(baseWorldPos.x(),baseWorldPos.y()-350,baseWorldPos.z()),osg::Vec3(baseWorldPos.x()-200,baseWorldPos.y()-1250,baseWorldPos.z()-200));

	randomMotion.set_mode(RandomMotion::INSTANT);
	randomMotion2.set_mode(RandomMotion::INSTANT);
	//randomMotion.set_rangeSpeed(10,15); // CONVERT BACK TO THIS FOR PUPIL

	randomMotion.set_rangeSpeed(25,30);
	randomMotion2.set_rangeSpeed(25,30);

	//randomMotion.set_rangeSpeed(1*100,1.1*100); // USE OR REVERT TO THIS
	//randomMotion.set_rangeSpeed(3.3*100,3.5*100);

	osg::Matrixd mat;
	osg::Matrixd mat2;

#ifndef RECORD_PUPIL
	new_target_ball = randomMotion.sol_pos();
	//randomMotion2.sol_pos();
#else
	new_target_ball = randomMotion.sol_pos_on_cue(pupil_adjusted);
	if(pupil_adjusted) {
		pupil_adjusted = false;
	}
#endif

	if(new_target_ball){
		int randVal = rand();
		vis_target_ec = (0.99-0.11)*rand()/((float)RAND_MAX)+0.11;
		//vis_target_ec = (vis_target_ec == 0.99) ? 0.11 : 0.99;
		osg::ref_ptr<osg::Material> mater2 = new osg::Material;
		mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

		//double vis_target_ec2 = (0.99-0.11)*rand()/((float)RAND_MAX)+0.11;
		
#ifdef SECOND_VISUAL_TARGET
		double vis_target_ec2 = 0.99;
		osg::ref_ptr<osg::Material> mater22 = new osg::Material;
		mater22->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec2, vis_target_ec2, vis_target_ec2, 1.0));
		
		visualTarget2.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater22.get(), osg::StateAttribute::ON);
#endif
		sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
		sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
		sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
		sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	}
	
	//osg::Vec3 v(randomMotion.get_curPos());
	osg::Vec3 v(baseWorldPos.x()-100, baseWorldPos.y()-600, baseWorldPos.z()+160);
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);	
	
#ifdef SECOND_VISUAL_TARGET
	//osg::Vec3 v2(randomMotion2.get_curPos());
	osg::Vec3 v2(baseWorldPos.x()-15, baseWorldPos.y()-200, baseWorldPos.z());
	osg::Quat q2(atan2(v2.x(),v2.z()),osg::Y_AXIS, -atan2(v2.y()-8,sqrt(v2.x()*v2.x()+v2.z()*v2.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat2.makeRotate(q2);
	mat2.setTrans(v2);
	
	visualTarget2.get()->setMatrix(mat2);	
#endif
}

double lens_ec, lens_distance;
void move_visual_target_lens_training(osg::MatrixTransform* trans_visual_target, double time)
{	
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	static int cnt = 0;
	//int tot_count = 90;
	//int tot_count = 45;
	int tot_count = 30;
	static double priorSelection = 0.0;

	static bool bInitialized = false;
	if(!bInitialized)
	{
		new_target_ball = true;
		//vis_target_ec = randomNumber(0.11,0.99);
		//vis_target_ec = 0.16;
		vis_target_ec = 0.6;
		//lens_distance = randomNumber(500,750);
		//lens_distance = randomNumber(150,1250);
		lens_distance = randomNumber(350,1250);
		//lens_distance = 450;
		//lens_distance = randomNumber(150,250);
		priorSelection = lens_distance;
		bInitialized = true;
	}

#ifndef RECORD_LENS
	if(cnt < tot_count){
		cnt++;
		new_target_ball = false;
	}
	else {
		new_target_ball = true;
		//int randVal = rand();
		//vis_target_ec = randomNumber(0.11,0.99);
		vis_target_ec = 0.6;
		//lens_ec = randomNumber(0.11,0.99);
		//lens_distance = randomNumber(500,750);
		//lens_distance = randomNumber(150,950);
		//lens_distance = (priorSelection <= 550) ? randomNumber(650,950) : randomNumber(150,400);
		lens_distance = (priorSelection <= 800) ? randomNumber(800,1250) : randomNumber(350,800);
		priorSelection = lens_distance;
		//ec = 0.55;
		//distance = 445;
		//cnt = randomNumber(0,45);
		cnt = randomNumber(0,10);
	}
#else

	if(lens_adjusted) {
		new_target_ball = true;
		//vis_target_ec = randomNumber(0.11,0.99);
		vis_target_ec = 0.6;
		//lens_distance = randomNumber(500,750);
		//lens_distance = randomNumber(150,950);
		//lens_distance = (priorSelection <= 1045) ? randomNumber(1045,2000) : randomNumber(90,1045);
		//lens_distance = (priorSelection <= 550) ? randomNumber(650,1250) : randomNumber(150,400);
		lens_distance = (priorSelection <= 800) ? randomNumber(800,1250) : randomNumber(350,800);
		priorSelection = lens_distance;
		lens_adjusted = false;
	}
	else {
		new_target_ball = false;
	}
#endif

	//cout << "count, ec, distance: " << cnt << " " << ec << " " << distance << endl;

	//vis_target_ec = lens_ec;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	trans_visual_target->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);
	
#ifdef LENS_VIDEO
	sceneView->setClearColor(osg::Vec4(0.05, 0.05, 0.05, 1.0));
#else
	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
#endif

	osg::Vec3 target(baseWorldPos.x(), baseWorldPos.y()-lens_distance, baseWorldPos.z());
	//osg::Vec3 target(baseWorldPos.x()+randomRange(-10,10), baseWorldPos.y()-lens_distance, baseWorldPos.z()+randomRange(-10,10));
	//osg::Vec3 target(baseWorldPos.x(), -900.32, baseWorldPos.z());

	osg::Matrixd mat;
	osg::Vec3 v(target);
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);	
}

void move_visual_target_lens_online_fixed_positions(osg::MatrixTransform* trans_visual_target, double time)
{	
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	static int cnt = 0;
	int tot_count = 50;

	vector<osg::Vec3> targetPoints;
	vector<double> targetIntensity;

	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-350, baseWorldPos.z())); // keep
	targetIntensity.push_back(0.7);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-674, baseWorldPos.z())); // keep
	targetIntensity.push_back(0.8);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-800, baseWorldPos.z())); // throw
	targetIntensity.push_back(0.6);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-890, baseWorldPos.z())); // throw
	targetIntensity.push_back(0.65);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()-500, baseWorldPos.z())); // throw
	targetIntensity.push_back(0.75);

	static int targetIndex = 0;
	static int current_index = 0;

#ifdef LENS_VIDEO
		/*if(targetIndex == 3 || targetPoints.size() == 1) {
			holdBallPosition = true;
		}*/
	if(holdBallPosition) {
		targetIndex = 3;
	}
	else if(cnt < tot_count){

#else
	if(cnt < tot_count){
#endif


	//if(cnt < tot_count){
		new_target_ball = false;
		cnt++;
	}
	else {
		//targetIndex = min(targetIndex + 1, targetPoints.size() - 1);

		if(!holdBallPosition || targetMoveOverride) {
			targetIndex = (targetIndex + 1) % (targetPoints.size() - 1);
		}
		new_target_ball = true;
		if(tot_count == 1) {
			tot_count *= 45;
		}
		cnt = 0;
	}

	osg::Vec3 target = targetPoints.at(targetIndex);
	vis_target_ec = targetIntensity.at(targetIndex);

	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	trans_visual_target->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);
		
#ifdef LENS_VIDEO
	sceneView->setClearColor(osg::Vec4(0.05, 0.05, 0.05, 1.0));
#else
	sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
#endif

	osg::Matrixd mat;
	osg::Vec3 v(target);
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);	
}

void move_visual_target_foveation_training(osg::MatrixTransform* trans_visual_target, double time)
{	
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	randomMotion.set_bpyramid(osg::Vec3(baseWorldPos.x(),baseWorldPos.y()-350,baseWorldPos.z()),osg::Vec3(baseWorldPos.x()-500,baseWorldPos.y()-1250,baseWorldPos.z()-500));
	randomMotion.set_mode(RandomMotion::INSTANT);
	randomMotion.set_rangeSpeed(1,1);
	//randomMotion.set_rangeSpeed(5,5);

	osg::Matrixd mat;
	new_target_ball = randomMotion.sol_pos();

	if(new_target_ball){
		int randVal = rand();
		vis_target_ec = (0.99-0.11)*rand()/((float)RAND_MAX)+0.11;
		osg::ref_ptr<osg::Material> mater2 = new osg::Material;
		mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

		sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
		sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
		sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
		sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	}
	
	osg::Vec3 v(randomMotion.get_curPos());
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);
}

void move_visual_target_lens_training_new(osg::MatrixTransform* trans_visual_target, double time)
{	
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos =base->frame().trn();

	randomMotion.set_bpyramid(osg::Vec3(baseWorldPos.x(),baseWorldPos.y()-350,baseWorldPos.z()),osg::Vec3(baseWorldPos.x(),baseWorldPos.y()-1250,baseWorldPos.z()));
	randomMotion.set_mode(RandomMotion::INSTANT);
	randomMotion.set_rangeSpeed(1,1);

	osg::Matrixd mat;
	new_target_ball = randomMotion.sol_pos();

	if(new_target_ball){
		int randVal = rand();
		vis_target_ec = (0.99-0.45)*rand()/((float)RAND_MAX)+0.45;
		osg::ref_ptr<osg::Material> mater2 = new osg::Material;
		mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

		sceneView->setClearColor(osg::Vec4(0.4*vis_target_ec,0.4*vis_target_ec,0.4*vis_target_ec,1.0));
		sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
		sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
		sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec,vis_target_ec,vis_target_ec,1.0));
	}
	
	osg::Vec3 v(randomMotion.get_curPos());
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);
}

bool move_visual_target_fixed_position(osg::MatrixTransform* trans_visual_target, double time)
{
	bool bHit = false;
	osg::Matrixd mat;
	static int counter = 1;
	static int changeVal = 120;
	static bool firstTime = true;
	static int eclevel = 0;
	//bool random = true;
	bool random = false;
	osg::Vec3 pos;
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball");
	gVec3 baseWorldPos =base->frame().trn();

	if(counter < 2) {
		pos.set(baseWorldPos.x(), baseWorldPos.y()-500, baseWorldPos.z());
	}
	else {
		//pos.set(baseWorldPos.x()+120, baseWorldPos.y()-500, baseWorldPos.z()-100);
		pos.set(baseWorldPos.x()-150.0, baseWorldPos.y()-500, baseWorldPos.z()+40);
		//pos.set(baseWorldPos.x(), baseWorldPos.y()-500, baseWorldPos.z());
	}

	new_target_ball = false;
	counter++;

	vis_target_ec = 1.0;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

	// set the doll's orientation as facing against the head
	osg::Vec3 v(pos);
	osg::Quat q(atan2(v.x(),v.z()),osg::Y_AXIS, -atan2(v.y()-8,sqrt(v.x()*v.x()+v.z()*v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);	
	return bHit;
}

void readStdMeanLeft()
{
	int p=0;
	cout << "reading mean and std"<<endl;
#ifdef OLD_RETINA
	ifstream ifs("meanEyeleft.csv");
#else
	ifstream ifs("meanFoveation.csv");
#endif
	if(!ifs){
		cout<<"入力エラー";
		return;
	}
	string str;
	while(getline(ifs,str)){
		string token;
		istringstream stream(str);
		while(getline(stream,token,',')){
			float temp=stof(token);
			meanValueLeft[p] = temp;
			p++;
		}
	}

	p=0;
#ifdef OLD_RETINA
	ifstream ifs2("stdEyeleft.csv");
#else
	ifstream ifs2("stdFoveation.csv");
#endif
	if(!ifs2){
		cout<<"入力エラー";
		return;
	}
	while(getline(ifs2,str)){
		string token;
		istringstream stream(str);
		while(getline(stream,token,',')){
			float temp=stof(token);
			stdValueLeft[p] = temp;
			p++;
		}
	}
}

void readStdMeanRight()
{
	int p=0;
	cout << "reading mean and std"<<endl;
	ifstream ifs("meanEyeright.csv");
	if(!ifs){
		cout<<"入力エラー";
		return;
	}
	string str;
	while(getline(ifs,str)){
		string token;
		istringstream stream(str);
		while(getline(stream,token,',')){
			float temp=stof(token);
			meanValueRight[p] = temp;
			p++;
		}
	}

	p=0;

	ifstream ifs2("strEyeright.csv");
	if(!ifs2){
		cout<<"入力エラー";
		return;
	}
	while(getline(ifs2,str)){
		string token;
		istringstream stream(str);
		while(getline(stream,token,',')){
			float temp=stof(token);
			stdValueRight[p] = temp;
			p++;
		}
	}
}

void readStdMeanPupil()
{
	int p=0;
	cout << "reading mean and std pupil"<<endl;
	ifstream ifs("meanPupil.csv");
	if(!ifs){
		cout<<"入力エラー";
		return;
	}
	string str;
	while(getline(ifs,str)){
		string token;
		istringstream stream(str);
		while(getline(stream,token,',')){
			float temp=stof(token);
			meanValuePupil[p] = temp;
			p++;
		}
	}

	p=0;
	ifstream ifs2("stdPupil.csv");
	if(!ifs2){
		cout<<"入力エラー";
		return;
	}
	while(getline(ifs2,str)){
		string token;
		istringstream stream(str);
		while(getline(stream,token,',')){
			float temp=stof(token);
			stdValuePupil[p] = temp;
			p++;
		}
	}
}

void readStdMeanLens()
{
	int p=0;
	cout << "reading mean and std lens"<<endl;
	ifstream ifs("meanLens.csv"); // physical
	if(!ifs){
		cout<<"入力エラー";
		return;
	}
	string str;
	while(getline(ifs,str)){
		string token;
		istringstream stream(str);
		while(getline(stream,token,',')){
			float temp=stof(token);
			meanValueLens[p] = temp;
			p++;
		}
	}

	p=0;
	ifstream ifs2("stdLens.csv"); // physical
	if(!ifs2){
		cout<<"入力エラー";
		return;
	}
	while(getline(ifs2,str)){
		string token;
		istringstream stream(str);
		while(getline(stream,token,',')){
			float temp=stof(token);
			stdValueLens[p] = temp;
			p++;
		}
	}
}

int main( int argc, char **argv )
{	
	unsigned int i;
	int cnt=0;

	/*
	scanpath = new float*[122743];
	for(int i = 0; i < 122743; ++i)
	*/

	scanpath = new float*[13];
	for(int i = 0; i < 13; ++i)
		scanpath[i] = new float[4];

	ifstream ifs("scanepath3.csv", ios::in);
	if(!ifs){
		cout << "Could not open delta file. Exiting" << endl;
		return -1;
	}

	string str;
	int p = 0;
	int q = 0;
	while(getline(ifs,str)){
		string token;
		istringstream stream(str);
		while(getline(stream,token,',')){
			float temp=stof(token);
			scanpath[q][p] = temp;
			p++;
			if(p==4)
				break;
		}
		p=0;
		q++;
	}

	leye_IK_solved = true;
	leye_muscle_solved = true;
	start_recording_pupil = false;
	start_recording_lens = false;
	start_recording_foveation = false;
	firstTimeHit = true;
	stored_focal_point = Vec3f_local(0);
	record_lens_data = true;

#ifdef LENS_VIDEO
	show_forward_trace_rays = false;
	//show_forward_trace_rays = true;
	show_reverse_trace_rays = false;
	show_full_ball_trace = false;
	full_ball_alpha_channel = 0.0;
	forward_rays_alpha_channel = 0.0;
	//forward_rays_alpha_channel = 1.0;
	reverse_rays_alpha_channel = 0.0;
	targetMoveOverride = false;
	oscillateLensCilliaryActivation = false;
	lensMovementDemoComplete = false;
	oscillatePupilSphincterActivation = false;
	pupilMovementDemoComplete = false;
	photoreceptor_to_trace_rho = 0;
	photoreceptor_to_trace_theta = 0;
	//holdBallPosition = false;
	holdBallPosition = true;
#else
	show_forward_trace_rays = true;
	show_reverse_trace_rays = false;
	show_full_ball_trace = false;
	full_ball_alpha_channel = 1.0;
	forward_rays_alpha_channel = 1.0;
	reverse_rays_alpha_channel = 1.0;
	photoreceptor_to_trace_rho = 0;
	photoreceptor_to_trace_theta = 0;
#endif
	
	int j;
	if(retina_distribution_noise) delete[] retina_distribution_noise;
	retina_distribution_noise = new gReal[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];

	char buf[128];

#ifdef OLD_RETINA
	FILE* fp = fopen("retina_distribution_noise.txt","r");
#else
	FILE* fp = fopen("denser_retina_distribution_noise.txt","r");
#endif

	if(!fp)
	{
		for(j=0;j<RAYTRACE_WIDTH*RAYTRACE_HEIGHT;j++)	retina_distribution_noise[j] = 0;
	}
	else
	{
		for(j=0;j<RAYTRACE_WIDTH*RAYTRACE_HEIGHT;j++){			
			fscanf(fp,"%s",buf);
			retina_distribution_noise[j] = atof(buf);
		}
	}
	
#ifdef ONLINE_FOVEATION
#ifdef OLD_RETINA
	mRayKeras = new keras::KerasModel("dumped_raytrace_left.nnet", true);
#else
	mRayKeras = new keras::KerasModel("dumped_foveation.nnet", true);
#endif
#endif
#ifdef ONLINE_FOVEATION
	sampleRayKeras = new keras::DataChunkFlat();
	readStdMeanLeft();
#endif

#ifdef ONLINE_PUPIL
	mPupilKeras = new keras::KerasModel("dumped_pupil.nnet", true);
	samplePupilKeras = new keras::DataChunkFlat();
	readStdMeanPupil();
#endif

#ifdef ONLINE_LENS
	mLensKeras = new keras::KerasModel("dumped_lens.nnet", true);
	sampleLensKeras = new keras::DataChunkFlat();
	readStdMeanLens();
#endif

#ifdef ONLINE_MOTOR
	readStdMeanLEye();
	//mLEye = new KerasModel("dumped_eye.nnet", true);
	mLEye = new KerasModel("dumped_eye_test.nnet", true);
	sampleLEye = new DataChunkFlat();
#endif

	/*
	init_glutLeft();
	init_osgLeft();
	
	init_glut_PR();

#if defined(TEST_PUPIL) || defined(TEST_LENS) || defined(TEST_FOVEATION)
	init_glut_PRTest();
#endif

	init_osg_PR();
	*/
	
	active_idx_pupil.push_back(1000000);
	active_idx_color_pupil.push_back(Vec3f_local(1));

	active_idx_lens.push_back(1000000);
	active_idx_color_lens.push_back(Vec3f_local(1));

	active_idx_foveation.push_back(1000000);
	active_idx_color_foveation.push_back(Vec3f_local(1));

	initialConfig();

	//int i;
	i=0;
	time_t t1;
	time(&t1);
	srand(t1);

	SYSTEM_INFO SystemInfo;
	GetSystemInfo(&SystemInfo);
	NumThreads = SystemInfo.dwNumberOfProcessors; 

	//SNOPT init
	integer iPrint = 0; //9 will creat "fort.9" file. 0 for no print (to file)
	integer iSumm = 0;//6;  //6. 0 for no summary (to screen)
	snCw = new char[snLenCw];
	snIw = new integer[snLenIw];
	snRw = new doublereal[snLenRw];
	sninit_(&iPrint,&iSumm,snCw,&snLenCw,snIw,&snLenIw,snRw,&snLenRw,0);
	integer maxIter = 10000;
	integer inform = 0;
	snseti_("Major iterations limit", &maxIter, &iPrint, &iSumm, &inform, snCw,&snLenCw,snIw,&snLenIw,snRw,&snLenRw, 22,0); //22 = length of buffer
	maxIter = 100000;
	snseti_("Iterations limit", &maxIter, &iPrint, &iSumm, &inform, snCw,&snLenCw,snIw,&snLenIw,snRw,&snLenRw, 16,0); //16 = length of buffer
	integer sb = 700;
	snseti_("Superbasics limit",&sb, &iPrint, &iSumm, &inform, snCw,&snLenCw,snIw,&snLenIw,snRw,&snLenRw, 16,0); //16 = length of buffer
	
	//init glut
	init_glut(argc,argv);
	//init osg
	init_osg();

	int muscleUb;
	int jointUb;

	//manipulator
	visManip = new osg::MatrixTransform();

	//470 is the height of the foot location when the posture is sitting down.
	osg::Matrix mat;
	mat.makeTranslate(0,0,0);
	
	/*visManip2 = new osg::MatrixTransform();
	visManip2->addChild( osgDB::readNodeFile("props/floor2.3DS") );
	visManip2->setMatrix(mat);
	sceneGroup->addChild(visManip2.get());*/

	//osg::Matrix matHead;
	//matHead.makeTranslate(0,0,1750);
	//headSkin = new osg::MatrixTransform();
	////matHead.setTrans(v2);
	//			
	//headSkin.get()->addChild( osgDB::readNodeFile("props/Head/head4.3DS") );
	//headSkin.get()->setMatrix(matHead);
	//sceneGroup->addChild(headSkin.get());
	
	if(exportMotion){
		fileBall = fopen(".\\exportMotion\\BallMotion.txt","w");
		fileMotionLEye = fopen(".\\exportMotion\\MotionLEye.txt","w");
		//fileMotionLEye = fopen(".\\exportMotion\\MotionLEye.csv","w");
		fileMuscleMotionLEye = fopen(".\\exportMotion\\MuscleMotionLEye.txt","w");
		fileDimensionsPupil = fopen(".\\exportMotion\\PupilRadius.txt","w");
		fileDimensionsLens = fopen(".\\exportMotion\\LensWidthRadius.txt","w");
		fileRaytracePoints = fopen(".\\exportMotion\\RaytracePoints.txt","w");
	}
	//adjust trunk fb gains
	gReal scale = 0.1;
	   
	viewRot = 0*gDTR;
	//viewRot=90.1*gDTR; // lens portion
	//viewRot=20.0*gDTR; // pupil portion
	//viewRot=-75.0*gDTR; // reverse trace
	viewElev = 0.f;
	//viewElev = -34.0*gDTR;

#ifdef LENS_VIDEO
	viewRot = 90.0*gDTR;
	//viewRot = 70.1*gDTR;
	viewRadius=1500.f;
	//viewRadius=300.f;
	//viewRadius=330.f;
	//viewRadius=100.f;
	centerPos[0]=35.f; centerPos[1]=-345.f; centerPos[2]=1848.f;
	//centerPos[0]=35.f; centerPos[1]=-245.f; centerPos[2]=1848.f;
	//centerPos[0]=35.f; centerPos[1]=-105.f; centerPos[2]=1848.f;
#else
	//viewRot = 90.0*gDTR;
	viewRadius=100.f;
	//viewRadius=1000.f;
	centerPos[0]=35.f; centerPos[1]=-105.f; centerPos[2]=1848.f;
	//centerPos[0]=35.f; centerPos[1]=-115.f; centerPos[2]=1848.f;
#endif

	//recordMuscleAct = true;
	//if(recordMuscleAct) prepareRecord();

	//this is for generating training sets for deep neural net
	bTargetPosAchieved = false;

	vis_target_ec = 0.0;
	create_eye_ball_left();
	create_visual_target2();
	create_eye_gaze_hitbox();

	//create_eye_ball_right();

#ifdef SHOW_EYELINES
	createEyeLine();
#endif
#if defined(SHOW_RAYS) || defined(SHOW_LENS_SPHERES)
	createRaytraceLine();
	createFocalPointLine();
#endif

	//this if for rendering. when you want to use above function, disable this so that it is going to be faster and no display.
	glutMainLoop();

	finish();

	delete SysLEye;
	delete VisLEye;

#ifdef ONLINE_FOVEATION
	delete mRayKeras;
	delete sampleRayKeras;
#endif
#ifdef ONLINE_PUPIL
	delete mPupilKeras;
	delete samplePupilKeras;
#endif
#ifdef ONLINE_LENS
	delete mLensKeras;
	delete sampleLensKeras;
#endif
#ifdef ONLINE_MOTOR
	delete mLEye;
	delete sampleLEye;
#endif

	return 0;
}
