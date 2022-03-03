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

//#include "MuscleOptimizer/MuscleOptimizer.h"

//#include "SdA\Sda_Decoder.h"

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

// Taasin's Organization Code 2022
#include <globals.h>
#include "VisualTarget.h"
#include "EyeUtils.h"
#include "Utils.h"
#include "TrainingDataset.h"

void exportReverseTraceBuffer();
void raytracer(vector<Vec3f_local> spos, float radius, Vec3f_local ray_orig, Vec3f_local ray_di, Vec3f_local* returnedImagePupil, Vec3f_local* returnedImageLens, Vec3f_local* returnedImageFoveation, int side);
Vec3f_local trace(const Vec3f_local& rayorig, Vec3f_local& raydir,
	std::vector<Sphere_local>& Sphere_locals, const int& depth, int filterOption = 0);
void myComputeThetaPhi();
void myComputeThetaPhi2();
void readStdMeanLeft();

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

gVec3 get_currentLeftAmrPos();
gVec3 get_currentLeftAmrPos_no_print();
gVec3 computeTargetPos2(double* target, gVec3 cur_pos);
void generateTrainingDataFullBody();
void generateTrainingDataFullBody2();
void generateTrainingDataFullBody3();
void generateTrainingDataFullBody2015();

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

// void testSavedMuscleActivation();

/***************************************************************************/

#include "Keras\keras_model.h"
#include <iterator>

using namespace keras;

void resetToInitialPose();

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

void doOneStepPoseBasedTargetInverseDynamicsControlForGenerateTrainingDatasetIntergrateToNextTimestepChangingMagnitudeLeye(gReal coact, gReal interval, int incr, ofstream& foutput, ofstream& foutputDelta, ofstream& finput, bool generatingTrainingDataset, gVec3 target);

void computeThetaPhi();
gVec3 computeTargetUnitVecRegularized(double* target, gVec3 cur_pos);

void setLarmJointAngle();

void reflexTest(gVec3 v);

void computeDesiredMuscleState3(gVec3 vec, gVec3 cur, gReal timestep);

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
void move_visual_target_projectile_motion(osg::MatrixTransform* trans_visual_target, double time);

gVec3 getcurrent_ori();
gVec3 getcurrent_ori_leye();

void adjust_pupil_sphincter_muscles();
void compute_pupil_size();

void adjust_lens_cilliary_muscles();
void compute_lens_size();
void maybeRecordBallPos();
Vec3f_local compute_focal_point();

void computeDesiredMuscleState3LEye(gVec3 vec, gVec3 cur, gReal timestep);
void UpdateSimpleVertex(gReal elapsed_time);

void drawTargetBox();
void setCameraParameter();

#pragma comment(linker, "/STACK:1500000")
#pragma comment(linker, "/HEAP:1500000")

//using namespace osg;
#include <iostream>
//using namespace cv;
//#include "Perception\target_detection.h"

#include <algorithm>    // std::min
#include <fstream>

static gReal randomRange(gReal low, gReal up);

//#define PERLIN_TARGET_LOCATION

#ifdef PERLIN_TARGET_LOCATION

#include <Perlin\Perlin.h>
Perlin plarm;
Perlin prarm;
Perlin plleg;
Perlin prleg;

#endif

//[comment]
// This variable controls the maximum recursion depth
//[/comment]
#define MAX_RAY_DEPTH 4

float mix(const float &a, const float &b, const float &mix)
{
    return b * mix + a * (1 - mix);
}

static gReal randomNumber(gReal low, gReal high)
{
	return low + ((high - low) * rand()) / RAND_MAX;
}

static gReal randomRange(gReal low, gReal up)
{
	return rand() * ((up - low) / RAND_MAX) + low;
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


void maybeRecordBallPos() {
	cout << "lpfd len: " << lfpd.length() << endl;
	if(lfpd.length() <= 0.4) {
		gVec3 ballpos = visualTarget.get()->getMatrix().getTrans().ptr();
		finputlensmaybeballpos << ballpos.x() << "," << ballpos.y() << "," << ballpos.z() << "," << vis_target_ec << endl;
	}
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
	//drawTargetBox();

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
	//sceneViewLeft->getViewMatrixAsLookAt(eye,centre,up);

	if(visualTarget.get() != nullptr){
		 

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

#if defined(ONLINE_FOVEATION)
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

#if defined(ONLINE_FOVEATION)
				ray_vec.push_back(vf.x);
#endif
#ifdef ONLINE_PUPIL
				ray_pupil.push_back(vp.x);
#endif
#ifdef ONLINE_LENS
				ray_lens.push_back(vl.x);
#endif
			}
		
		double tyl, txl;
		static double prior_lens_error = FLT_MAX;
		static int prior_step_lens = 0;
		static bool firstTimeLens = true;
 		if(targetExist){
			cout << "Targer" << endl;
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

#if defined(ONLINE_FOVEATION)
			tyl = returned[0]*stdValueLeft[0] + meanValueLeft[0] + eye_cyl;
			txl = returned[1]*stdValueLeft[1] + meanValueLeft[1] + eye_cxl;
			tyl_temp = tyl;
			txl_temp = txl;
			// cout << "new estimated raw theta, phi (degrees): " << tyl*gRTD << " " << txl*gRTD << endl;
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
	//computeThetaPhi();
	//myComputeThetaPhi();

	sceneViewLeft->update();
	// do the cull traversal, collect all objects in the view frustum into a sorted set of rendering bins
	sceneViewLeft->cull();

	sceneViewLeft->draw();
	// Swap Buffers
	glutSwapBuffers();
    glutPostRedisplay();
}

//#define SHOW_PUPIL
//#define SHOW_LENS
#define SHOW_FOVEATION
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
				//testLoopKinematicAndDNNFixatonsDisplay();
				//generateTrainingDatasetMotorWithDisplay_fixaton();
				generateTrainingDatasetMotor();
				//generateTrainingDatasetMotor_fixaton_dataset();
#endif
				break;
			}
			case EYE_CONTROL:
			{
				dumpImages = true;
				// SNN demonstration
				//move_visual_target_saccade_fixed(visualTarget.get(), curTime);
				move_visual_target_smooth_pursuit(visualTarget.get(), curTime);
				//move_visual_target_smooth_pursuit_lateral(visualTarget.get(), curTime);
				//move_visual_target_fixed_position(visualTarget.get(), curTime);
				//move_visual_target_projectile_motion(visualTarget.get(), curTime);

				// Pupil demonstration
				//move_visual_target_ball_thrown(visualTarget.get(), curTime);
				//move_visual_target_smooth_pursuit(visualTarget.get(), curTime);
				
				// Pupil Training
				//move_visual_target_pupil_training(visualTarget.get(), curTime);

				// Lens Training
				//move_visual_target_lens_training(visualTarget.get(), curTime);

				// Foveation Training
				//move_visual_target_foveation_training(visualTarget.get(), curTime);

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
				//move_visual_target_saccade_fixed(visualTarget.get(), curTime);

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
				//computeThetaPhi();
				doMuscleControlKerasLEye(targeting_location);
#else
				// Inverse Dynamics
				//doInverseDynamicMuscleControlLeye(targeting_location);
				//computeThetaPhi();
				start_recording_foveation = true;
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
	while(cnt <= 1000000)
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
		if(cnt%1000==0) {
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
#ifdef ONLINE_EXE
	myComputeThetaPhi2();
#endif

#if defined(ONLINE_FOVEATION) || defined(ONLINE_EXE)
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

	// Everything below this just records data for plotting
	///*

	int num_eoc_muscles = 6;

	ofstream foutput("kerasOutputLeyeActivationsForPlotting.csv", ios::app);
	ofstream foutputori("angleRecordings/outputOriLeye.csv", ios::app);
	ofstream foutputvel("angleRecordings/outputVelLeye.csv", ios::app);
	ofstream foutputacc("angleRecordings/outputAccLeye.csv", ios::app);

	static double prev_vel[2] = { 0.0, 0.0 };

	for (int i = 0; i < num_eoc_muscles - 1; i++) {
		foutput << GV_get(SysLEye->actLevel(), i) << ",";
	}
	foutput << GV_get(SysLEye->actLevel(), num_eoc_muscles - 1) << endl;
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
	prev_vel[1] = velz;
	//*/
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

	curTime += simulationTimeStep;

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
	///*
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
	prev_vel[1] = velz;
	//*/
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



int main( int argc, char **argv )
{
	// TODO: error checking
	 // system("C:\\Users\\taasi\\Desktop\\RunSNN\\dist\\main\\main.exe");
	/*
	ofstream fdistoutput("eyeDist.csv", std::ofstream::out);
	float spread = 65, aspectratio = RAYTRACE_WIDTH / float(RAYTRACE_HEIGHT);

	double ray_width = RAYTRACE_WIDTH;
	//double divisor = 5.0;
	double divisor = 10.0;
	double maxRadius = 2.0 / 3.0 * std::exp(ray_width / divisor);
	double gaussian_mean = 0;
	double gaussian_spread = (double)RAYTRACE_WIDTH; // Peripheral falls 1 stdev / spread weight of center
	//5
	//#pragma omp parallel for num_threads(8)
	reverseRayBuffer.clear();
	for (int rho = 0; rho < RAYTRACE_WIDTH; rho++) {
		// Change_by_honglin 3: the increment is 1 instead of 4
		for (unsigned int thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT * 1; thetaLogPolar += 1) {
			// Change_by_honglin 4: generate new noise.txt and change the increment from 4 to 1
			double logPolarX = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0 / 3.0 * std::exp(double(rho / divisor) + retina_distribution_noise[rho + RAYTRACE_WIDTH * thetaLogPolar / 1]) * cos(double(thetaLogPolar * DEG_TO_RAD + retina_distribution_noise[rho + RAYTRACE_WIDTH * thetaLogPolar / 1]));
			double logPolarY = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0 / 3.0 * std::exp(double(rho / divisor) + retina_distribution_noise[rho + RAYTRACE_WIDTH * thetaLogPolar / 1]) * sin(double(thetaLogPolar * DEG_TO_RAD + retina_distribution_noise[rho + RAYTRACE_WIDTH * thetaLogPolar / 1]));

			float yy = (1 * ((logPolarX) / maxRadius)) * spread * D2R;
			float xx = (1 * ((logPolarY) / maxRadius)) * spread * D2R;
			fdistoutput << xx << ',' << yy << endl;
		}
	}
	fdistoutput.close();
	*/

	unsigned int i;
	int cnt=0;/*
	scanpath = new float*[122743];
	for(int i = 0; i < 122743; ++i)*/
	
	scanpath = new float*[13];
	for(int i = 0; i < 13; ++i)
		scanpath[i] = new float[4];

	ifstream ifs("scanepath3.csv", ios::in);
	if(!ifs){
		cout << "Could not open delta file. Exiting" << endl;
		return -1;
	}
	
	string str;
	int p=0;
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

#ifdef ONLINE_EXE
	readStdMeanLeft();
#endif

#ifdef DELTA_ONV
	// both files should be cleared when they are opened
	// write all zeros to ONV file to start
	ofstream fonvOut("C:\\Users\\taasi\\Desktop\\RunSNN\\files\\onvOut.csv", std::fstream::trunc);

	for (int i = 0; i < 14399; i++) {
		fonvOut << "0,";
	}
	fonvOut << "0" << endl;

	fonvOut.close();
#endif

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

	
	//init_glutLeft();
	//init_osgLeft();
	
	init_glut_PR();
//#if defined(TEST_PUPIL) || defined(TEST_LENS) || defined(TEST_FOVEATION)
//	init_glut_PRTest();
//#endif
	init_osg_PR();
	
	
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

	//initTargetBox();

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
