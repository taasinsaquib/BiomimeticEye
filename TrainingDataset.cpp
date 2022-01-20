#include "globals.h"
#include "TrainingDataset.h"

#include "gkd/gkd_multibody_system.h"

#include "VisualTarget.h"
#include "EyeUtils.h"
#include "Utils.h"

void move_visual_target_pupil_training(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_lens_training_new(osg::MatrixTransform* trans_visual_target, double time);
void move_visual_target_foveation_training(osg::MatrixTransform* trans_visual_target, double time);
gVec3 getcurrent_ori_leye();
void compute_pupil_size();
void compute_lens_size();
void setRandomPupilAndLensLengths();
void finish();
void raytracer(vector<Vec3f_local> spos, float radius, Vec3f_local ray_orig, Vec3f_local ray_di, Vec3f_local* returnedImagePupil, Vec3f_local* returnedImageLens, Vec3f_local* returnedImageFoveation, int side);
double phi_vec_conversion(gVec3 v);
double theta_vec_conversion(gVec3 v);
void resetToInitialPoseLeye();

void move_visual_target_smooth_pursuit_lateral(osg::MatrixTransform* trans_visual_target, double time);

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

void generateTrainingDatasetPupil()
{
	///*
	unsigned int i;
	int cnt = 0;
	int iterations = 10;
	final_record_iteration_pupil = false;

	//500000
	while (cnt <= iterations)
	{
		if (cnt == iterations) {
			final_record_iteration_pupil = true;
		}
		move_visual_target_pupil_training(visualTarget.get(), curTime);
		gVec3 ori_leye = getcurrent_ori_leye();
		gLink* base = SysLEye->findLink("Eyeball");
		gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
		gVec3 localTargetPos = targetWorldPos - base->frame().trn();
		targeting_location.set(localTargetPos.x(), localTargetPos.y(), localTargetPos.z());
		osg::Vec3 eye(base->frame().trn().x(), base->frame().trn().y(), base->frame().trn().z());
	//*/
		/*osg::Vec3d spherePos = visualTarget.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local = Vec3f_local(float(spherePos.x()),float(spherePos.y()),float(spherePos.z()));
		Vec3f_local returnedImage[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];*/
	///*
		osg::Vec3d spherePos1 = visualTarget.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local1 = Vec3f_local(float(spherePos1.x()), float(spherePos1.y()), float(spherePos1.z()));

		vector<Vec3f_local> spehere_poss;
		spehere_poss.push_back(sherePos_local1);

#ifdef SECOND_VISUAL_TARGET
		osg::Vec3d spherePos2 = visualTarget2.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local2 = Vec3f_local(float(spherePos2.x()), float(spherePos2.y()), float(spherePos2.z()));

		spehere_poss.push_back(sherePos_local2);
#endif

		Vec3f_local returnedImagePupil[RAYTRACE_WIDTH * RAYTRACE_HEIGHT];
		Vec3f_local returnedImageLens[RAYTRACE_WIDTH * RAYTRACE_HEIGHT];
		Vec3f_local returnedImageFoveation[RAYTRACE_WIDTH * RAYTRACE_HEIGHT];
		//raytracer(sherePos_local, SPHERE_SIZE, Vec3f_local(eye.x(),eye.y(), eye.z()), Vec3f_local(ori_leye.x()*gDTR,0,ori_leye.z()*gDTR),returnedImage,0);
		raytracer(spehere_poss, SPHERE_SIZE, Vec3f_local(eye.x(), eye.y(), eye.z()), Vec3f_local(ori_leye.x() * gDTR, 0, ori_leye.z() * gDTR), returnedImagePupil, returnedImageLens, returnedImageFoveation, 0);

		int repeat = (int)(displayTimeStep / simulationTimeStep) + 1;
		for (int i = 0; i < repeat; ++i) {
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
		if (cnt % 10 == 0)
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
	//*/
}

void generateTrainingDatasetLens()
{
	unsigned int i;
	int cnt = 0;
#ifdef FULL_LENS_DATASET_GENERATION
	int iterations = 10000;
#else
	int iterations = 200;
#endif
	final_record_iteration_lens = false;

	//500000
	while (cnt <= iterations)
	{
		if (cnt == iterations) {
			final_record_iteration_lens = true;
		}
		//move_visual_target_lens_training(visualTarget.get(), curTime);
		move_visual_target_lens_training_new(visualTarget.get(), curTime);
		gVec3 ori_leye = getcurrent_ori_leye();
		gLink* base = SysLEye->findLink("Eyeball");
		gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
		gVec3 localTargetPos = targetWorldPos - base->frame().trn();
		targeting_location.set(localTargetPos.x(), localTargetPos.y(), localTargetPos.z());
		osg::Vec3 eye(base->frame().trn().x(), base->frame().trn().y(), base->frame().trn().z());

		/*osg::Vec3d spherePos = visualTarget.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local = Vec3f_local(float(spherePos.x()),float(spherePos.y()),float(spherePos.z()));
		Vec3f_local returnedImage[RAYTRACE_WIDTH*RAYTRACE_HEIGHT];*/

		osg::Vec3d spherePos1 = visualTarget.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local1 = Vec3f_local(float(spherePos1.x()), float(spherePos1.y()), float(spherePos1.z()));

		vector<Vec3f_local> spehere_poss;
		spehere_poss.push_back(sherePos_local1);

#ifdef SECOND_VISUAL_TARGET

		osg::Vec3d spherePos2 = visualTarget2.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local2 = Vec3f_local(float(spherePos2.x()), float(spherePos2.y()), float(spherePos2.z()));

		spehere_poss.push_back(sherePos_local2);
#endif

		Vec3f_local returnedImagePupil[RAYTRACE_WIDTH * RAYTRACE_HEIGHT];
		Vec3f_local returnedImageLens[RAYTRACE_WIDTH * RAYTRACE_HEIGHT];
		Vec3f_local returnedImageFoveation[RAYTRACE_WIDTH * RAYTRACE_HEIGHT];
		//raytracer(sherePos_local, SPHERE_SIZE, Vec3f_local(eye.x(),eye.y(), eye.z()), Vec3f_local(ori_leye.x()*gDTR,0,ori_leye.z()*gDTR),returnedImage,0);
		raytracer(spehere_poss, SPHERE_SIZE, Vec3f_local(eye.x(), eye.y(), eye.z()), Vec3f_local(ori_leye.x() * gDTR, 0, ori_leye.z() * gDTR), returnedImagePupil, returnedImageLens, returnedImageFoveation, 0);

		int repeat = (int)(displayTimeStep / simulationTimeStep) + 1;
		for (int i = 0; i < repeat; ++i) {
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
		if (cnt % 10 == 0)
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
	int cnt = 0;
#ifdef FULL_FOVEATION_DATASET_GENERATION
	int iterations = 10000;
#else
	int iterations = 10;
#endif

	///*
	clock_t tStart = clock();
	double priorTimeStamp = (double)tStart;
	//*/

	while (cnt < iterations)
	{
		move_visual_target_foveation_training(visualTarget.get(), curTime);
		gVec3 ori_leye = getcurrent_ori_leye();
		gLink* base = SysLEye->findLink("Eyeball");
		gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
		gVec3 localTargetPos = targetWorldPos - base->frame().trn();
		targeting_location.set(localTargetPos.x(), localTargetPos.y(), localTargetPos.z());
		osg::Vec3 eye(base->frame().trn().x(), base->frame().trn().y(), base->frame().trn().z());

		osg::Vec3d spherePos1 = visualTarget.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local1 = Vec3f_local(float(spherePos1.x()), float(spherePos1.y()), float(spherePos1.z()));

		vector<Vec3f_local> spehere_poss;
		spehere_poss.push_back(sherePos_local1);

		Vec3f_local returnedImagePupil[RAYTRACE_WIDTH * RAYTRACE_HEIGHT];
		Vec3f_local returnedImageLens[RAYTRACE_WIDTH * RAYTRACE_HEIGHT];
		Vec3f_local returnedImageFoveation[RAYTRACE_WIDTH * RAYTRACE_HEIGHT];
		//raytracer(sherePos_local, SPHERE_SIZE, Vec3f_local(eye.x(),eye.y(), eye.z()), Vec3f_local(ori_leye.x()*gDTR,0,ori_leye.z()*gDTR),returnedImage,0);
		setRandomPupilAndLensLengths();
		raytracer(spehere_poss, SPHERE_SIZE, Vec3f_local(eye.x(), eye.y(), eye.z()), Vec3f_local(ori_leye.x() * gDTR, 0, ori_leye.z() * gDTR), returnedImagePupil, returnedImageLens, returnedImageFoveation, 0);

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
		if (cnt % 1 == 0) {
			cout << "iteration = " << cnt << endl;
			///*
			double curTime = (double)(clock());
			cout << "time taken: " << (curTime - priorTimeStamp) / CLOCKS_PER_SEC << endl;
			priorTimeStamp = curTime;
			//*/
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

/***************************************************************************/

void testGeneratedDatasetFromTrainingLeyeMotor()
{
	static bool bRead = false;
	int num_eoc_muscles = 6;
	static int cur = 0;
	int i = 0;
	int p = 0;
	int pd = 0;
	int q = 0;
	int qd = 0;
	static int totalRow = 0;
	static int totalRowDelta = 0;
	static int totalRowReset = 0;
	float base[6];
	if (!bRead) {
		ifstream ifs(".\\motorData\\outputTestDeltaMotorFixaton.csv", ios::app);
		ifstream finput(".\\motorData\\inputTestMotorFixaton.csv", ios::app);
		ifstream fresets(".\\motorData\\resetsFixaton.csv", ios::app);
		if (!ifs) {
			cout << "Could not open delta file. Exiting" << endl;
			return;
		}
		else if (!finput) {
			cout << "Could not open input file. Exiting" << endl;
			return;
		}
		string str;
		string inp;
		string res;
		while (getline(finput, inp)) {
			cout << "Reading line " << totalRow << ": " << inp << endl;
			string token;
			istringstream stream(inp);
			while (getline(stream, token, ',')) {
				double temp = stod(token);
				muscle_activation_data_leye[totalRow][q] = temp;
				q++;
				if (q >= num_eoc_muscles + 2) {
					q = 0;
				}
			}
			totalRow++;
			if (totalRow >= 1) {
				break;
			}
		}
		while (getline(ifs, str)) {
			cout << "Reading delta line " << totalRowDelta << ": " << str << endl;
			string token;
			istringstream stream(str);
			while (getline(stream, token, ',')) {
				double temp = stod(token);
				muscle_activation_data_delta_leye[totalRowDelta][qd] = temp;
				qd++;
				if (qd >= num_eoc_muscles) {
					qd = 0;
				}
			}
			totalRowDelta++;
			if (totalRowDelta >= 5000) {
				break;
			}
		}
		while (getline(fresets, res)) {
			cout << "Reading reset line " << totalRowReset << ": " << res << endl;
			resets_leye[totalRowReset] = stoi(res);
			totalRowReset++;
			if (totalRowReset >= 5000) {
				break;
			}
		}
		bRead = true;
		for (int t = 0; t < num_eoc_muscles; t++) {
			base[t] = muscle_activation_data_leye[0][t + 2];
		}
		for (int t = 0; t < num_eoc_muscles; t++) {
			SysLEye->setActLevel(t, base[t]);
		}
	}

	i = 0;
	if (resets_leye[cur] == 1) {
		cout << "FORCED EYE RESET" << endl;
		resetToInitialPoseLeye();
	}

	/*cout << "eye acceleration: " << SysLEye->eyeball->frameAcc().rot().magnitude() << endl;
	cout << "eye velocity: " << SysLEye->eyeball->frameVel().rot().magnitude() << endl;*/

	//cout << std::fixed;
	for (int t = 0; t < num_eoc_muscles; t++) {
		//cout << "act and delta: " << setprecision(15) << SysLEye->muscle(t)->actLevel() << " " << muscle_activation_data_delta_leye[cur][t] << endl;
		long double currAccLevel = SysLEye->muscle(t)->actLevel();
		SysLEye->setActLevel(t, currAccLevel + (long double)muscle_activation_data_delta_leye[cur][t]);
	}
	cur++;
	cout << "this is interation " << cur << endl;
	if (cur > totalRowDelta) {
		cout << "end of iteration. rewind to the top" << endl;
		bPlay = false;
	}

	static FILE* velocities = fopen("vels.csv", "w");
	fprintf(velocities, "%g\n", SysLEye->eyeball->frameVel().rot().magnitude());

	SysLEye->clearAllForcesWithoutMuscle();
	SysLEye->updateExtForce();
	SysLEye->computeAccelerationFeatherstone();

	/*for(int i = 3; i < SysLEye->dof(); i++) {
		GV_set(SysLEye->acc(),i,0.0);
	}*/

	int repeat = (int)(displayTimeStep / simulationTimeStep);
	for (int i = 0; i < repeat; ++i) {
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

	static double image_leye_pupil_temp[1500][RAYTRACE_WIDTH * RAYTRACE_HEIGHT];

	if (!bRead) {
		ifstream ifs(dirPrefix + "\\pupilData\\output_iris_sphincter_delta_act.csv");
		ifstream finput(dirPrefix + "\\pupilData\\iris_sphincter_act.csv");
		ifstream finputball(dirPrefix + "\\pupilData\\ballpos.csv");
		ifstream finputlenspupil(dirPrefix + "\\pupilData\\randomlensact.csv");
		ifstream finputimg(dirPrefix + "\\pupilData\\image_x_act_iris_sphincter.csv");
		if (!ifs) {
			cout << "Could not open delta file. Exiting" << endl;
			return;
		}
		else if (!finput) {
			cout << "Could not open input file. Exiting" << endl;
			return;
		}
		else if (!finputball) {
			cout << "Could not open ball file. Exiting" << endl;
			return;
		}
		else if (!finputlenspupil) {
			cout << "Could not open lens file. Exiting" << endl;
			return;
		}
		else if (!finputimg) {
			cout << "Could not open image file. Exiting" << endl;
			return;
		}
		string str;
		string inp;
		string bp;
		string lens;
		string img;
		while (getline(finput, inp)) {
			cout << "Reading line " << totalRow << ": " << inp << endl;
			string token;
			istringstream stream(inp);
			muscle_activation_data_leye_pupil_start = stof(inp);
			totalRow++;
			if (totalRow >= 1) {
				break;
			}
		}
		while (getline(ifs, str)) {
			cout << "Reading delta line " << totalRowDelta << ": " << str << endl;
			string token;
			istringstream stream(str);
			muscle_activation_data_delta_leye_pupil[totalRowDelta] = stof(str);
			totalRowDelta++;
			if (totalRowDelta >= numTestingSamples) {
				break;
			}
		}
		while (getline(finputlenspupil, lens)) {
			cout << "Reading lens line " << totalRowLensDelta << ": " << lens << endl;
			string token;
			istringstream stream(lens);
			random_lens_cilliary_act[totalRowLensDelta] = stof(lens);
			totalRowLensDelta++;
			if (totalRowLensDelta >= numTestingSamples) {
				break;
			}
		}
		while (getline(finputball, bp)) {
			cout << "Reading ball pos line " << totalRowBallPos << ": " << bp << endl;
			string token;
			istringstream stream(bp);
			int qd = 0;
			while (getline(stream, token, ',')) {
				double temp = stof(token);
				ballpos_leye_pupil[totalRowBallPos][qd] = temp;
				qd++;
				if (qd >= 4) {
					qd = 0;
				}
			}
			totalRowBallPos++;
			if (totalRowBallPos >= numTestingSamples) {
				break;
			}
		}
		while (getline(finputimg, img)) {
			cout << "Reading onv line " << totalRowImg << endl;
			string token;
			istringstream stream(img);
			int qd = 0;
			while (getline(stream, token, ',')) {
				double temp = stof(token);
				image_leye_pupil_temp[totalRowImg][qd] = temp;
				qd++;
				if (qd >= RAYTRACE_WIDTH * RAYTRACE_HEIGHT) {
					qd = 0;
					break;
				}
			}
			totalRowImg++;
			if (totalRowImg >= numTestingSamples) {
				break;
			}
		}
		bRead = true;
		for (int i = 6; i < 14; i++) {
			SysLEye->muscle(i)->setActLevel(muscle_activation_data_leye_pupil_start);
		}
	}

	for (int t = 6; t < 14; t++) {
		double currAccLevel = SysLEye->muscle(t)->actLevel();
		SysLEye->muscle(t)->setActLevel(currAccLevel + muscle_activation_data_delta_leye_pupil[cur]);
	}
	for (int i = 14; i < 22; i++) {
		SysLEye->muscle(i)->setActLevel(random_lens_cilliary_act[cur]);
	}
	for (int i = 38; i < 54; i++) {
		SysLEye->muscle(i)->setActLevel(0.072 * (1 - random_lens_cilliary_act[cur]));
	}

	for (int k = 0; k < RAYTRACE_WIDTH * RAYTRACE_HEIGHT; k++) {
		image_leye_pupil[k] = image_leye_pupil_temp[cur][k];
	}

	vis_target_ec = ballpos_leye_pupil[cur][3];
	osg::Matrixd mat;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

	sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));

	osg::Vec3 v(ballpos_leye_pupil[cur][0], ballpos_leye_pupil[cur][1], ballpos_leye_pupil[cur][2]);
	osg::Quat q(atan2(v.x(), v.z()), osg::Y_AXIS, -atan2(v.y() - 8, sqrt(v.x() * v.x() + v.z() * v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	visualTarget.get()->setMatrix(mat);

	cur++;
	cout << "this is iteration " << cur << endl;
	if (cur > totalRowDelta) {
		cout << "end of iteration. rewind to the top" << endl;
		bPlay = false;
	}

	int repeat = (int)(displayTimeStep / simulationTimeStep) + 1;
	for (int i = 0; i < repeat; i++)
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

	static double image_leye_lens_temp[1500][RAYTRACE_WIDTH * RAYTRACE_HEIGHT];

	//muscle_activation_data_leye_lens_start = 0.0;
	if (!bRead) {
		ifstream ifs(dirPrefix + "\\lensData\\output_lens_cilliary_delta_act.csv");
		ifstream finput(dirPrefix + "\\lensData\\cilliary_act.csv");
		ifstream finputball(dirPrefix + "\\lensData\\ballpos.csv");
		ifstream finputpupil(dirPrefix + "\\lensData\\randompupilact.csv");
		ifstream finputimg(dirPrefix + "\\lensData\\image_x_act_cilliary.csv");
		if (!ifs) {
			cout << "Could not open delta file. Exiting" << endl;
			return;
		}
		else if (!finput) {
			cout << "Could not open input file. Exiting" << endl;
			return;
		}
		else if (!finputball) {
			cout << "Could not open ball file. Exiting" << endl;
			return;
		}
		else if (!finputpupil) {
			cout << "Could not open random pupil act. Exiting" << endl;
			return;
		}
		else if (!finputimg) {
			cout << "Could not open image file. Exiting" << endl;
			return;
		}
		string str;
		string inp;
		string bp;
		string pupil;
		string img;
		while (getline(finput, inp)) {
			cout << "Reading line " << totalRow << ": " << inp << endl;
			string token;
			istringstream stream(inp);
			muscle_activation_data_leye_lens_start = stof(inp);
			totalRow++;
			if (totalRow >= 1) {
				break;
			}
		}
		while (getline(ifs, str)) {
			cout << "Reading delta line " << totalRowDelta << ": " << str << endl;
			string token;
			istringstream stream(str);
			muscle_activation_data_delta_leye_lens[totalRowDelta] = stof(str);
			totalRowDelta++;
			if (totalRowDelta >= numTestingSamples) {
				break;
			}
		}
		while (getline(finputpupil, pupil)) {
			cout << "Reading pupil line " << totalRowPupilAct << ": " << pupil << endl;
			string token;
			istringstream stream(pupil);
			random_pupil_sphincter_act[totalRowPupilAct] = stof(pupil);
			totalRowPupilAct++;
			if (totalRowPupilAct >= numTestingSamples) {
				break;
			}
		}
		while (getline(finputball, bp)) {
			cout << "Reading ball pos line " << totalRowBallPos << ": " << bp << endl;
			string token;
			istringstream stream(bp);
			int qd = 0;
			while (getline(stream, token, ',')) {
				double temp = stof(token);
				ballpos_leye_lens[totalRowBallPos][qd] = temp;
				qd++;
				if (qd >= 4) {
					qd = 0;
				}
			}
			totalRowBallPos++;
			if (totalRowBallPos >= numTestingSamples) {
				break;
			}
		}
		while (getline(finputimg, img)) {
			cout << "Reading onv line " << totalRowImg << endl;
			string token;
			istringstream stream(img);
			int qd = 0;
			while (getline(stream, token, ',')) {
				double temp = stof(token);
				image_leye_lens_temp[totalRowImg][qd] = temp;
				qd++;
				if (qd >= RAYTRACE_WIDTH * RAYTRACE_HEIGHT) {
					qd = 0;
					break;
				}
			}
			totalRowImg++;
			if (totalRowImg >= numTestingSamples) {
				break;
			}
		}
		bRead = true;
		for (int i = 14; i < 22; i++) {
			SysLEye->muscle(i)->setActLevel(muscle_activation_data_leye_lens_start);
		}
		for (int i = 38; i < 54; i++) {
			SysLEye->muscle(i)->setActLevel(0.072 * (1 - muscle_activation_data_leye_lens_start));
		}
	}

	double currAccLevel = SysLEye->muscle(14)->actLevel();
	for (int t = 14; t < 22; t++) {
		SysLEye->muscle(t)->setActLevel(currAccLevel + muscle_activation_data_delta_leye_lens[cur]);
	}
	for (int t = 38; t < 54; t++) {
		SysLEye->muscle(t)->setActLevel(0.072 * (1 - (currAccLevel + muscle_activation_data_delta_leye_lens[cur])));
	}

	for (int i = 6; i < 14; i++) {
		SysLEye->muscle(i)->setActLevel(random_pupil_sphincter_act[cur]);
	}

	for (int k = 0; k < RAYTRACE_WIDTH * RAYTRACE_HEIGHT; k++) {
		image_leye_lens[k] = image_leye_lens_temp[cur][k];
	}

	vis_target_ec = ballpos_leye_lens[cur][3];
	osg::Matrixd mat;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

	sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));

	osg::Vec3 v(ballpos_leye_lens[cur][0], ballpos_leye_lens[cur][1], ballpos_leye_lens[cur][2]);
	osg::Quat q(atan2(v.x(), v.z()), osg::Y_AXIS, -atan2(v.y() - 8, sqrt(v.x() * v.x() + v.z() * v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	visualTarget.get()->setMatrix(mat);

	cur++;
	cout << "this is interation " << cur << endl;
	if (cur > totalRowDelta) {
		cout << "end of iteration. rewind to the top" << endl;
		bPlay = false;
	}

	int repeat = (int)(displayTimeStep / simulationTimeStep) + 1;
	for (int i = 0; i < repeat; i++)
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

	static double image_leye_foveation_temp[1500][RAYTRACE_WIDTH * RAYTRACE_HEIGHT];

	if (!bRead) {
		ifstream finputimg(dirPrefix + "\\perceptionData\\image_x.csv");
		ifstream foutput(dirPrefix + "\\perceptionData\\output_etheta_ephi.csv");
		ifstream finputball(dirPrefix + "\\perceptionData\\ballpos.csv");
		ifstream finputpupil(dirPrefix + "\\perceptionData\\randompupilact.csv");
		ifstream finputlens(dirPrefix + "\\perceptionData\\randomlensact.csv");

		if (!finputimg) {
			cout << "Could not open image file. Exiting" << endl;
			return;
		}
		else if (!foutput) {
			cout << "Could not open output file. Exiting" << endl;
			return;
		}
		else if (!finputball) {
			cout << "Could not open ball file. Exiting" << endl;
			return;
		}
		else if (!finputpupil) {
			cout << "Could not open random pupil act. Exiting" << endl;
			return;
		}
		else if (!finputlens) {
			cout << "Could not open random pupil act. Exiting" << endl;
			return;
		}
		string img;
		string etp;
		string bp;
		string pupil;
		string lens;

		while (getline(finputimg, img)) {
			cout << "Reading onv line " << totalRowImg << endl;
			string token;
			istringstream stream(img);
			int qd = 0;
			while (getline(stream, token, ',')) {
				double temp = stof(token);
				image_leye_foveation_temp[totalRowImg][qd] = temp;
				qd++;
				if (qd >= RAYTRACE_WIDTH * RAYTRACE_HEIGHT) {
					qd = 0;
					break;
				}
			}
			totalRowImg++;
			if (totalRowImg >= numTestingSamples) {
				break;
			}
		}
		while (getline(foutput, etp)) {
			cout << "Reading output line " << totalRowEtp << endl;
			string token;
			istringstream stream(etp);
			int qd = 0;
			while (getline(stream, token, ',')) {
				double temp = stof(token);
				etheta_ephi_perception[totalRowEtp][qd] = temp;
				qd++;
				if (qd >= 2) {
					qd = 0;
					break;
				}
			}
			totalRowEtp++;
			if (totalRowEtp >= numTestingSamples) {
				break;
			}
		}
		while (getline(finputpupil, pupil)) {
			cout << "Reading pupil line " << totalRowPupilAct << ": " << pupil << endl;
			string token;
			istringstream stream(pupil);
			random_pupil_sphincter_act_foveation[totalRowPupilAct] = stof(pupil);
			totalRowPupilAct++;
			if (totalRowPupilAct >= numTestingSamples) {
				break;
			}
		}
		while (getline(finputlens, lens)) {
			cout << "Reading lens line " << totalRowLensAct << ": " << lens << endl;
			string token;
			istringstream stream(lens);
			random_lens_cilliary_act_foveation[totalRowLensAct] = stof(lens);
			totalRowLensAct++;
			if (totalRowLensAct >= numTestingSamples) {
				break;
			}
		}
		while (getline(finputball, bp)) {
			cout << "Reading ball pos line " << totalRowBallPos << ": " << bp << endl;
			string token;
			istringstream stream(bp);
			int qd = 0;
			while (getline(stream, token, ',')) {
				double temp = stof(token);
				ballpos_leye_foveation[totalRowBallPos][qd] = temp;
				qd++;
				if (qd >= 4) {
					qd = 0;
				}
			}
			totalRowBallPos++;
			if (totalRowBallPos >= numTestingSamples) {
				break;
			}
		}
		bRead = true;
	}

	SysLEye->adjust_pupil(random_pupil_sphincter_act_foveation[cur]);
	SysLEye->adjust_lens(random_lens_cilliary_act_foveation[cur]);

	for (int k = 0; k < RAYTRACE_WIDTH * RAYTRACE_HEIGHT; k++) {
		image_leye_perception[k] = image_leye_foveation_temp[cur][k];
	}

	vis_target_ec = ballpos_leye_foveation[cur][3];
	osg::Matrixd mat;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

	sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));

	osg::Vec3 v(ballpos_leye_foveation[cur][0], ballpos_leye_foveation[cur][1], ballpos_leye_foveation[cur][2]);
	osg::Quat q(atan2(v.x(), v.z()), osg::Y_AXIS, -atan2(v.y() - 8, sqrt(v.x() * v.x() + v.z() * v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	visualTarget.get()->setMatrix(mat);

	gLink* base = SysLEye->findLink("Eyeball");
	gVec3 targetWorldPos = visualTarget.get()->getMatrix().getTrans().ptr();
	gVec3 localTargetPos = targetWorldPos - base->frame().trn();
	double theta = theta_vec_conversion(localTargetPos);
	double phi = -phi_vec_conversion(localTargetPos);

	cout << "Etha (reocrded), Etheta (live), diff: " << etheta_ephi_perception[cur][0] << " " << theta << " " << abs(etheta_ephi_perception[cur][0] - theta) << endl;
	cout << "Ephi (reocrded), Ephi (live), diff: " << etheta_ephi_perception[cur][1] << " " << phi << " " << abs(etheta_ephi_perception[cur][1] - phi) << endl;

	cur++;
	cout << "this is interation " << cur << endl;
	if (cur > totalRowImg) {
		cout << "end of iteration. rewind to the top" << endl;
		bPlay = false;
	}
}