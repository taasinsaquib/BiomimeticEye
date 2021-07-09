void writeBallPoses(int frame, FILE* fp)
{
	assert(fp != NULL);
	
	fprintf(fp,"%d %g\n",frame,vis_target_ec);
	fprintf(fp,"%g %g %g\n", visualTarget.get()->getMatrix().getTrans().x(),visualTarget.get()->getMatrix().getTrans().y(), visualTarget.get()->getMatrix().getTrans().z());

	fflush(fp);
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