#include "globals.h"
#include "VisualTarget.h"

void create_visual_target(void)
{
	visualTarget = new osg::MatrixTransform;
	visualTarget.get()->setDataVariance(osg::Object::DYNAMIC);
	//use this if we want to have some other object other than sphere. we can import 3ds models maybe baseball
	//visualTarget.get()->addChild( osgDB::readNodeFile("prop/legoman2.3ds"));

#ifdef CONTROL_LEGS
	osg::Sphere* unitCube = new osg::Sphere(osg::Vec3(0, 0, 0), SPHERE_SIZE);
#else
	osg::Sphere* unitCube = new osg::Sphere(osg::Vec3(0, 0, 0), SPHERE_SIZE);
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
	visualTarget.get()->addChild(osg::ref_ptr<osg::Node>(osgDB::readNodeFile("props\\Soccer\\soccer.3DS")));
#else

	//visualTarget.get()->addChild( osg::ref_ptr<osg::Node>(osgDB::readNodeFile("props\\Drone\\drone_blue.osg")));
	visualTarget.get()->addChild(basicShapesGeode);
#endif
	//gLink* base = SysNeck->root();
	//base = SysNeck->findLink("Skull"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Skull");
	gVec3 baseWorldPos = base->frame().trn();

	//initial position
	osg::Matrixd mat;

#ifdef CONTROL_LEGS

	mat.makeTranslate(baseWorldPos.x() - 30, baseWorldPos.y() - 650, baseWorldPos.z() - 1500);
#else
	mat.makeTranslate(baseWorldPos.x() - 30, baseWorldPos.y() - 500, baseWorldPos.z() + 50);
#endif
	visualTarget.get()->setMatrix(mat);


	if (sceneGroup) sceneGroup->addChild(visualTarget.get());

	//randomMotion.set_bbox(Vec3(-30,-5,25),Vec3(30,5,30));
	//randomMotion.set_bbox(Vec3(-7,-7,25),Vec3(7,7,30));
	randomMotion.set_bbox(osg::Vec3(baseWorldPos.x() + 3, baseWorldPos.y() + 4, baseWorldPos.z() - 1), osg::Vec3(baseWorldPos.x() + 3, baseWorldPos.y() - 4, baseWorldPos.z() + 1));
	//randomMotion.set_bbox(osg::Vec3(baseWorldPos.x()-300,baseWorldPos.y()-100,baseWorldPos.z()-150),osg::Vec3(baseWorldPos.x()+300,baseWorldPos.y()-450,baseWorldPos.z()+150));
	//randomMotion.set_bbox(osg::Vec3(baseWorldPos.x()+100,baseWorldPos.y()-100,baseWorldPos.z()-1150),osg::Vec3(baseWorldPos.x()+500,baseWorldPos.y()-250,baseWorldPos.z()-850));
#ifdef INSTANT_MODE
	randomMotion.set_mode(RandomMotion::INSTANT);
	randomMotion.set_rangeSpeed(20, 50);
#else
	randomMotion.set_mode(RandomMotion::LINEAR);
	randomMotion.set_rangeSpeed(1, 30);
#endif

	//randomMotion.set_mode(RandomMotion::LINEAR);
	randomMotion.set_noisy(true);
	randomMotion.set_rangeNoiseSpeed(0, 0.4);
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

	osg::Sphere* unitCube = new osg::Sphere(osg::Vec3(0, 0, 0), SPHERE_SIZE);
#else
	osg::Sphere* unitCube = new osg::Sphere(osg::Vec3(0, 0, 0), SPHERE_SIZE);
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
	mater->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0, 1.0, 1.0, 1.0));
	mater->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0, 1.0, 1.0, 1.0));
	mater->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0, 1.0, 1.0, 1.0));
	mater->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0, 1.0, 1.0, 1.0));
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
	visualTarget.get()->addChild(osg::ref_ptr<osg::Node>(osgDB::readNodeFile("props\\Soccer\\soccer.3DS")));
#else
	visualTarget.get()->addChild(basicShapesGeode);

#ifdef SECOND_VISUAL_TARGET
	visualTarget2.get()->addChild(basicShapesGeode);
#endif
#endif

	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball");
	gVec3 baseWorldPos = base->frame().trn();

	osg::Matrixd mat;

#ifdef CONTROL_LEGS

	mat.makeTranslate(baseWorldPos.x() - 30, baseWorldPos.y() - 650, baseWorldPos.z() - 1500);
#else
	//mat.makeTranslate(baseWorldPos.x(), baseWorldPos.y()+yplane, baseWorldPos.z()); // use this
	mat.makeTranslate(baseWorldPos.x(), baseWorldPos.y() - 445, baseWorldPos.z()); // use this
#endif
	visualTarget.get()->setMatrix(mat);
	if (sceneGroup) {
		sceneGroup->addChild(visualTarget.get());
	}
#ifdef SECOND_VISUAL_TARGET
	visualTarget2.get()->setMatrix(mat);
	if (sceneGroup) {
		sceneGroup->addChild(visualTarget2.get());
	}
#endif

	//randomMotion.set_bbox(Vec3(-30,-5,25),Vec3(30,5,30));
	//randomMotion.set_bbox(Vec3(-7,-7,25),Vec3(7,7,30));
	randomMotion.set_bbox(osg::Vec3(baseWorldPos.x() + 3, baseWorldPos.y() + 4, baseWorldPos.z() - 1), osg::Vec3(baseWorldPos.x() + 3, baseWorldPos.y() - 4, baseWorldPos.z() + 1));
	randomMotion2.set_bbox(osg::Vec3(baseWorldPos.x() + 3, baseWorldPos.y() + 4, baseWorldPos.z() - 1), osg::Vec3(baseWorldPos.x() + 3, baseWorldPos.y() - 4, baseWorldPos.z() + 1));
#ifdef INSTANT_MODE
	randomMotion.set_mode(RandomMotion::INSTANT);
	randomMotion.set_rangeSpeed(20, 50);
#else
	randomMotion.set_mode(RandomMotion::LINEAR);
	randomMotion.set_rangeSpeed(1, 30);

	randomMotion2.set_mode(RandomMotion::LINEAR);
	randomMotion2.set_rangeSpeed(1, 30);
#endif

	//randomMotion.set_mode(RandomMotion::LINEAR);
	randomMotion.set_noisy(true);
	randomMotion.set_rangeNoiseSpeed(0, 0.4);
	randomMotion.set_noisyDistance(2);
	randomMotion.set_initialPosition(mat.getTrans());

	randomMotion2.set_noisy(true);
	randomMotion2.set_rangeNoiseSpeed(0, 0.4);
	randomMotion2.set_noisyDistance(2);
	randomMotion2.set_initialPosition(mat.getTrans());
}

void move_visual_target(osg::MatrixTransform* trans_visual_target, double time)
{
	osg::Matrixd mat;
	bool new_target = randomMotion.sol_pos();

	// set the doll's orientation as facing against the head
	osg::Vec3 v(randomMotion.get_curPos());
	osg::Quat q(atan2(v.x(), v.z()), osg::Y_AXIS, -atan2(v.y() - 8, sqrt(v.x() * v.x() + v.z() * v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);
}

void move_visual_target_eye_training(osg::MatrixTransform* trans_visual_target, double time)
{
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos = base->frame().trn();

	//randomMotion.set_bbox(osg::Vec3(baseWorldPos.x()-250,baseWorldPos.y()-4000,baseWorldPos.z()-250),osg::Vec3(baseWorldPos.x()+250,baseWorldPos.y()-400,baseWorldPos.z()+250));
	//randomMotion.set_bbox(osg::Vec3(baseWorldPos.x()-120,baseWorldPos.y()-500,baseWorldPos.z()-130),osg::Vec3(baseWorldPos.x()+120,baseWorldPos.y()-400,baseWorldPos.z()+130));
	randomMotion.set_bpyramid(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 250, baseWorldPos.z()), osg::Vec3(baseWorldPos.x() - 300, baseWorldPos.y() - 1200, baseWorldPos.z() - 300)); // use this
	randomMotion.set_mode(RandomMotion::INSTANT);
	//randomMotion.set_rangeSpeed(1*100,1.1*100);
	//randomMotion.set_rangeSpeed(4,4.1); // USE OR REVERT TO THIS
	//randomMotion.set_rangeSpeed(5,5.1); // USE OR REVERT TO THIS
	randomMotion.set_rangeSpeed(1 * 50, 1.1 * 50);

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
	osg::Quat q(atan2(v.x(), v.z()), osg::Y_AXIS, -atan2(v.y() - 8, sqrt(v.x() * v.x() + v.z() * v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);
}

void move_visual_target_saccade_fixed(osg::MatrixTransform* trans_visual_target, double time)
{
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos = base->frame().trn();

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

	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() + yplane, baseWorldPos.z()));
	targetIntensities.push_back(0.7);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x() - xbound / 1.3, baseWorldPos.y() + yplane, baseWorldPos.z() + zbound / 2));
	targetIntensities.push_back(0.7);
	//targetIntensities.push_back(0.4);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x() + 2 * xbound / 3, baseWorldPos.y() + yplane, baseWorldPos.z() - 2 * zbound / 3));
	targetIntensities.push_back(0.4);
	//targetIntensities.push_back(0.8);
	/*targetPoints.push_back(osg::Vec3(baseWorldPos.x()+xbound, baseWorldPos.y()+yplane, baseWorldPos.z()-2*zbound/3));
	targetIntensities.push_back(0.4);*/
	targetPoints.push_back(osg::Vec3(baseWorldPos.x() - xbound / 3.1, baseWorldPos.y() + yplane, baseWorldPos.z() - zbound / 3));
	//targetPoints.push_back(osg::Vec3(baseWorldPos.x()-xbound/1.7, baseWorldPos.y()-400, baseWorldPos.z()+zbound/2));
	//targetPoints.push_back(osg::Vec3(baseWorldPos.x()-xbound/1.1, baseWorldPos.y()-400, baseWorldPos.z()));
	//targetIntensities.push_back(0.8);
	targetIntensities.push_back(0.6);

	/*targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()+yplane, baseWorldPos.z()));
	targetIntensities.push_back(0.99);*/

	targetPoints.push_back(osg::Vec3(baseWorldPos.x() - xbound / 7, baseWorldPos.y() + yplane, baseWorldPos.z() + zbound / 2));
	targetIntensities.push_back(0.2);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x() + xbound / 1.2, baseWorldPos.y() + yplane, baseWorldPos.z() + zbound / 1.3));
	targetIntensities.push_back(0.5);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x() + xbound / 1.2, baseWorldPos.y() + yplane, baseWorldPos.z() - zbound / 1.3));
	targetIntensities.push_back(0.6);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() + yplane, baseWorldPos.z() + zbound / 1.2));
	targetIntensities.push_back(0.4);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x() - xbound / 1.2, baseWorldPos.y() + yplane, baseWorldPos.z()));
	targetIntensities.push_back(0.9);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() + yplane, baseWorldPos.z() - zbound / 1.1));
	targetIntensities.push_back(0.5);

	static int targetIndex = 0;
	static int cnt = 0;

	if (cnt < tot_count) {
		new_target_ball = false;
		cnt++;
	}
	else {
		targetIndex = min(targetIndex + 1, targetPoints.size() - 1);
		new_target_ball = true;
		if (tot_count == 1) {
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

	if (cntVis < tot_count_vis) {
		cntVis++;
	}
	else {
		visTargetIndex = (visTargetIndex + 1) % targetIntensities.size();
		if (tot_count_vis == 1) {
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
	sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));

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
	gVec3 baseWorldPos = base->frame().trn();

	osg::Vec3 startPos(baseWorldPos.x(), baseWorldPos.y() + yplane, baseWorldPos.z());
	static osg::Vec3 cur_pos = startPos;

	float xbound = 120;
	float zbound = 130;

	// Target positions
	vector<osg::Vec3> targetPoints;
	vector<double> targetIntensities;

	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() + yplane, baseWorldPos.z()));
	targetIntensities.push_back(0.7);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x() + xbound, baseWorldPos.y() + yplane, baseWorldPos.z() - zbound));
	targetIntensities.push_back(0.4);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x() + xbound, baseWorldPos.y() + yplane, baseWorldPos.z() + zbound));
	targetIntensities.push_back(0.9);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x() - xbound, baseWorldPos.y() + yplane, baseWorldPos.z() + zbound));
	targetIntensities.push_back(0.3);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x() + xbound, baseWorldPos.y() + yplane, baseWorldPos.z() - zbound));
	targetIntensities.push_back(0.8);

	static int targetIndex = 0;
	static int cnt = 0;

	static double one_step_int = 0.7 / tot_count;

	if (cnt < tot_count) {
		cnt++;
	}
	else {
		//osg::Vec3 oldTarget = targetPoints.at(targetIndex);
		double oldTargetIntensity = targetIntensities.at(targetIndex);
		targetIndex = min(targetIndex + 1, targetPoints.size() - 1);
		//osg::Vec3 newTarget = targetPoints.at(targetIndex);
		double newTargetIntensity = targetIntensities.at(targetIndex);
		if (tot_count == 30) {
			tot_count *= 3;
		}
		cnt = 0;
		cout << "intensities: " << oldTargetIntensity << " " << newTargetIntensity << " " << newTargetIntensity - oldTargetIntensity << endl;
		//one_step = (newTarget-oldTarget) / tot_count;
		one_step_int = (newTargetIntensity - oldTargetIntensity) / tot_count;
	}

	cout << "vis target ec: " << vis_target_ec << endl;

	osg::Vec3 target = targetPoints.at(targetIndex);
	double targetIntensity = targetIntensities.at(targetIndex);
	one_step = target - cur_pos;
	one_step /= tot_count;
	//one_step_int = targetIntensity-vis_target_ec;
	//one_step_int /= tot_count;
	cur_pos += one_step;
	vis_target_ec = (one_step_int > 0) ? min(targetIntensity, vis_target_ec + one_step_int) : max(targetIntensity, vis_target_ec + one_step_int);

	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	trans_visual_target->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);
	sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));

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
	gVec3 baseWorldPos = base->frame().trn();

	osg::Vec3 startPos(baseWorldPos.x(), baseWorldPos.y() - 350, baseWorldPos.z());
	static osg::Vec3 cur_pos = startPos;

	// Target positions
	vector<osg::Vec3> targetPoints;
	vector<double> targetIntensities;

	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 350, baseWorldPos.z()));
	targetIntensities.push_back(0.6);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 650, baseWorldPos.z()));
	targetIntensities.push_back(0.6);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 900, baseWorldPos.z()));
	targetIntensities.push_back(0.6);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 500, baseWorldPos.z()));
	targetIntensities.push_back(0.6);
	/*targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y()+yplane, baseWorldPos.z()));
	targetIntensities.push_back(0.8);*/

	static int targetIndex = 0;
	static int cnt = 0;

	static double one_step_int = 0.6 / tot_count;

	if (cnt < tot_count) {
		cnt++;
	}
	else {
		//osg::Vec3 oldTarget = targetPoints.at(targetIndex);
		double oldTargetIntensity = targetIntensities.at(targetIndex);
		targetIndex = min(targetIndex + 1, targetPoints.size() - 1);
		//osg::Vec3 newTarget = targetPoints.at(targetIndex);
		double newTargetIntensity = targetIntensities.at(targetIndex);
		if (tot_count == 50) {
			tot_count += 2;
		}
		cnt = 0;
		cout << "intensities: " << oldTargetIntensity << " " << newTargetIntensity << " " << newTargetIntensity - oldTargetIntensity << endl;
		//one_step = (newTarget-oldTarget) / tot_count;
		one_step_int = (newTargetIntensity - oldTargetIntensity) / tot_count;
	}

	cout << "vis target ec: " << vis_target_ec << endl;
	if (tot_count == 50) {
		vis_target_ec = 0.6;
	}

	osg::Vec3 target = targetPoints.at(targetIndex);
	double targetIntensity = targetIntensities.at(targetIndex);
	one_step = target - cur_pos;
	one_step /= tot_count;
	//one_step_int = targetIntensity-vis_target_ec;
	//one_step_int /= tot_count;
	cur_pos += one_step;
	vis_target_ec = (one_step_int > 0) ? min(targetIntensity, vis_target_ec + one_step_int) : max(targetIntensity, vis_target_ec + one_step_int);

	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	trans_visual_target->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);
	sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));

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
	sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));

	osg::Vec3 one_step;
	static int counter = 0;
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos = base->frame().trn();

	osg::Vec3 startPos(baseWorldPos.x(), baseWorldPos.y() + yplane, baseWorldPos.z());
	static osg::Vec3 cur_pos = startPos;

	float xval = 60 * sin(counter * gDTR);
	counter++;

	osg::Vec3 target = osg::Vec3(baseWorldPos.x() + xval, baseWorldPos.y() + yplane, baseWorldPos.z());
	one_step = target - cur_pos;
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
	gVec3 baseWorldPos = base->frame().trn();

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
	sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
#endif

	osg::Vec3 one_step, one_step2;
	static int counter = 0;

	osg::Vec3 startPos(baseWorldPos.x(), baseWorldPos.y() - 400, baseWorldPos.z());
	static osg::Vec3 cur_pos = startPos;
	static osg::Vec3 cur_pos2 = startPos;

	float yval = 200 * sin(3 * counter * gDTR);
	counter++;

	osg::Vec3 targetLoc(baseWorldPos.x(), baseWorldPos.y() - 400 + yval, baseWorldPos.z());

	osg::Vec3 target = osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 400 + yval, baseWorldPos.z());
	osg::Vec3 target2 = osg::Vec3(baseWorldPos.x() - 30, baseWorldPos.y() - 400 - yval, baseWorldPos.z());
	one_step = target - cur_pos;
	one_step2 = target2 - cur_pos2;
	//one_step /= tot_count;
	cur_pos += one_step;
	cur_pos2 += one_step2;

	osg::Matrixd mat, mat2;
	osg::Quat q(atan2(cur_pos.x(), cur_pos.z()), osg::Y_AXIS, -atan2(cur_pos.y() - 8, sqrt(cur_pos.x() * cur_pos.x() + cur_pos.z() * cur_pos.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat2.makeRotate(q);
	mat.setTrans(cur_pos);
	trans_visual_target->setMatrix(mat);

	osg::Quat q2(atan2(cur_pos2.x(), cur_pos2.z()), osg::Y_AXIS, -atan2(cur_pos2.y() - 8, sqrt(cur_pos2.x() * cur_pos2.x() + cur_pos2.z() * cur_pos2.z())), osg::X_AXIS,
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
	gVec3 baseWorldPos = base->frame().trn();

	//pos.set(baseWorldPos.x()+120, baseWorldPos.y()-500, baseWorldPos.z()-180);
	pos.set(baseWorldPos.x(), baseWorldPos.y() - 500, baseWorldPos.z()); // use this
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
			vis_target_ec = max(0.1, double(randVal) / double(RAND_MAX) + 1.0 / double(RAND_MAX));
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

		sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
		sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	}

	counter++;

	// set the doll's orientation as facing against the head
	osg::Vec3 v(pos);
	osg::Quat q(atan2(v.x(), v.z()), osg::Y_AXIS, -atan2(v.y() - 8, sqrt(v.x() * v.x() + v.z() * v.z())), osg::X_AXIS,
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
	gVec3 baseWorldPos = base->frame().trn();

	vis_target_ec = 0.99;
	osg::ref_ptr<osg::Material> mater2 = new osg::Material;
	mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

	sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));

	static int tot_count = 1;

	// Target positions
	vector<osg::Vec3> targetPoints;

	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y(), baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 90.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 800.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 200.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 2000.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 300.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 350.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 400.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 450.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 500.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 550.0, baseWorldPos.z()));
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 600.0, baseWorldPos.z()));

	static int targetIndex = 0;
	static int cnt = 0;
	static int current_index = 0;

	if (cnt < tot_count) {
		new_target_ball = false;
		cnt++;
	}
	else {
		targetIndex = min(targetIndex + 1, targetPoints.size() - 1);
		new_target_ball = true;
		if (tot_count == 1) {
			tot_count *= 60;
		}
		cnt = 0;
	}

	osg::Vec3 target = targetPoints.at(targetIndex);

	osg::Matrixd mat;
	osg::Vec3 v(target);
	osg::Quat q(atan2(v.x(), v.z()), osg::Y_AXIS, -atan2(v.y() - 8, sqrt(v.x() * v.x() + v.z() * v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);
}

void move_visual_target_pupil_training(osg::MatrixTransform* trans_visual_target, double time)
{
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos = base->frame().trn();

	randomMotion.set_bpyramid(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 350, baseWorldPos.z()), osg::Vec3(baseWorldPos.x() - 200, baseWorldPos.y() - 1250, baseWorldPos.z() - 200));
	randomMotion2.set_bpyramid(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 350, baseWorldPos.z()), osg::Vec3(baseWorldPos.x() - 200, baseWorldPos.y() - 1250, baseWorldPos.z() - 200));

	randomMotion.set_mode(RandomMotion::INSTANT);
	randomMotion2.set_mode(RandomMotion::INSTANT);
	//randomMotion.set_rangeSpeed(10,15); // CONVERT BACK TO THIS FOR PUPIL

	randomMotion.set_rangeSpeed(25, 30);
	randomMotion2.set_rangeSpeed(25, 30);

	//randomMotion.set_rangeSpeed(1*100,1.1*100); // USE OR REVERT TO THIS
	//randomMotion.set_rangeSpeed(3.3*100,3.5*100);

	osg::Matrixd mat;
	osg::Matrixd mat2;

#ifndef RECORD_PUPIL
	new_target_ball = randomMotion.sol_pos();
	//randomMotion2.sol_pos();
#else
	new_target_ball = randomMotion.sol_pos_on_cue(pupil_adjusted);
	if (pupil_adjusted) {
		pupil_adjusted = false;
	}
#endif

	if (new_target_ball) {
		int randVal = rand();
		vis_target_ec = (0.99 - 0.11) * rand() / ((float)RAND_MAX) + 0.11;
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
		sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
		sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	}

	//osg::Vec3 v(randomMotion.get_curPos());
	osg::Vec3 v(baseWorldPos.x() - 100, baseWorldPos.y() - 600, baseWorldPos.z() + 160);
	osg::Quat q(atan2(v.x(), v.z()), osg::Y_AXIS, -atan2(v.y() - 8, sqrt(v.x() * v.x() + v.z() * v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);

#ifdef SECOND_VISUAL_TARGET
	//osg::Vec3 v2(randomMotion2.get_curPos());
	osg::Vec3 v2(baseWorldPos.x() - 15, baseWorldPos.y() - 200, baseWorldPos.z());
	osg::Quat q2(atan2(v2.x(), v2.z()), osg::Y_AXIS, -atan2(v2.y() - 8, sqrt(v2.x() * v2.x() + v2.z() * v2.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat2.makeRotate(q2);
	mat2.setTrans(v2);

	visualTarget2.get()->setMatrix(mat2);
#endif
}

void move_visual_target_foveation_training(osg::MatrixTransform* trans_visual_target, double time)
{
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos = base->frame().trn();

	randomMotion.set_bpyramid(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 350, baseWorldPos.z()), osg::Vec3(baseWorldPos.x() - 500, baseWorldPos.y() - 1250, baseWorldPos.z() - 500));
	randomMotion.set_mode(RandomMotion::INSTANT);
	randomMotion.set_rangeSpeed(1, 1);
	//randomMotion.set_rangeSpeed(5,5);

	osg::Matrixd mat;
	new_target_ball = randomMotion.sol_pos();

	if (new_target_ball) {
		int randVal = rand();
		vis_target_ec = (0.99 - 0.11) * rand() / ((float)RAND_MAX) + 0.11;
		osg::ref_ptr<osg::Material> mater2 = new osg::Material;
		mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

		sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
		sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	}

	osg::Vec3 v(randomMotion.get_curPos());
	osg::Quat q(atan2(v.x(), v.z()), osg::Y_AXIS, -atan2(v.y() - 8, sqrt(v.x() * v.x() + v.z() * v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);
}

void move_visual_target_lens_training_new(osg::MatrixTransform* trans_visual_target, double time)
{
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos = base->frame().trn();

	randomMotion.set_bpyramid(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 350, baseWorldPos.z()), osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 1250, baseWorldPos.z()));
	randomMotion.set_mode(RandomMotion::INSTANT);
	randomMotion.set_rangeSpeed(1, 1);

	osg::Matrixd mat;
	new_target_ball = randomMotion.sol_pos();

	if (new_target_ball) {
		int randVal = rand();
		vis_target_ec = (0.99 - 0.45) * rand() / ((float)RAND_MAX) + 0.45;
		osg::ref_ptr<osg::Material> mater2 = new osg::Material;
		mater2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		visualTarget.get()->getChild(0)->getStateSet()->setAttributeAndModes(mater2.get(), osg::StateAttribute::ON);

		sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
		sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
		sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	}

	osg::Vec3 v(randomMotion.get_curPos());
	osg::Quat q(atan2(v.x(), v.z()), osg::Y_AXIS, -atan2(v.y() - 8, sqrt(v.x() * v.x() + v.z() * v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);
}

void move_visual_target_lens_online_fixed_positions(osg::MatrixTransform* trans_visual_target, double time)
{
	gLink* base = SysLEye->root();
	base = SysLEye->findLink("Eyeball"); //this is temporary, the below one was the original, but did not work. probably need initialization.
	gVec3 baseWorldPos = base->frame().trn();

	static int cnt = 0;
	int tot_count = 50;

	vector<osg::Vec3> targetPoints;
	vector<double> targetIntensity;

	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 350, baseWorldPos.z())); // keep
	targetIntensity.push_back(0.7);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 674, baseWorldPos.z())); // keep
	targetIntensity.push_back(0.8);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 800, baseWorldPos.z())); // throw
	targetIntensity.push_back(0.6);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 890, baseWorldPos.z())); // throw
	targetIntensity.push_back(0.65);
	targetPoints.push_back(osg::Vec3(baseWorldPos.x(), baseWorldPos.y() - 500, baseWorldPos.z())); // throw
	targetIntensity.push_back(0.75);

	static int targetIndex = 0;
	static int current_index = 0;

#ifdef LENS_VIDEO
	/*if(targetIndex == 3 || targetPoints.size() == 1) {
		holdBallPosition = true;
	}*/
	if (holdBallPosition) {
		targetIndex = 3;
	}
	else if (cnt < tot_count) {

#else
	if (cnt < tot_count) {
#endif


		//if(cnt < tot_count){
		new_target_ball = false;
		cnt++;
	}
	else {
		//targetIndex = min(targetIndex + 1, targetPoints.size() - 1);

		if (!holdBallPosition || targetMoveOverride) {
			targetIndex = (targetIndex + 1) % (targetPoints.size() - 1);
		}
		new_target_ball = true;
		if (tot_count == 1) {
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
	sceneView->setClearColor(osg::Vec4(0.4 * vis_target_ec, 0.4 * vis_target_ec, 0.4 * vis_target_ec, 1.0));
	sceneView->getLight()->setDiffuse(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setSpecular(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
	sceneView->getLight()->setAmbient(osg::Vec4(vis_target_ec, vis_target_ec, vis_target_ec, 1.0));
#endif

	osg::Matrixd mat;
	osg::Vec3 v(target);
	osg::Quat q(atan2(v.x(), v.z()), osg::Y_AXIS, -atan2(v.y() - 8, sqrt(v.x() * v.x() + v.z() * v.z())), osg::X_AXIS,
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
	gVec3 baseWorldPos = base->frame().trn();

	if (counter < 2) {
		pos.set(baseWorldPos.x(), baseWorldPos.y() - 500, baseWorldPos.z());
	}
	else {
		//pos.set(baseWorldPos.x()+120, baseWorldPos.y()-500, baseWorldPos.z()-100);
		pos.set(baseWorldPos.x() - 150.0, baseWorldPos.y() - 500, baseWorldPos.z() + 40);
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
	osg::Quat q(atan2(v.x(), v.z()), osg::Y_AXIS, -atan2(v.y() - 8, sqrt(v.x() * v.x() + v.z() * v.z())), osg::X_AXIS,
		0, osg::Z_AXIS);
	mat.makeRotate(q);
	mat.setTrans(v);
	trans_visual_target->setMatrix(mat);
	return bHit;
}