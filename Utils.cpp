#include "globals.h"
#include "Utils.h"

#include "gkd/gkd_multibody_system.h"

gVec3 getcurrent_ori_leye();
void adjust_pupil_sphincter_muscles();
void compute_pupil_size();
void adjust_lens_cilliary_muscles();
void compute_lens_size();

double phi_vec_conversion(gVec3 v);
double theta_vec_conversion(gVec3 v);

void render(std::vector<Sphere_local>& Sphere_locals, Vec3f_local ray_orig, Vec3f_local ray_dir,
	Vec3f_local* returnedImage, int side);
void render_ultimate(std::vector<Sphere_local>& Sphere_locals, Vec3f_local ray_orig, Vec3f_local ray_dir, 
	Vec3f_local* returnedImagePupil, Vec3f_local* returnedImageLens, Vec3f_local* returnedImageFoveation, int side);

static gReal randomNumber(gReal low, gReal high)
{
	return low + ((high - low) * rand()) / RAND_MAX;
}

static gReal randomRange(gReal low, gReal up)
{
	return rand() * ((up - low) / RAND_MAX) + low;
}

void raytracer(vector<Vec3f_local> spos, float radius, Vec3f_local ray_orig, Vec3f_local ray_di, Vec3f_local* returnedImagePupil, Vec3f_local* returnedImageLens, Vec3f_local* returnedImageFoveation, int side)
{
	std::vector<Sphere_local> Sphere_locals;

	//Vec3f_local(0.00, 0.00, 0.00)
	for (std::vector<Vec3f_local>::iterator it = spos.begin(); it != spos.end(); ++it) {
		Sphere_locals.push_back(Sphere_local(*it, radius, Vec3f_local(vis_target_ec), 0, 0.0, Vec3f_local(vis_target_ec)));
	}
	//Sphere_locals.push_back(Sphere_local( pos, radius, Vec3f_local(vis_target_ec), 0, 0.0, Vec3f_local(vis_target_ec)));

#ifdef OLD_RETINA // If want faster speed
	render(Sphere_locals, ray_orig, ray_di, returnedImage, side);
#else
	//render(Sphere_locals, ray_orig, ray_di, returnedImageFoveation, side);
	render_ultimate(Sphere_locals, ray_orig, ray_di, returnedImagePupil, returnedImageLens, returnedImageFoveation, side);

#ifdef RECORD_PUPIL
	if (start_recording_pupil) {
		static int iteration = 0;
		static double prev_activation = 0.0;
		double ideal_utility = 25830.0;

		double currentAct = SysLEye->muscle(6)->actLevel();
		gVec3 ballpos = visualTarget.get()->getMatrix().getTrans().ptr();
		if (!final_record_iteration_pupil) {
			for (int i = 0; i < RAYTRACE_WIDTH * RAYTRACE_HEIGHT - 1; i++) {
				finputpupil << returnedImagePupil[i].x << ",";
			}
			finputpupil << returnedImagePupil[RAYTRACE_WIDTH * RAYTRACE_HEIGHT - 1].x << endl;
			finputpupilact << currentAct << endl;

			foutputpupilballpos << ballpos.x() << "," << ballpos.y() << "," << ballpos.z() << "," << vis_target_ec << endl;

			foutputpupilrandomlensact << SysLEye->muscle(14)->actLevel() << endl;
			foutputpupilutility << pupil_utility << "," << pupil_utility - ideal_utility << endl;

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
		/*
		static int iteration = 0;
		static double prev_activation = 0.0;

		double currentAct = SysLEye->muscle(14)->actLevel();
		gVec3 ballpos = visualTarget.get()->getMatrix().getTrans().ptr();
		if(!final_record_iteration_lens && lens_error == lens_error) {
			for(int i = 0; i< RAYTRACE_WIDTH*RAYTRACE_HEIGHT - 1; i++) {
				finputlens << returnedImageLens[i].x << ",";
			}
			finputlens << returnedImageLens[(RAYTRACE_WIDTH * RAYTRACE_HEIGHT) - 1].x << endl;

			// finputlens << currentAct << endl;
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

		prev_activation = SysLEye->muscle(14)->actLevel();
		*/

		gVec3 ballpos = visualTarget.get()->getMatrix().getTrans().ptr();
		if (record_lens_data) {
			for (int i = 0; i < RAYTRACE_WIDTH * RAYTRACE_HEIGHT; i++) {
				finputlens << returnedImageLens[i].x << ",";
			}
			finputlens << returnedImageLens[(RAYTRACE_WIDTH * RAYTRACE_HEIGHT) - 1].x << endl;
			finputlensact << SysLEye->muscle(14)->actLevel() << endl;
			foutputlens << lens_error << endl;
			cout << "LENS " << lens_error << endl;

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
		gVec3 localTargetPos = targetWorldPos - base->frame().trn();

		gVec3 ballpos = visualTarget.get()->getMatrix().getTrans().ptr();

		double theta = theta_vec_conversion(localTargetPos);
		double phi = -phi_vec_conversion(localTargetPos);
		for (int i = 0; i < (RAYTRACE_WIDTH * RAYTRACE_HEIGHT) - 1; i++) {
			finputperception << returnedImageFoveation[i].x << ",";
		}
		finputperception << returnedImageFoveation[(RAYTRACE_WIDTH * RAYTRACE_HEIGHT) - 1].x << endl;

		/*
		for (int i = 0; i < (RAYTRACE_WIDTH * RAYTRACE_HEIGHT); i++) {
			finputperception << returnedImageFoveation[i].x << ",";
		}
		for (int i = 0; i < (RAYTRACE_WIDTH * RAYTRACE_HEIGHT); i++) {
			finputperception << returnedImageFoveation[i].y << ",";
		}
		for (int i = 0; i < (RAYTRACE_WIDTH * RAYTRACE_HEIGHT) - 1; i++) {
			finputperception << returnedImageFoveation[i].z << ",";
		}
		finputperception << returnedImageFoveation[(RAYTRACE_WIDTH * RAYTRACE_HEIGHT) - 1].z << endl;
		*/

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
	const Vec3f_local& rayorig,
	Vec3f_local& raydir,
	std::vector<Sphere_local>& Sphere_locals,
	const int& depth,
	int filterOption = 0) // filter options: 0 = no filter, 1 = binary filter, 2 = inverse dot filter
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
		switch (filterOption) {
		case 0:
			retVal = bgCol;
			break;
		default:
			retVal = Vec3f_local(0, 0, 0);
			break;
		}
		//return bgCol;
		return retVal;
	}

	Vec3f_local phit = 0;
	Sphere_local* Sl = NULL;
	if (num_intersections > 1) { // multiple spheres were hit- take only closest one
		double min_dist = 10000000000000;
		int closestIndex = 0;
		for (int i = 0; i < tnears.size(); i++) {
			Vec3f_local tn = tnears.at(i);
			double dist = (tnears.at(i) - rayorig).length();
			if (dist < min_dist) {
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

	switch (filterOption) {
	case 1:
		retVal = Vec3f_local(1.0, 1.0, 1.0);
		break;
	case 2:
		retVal = Vec3f_local(1.0, 1.0, 1.0) * 10.0 * exp(-facingratio * 10.0);
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


void render(std::vector<Sphere_local>& Sphere_locals, Vec3f_local ray_orig, Vec3f_local ray_dir, Vec3f_local* returnedImage, int side)
{
	//unsigned width = wind[MAIN_WINDOW].width, height = wind[MAIN_WINDOW].height;
	unsigned width = RAYTRACE_WIDTH, height = RAYTRACE_HEIGHT;
	Vec3f_local* image = new Vec3f_local[width * height], * pixel = image;
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
	if (side == 0)
	{

		osg::Vec3 osg_eye_center(ray_orig.x, ray_orig.y, ray_orig.z);

		double xangle, yangle;
		xangle = ray_dir.x;
		yangle = ray_dir.z;

		double phi = xangle;
		double theta = yangle;


		osg::Vec3 eye_direction_vec(0, -1, 0);
		osg::Quat q2(phi, osg::X_AXIS, 0, osg::Y_AXIS, theta, osg::Z_AXIS);
		osg::Vec3 osg_eye_temp2 = q2 * (eye_direction_vec);

#ifdef SHOW_RAYS
		//draw ray trace lines
		osg::Vec3Array* vertices_eye = new osg::Vec3Array;
		osg::Vec4Array* colors_eye = new osg::Vec4Array;
		vertices_eye->setName("VertexRayTrace");
#endif

		osg::Vec3 rotated_eye_dir;

		double ray_width = RAYTRACE_WIDTH;
		double maxRadius = 2.0 / 3.0 * std::exp(ray_width / 5.0);
		//5

		//#pragma omp parallel for num_threads(8) 
		for (int rho = 0; rho < RAYTRACE_WIDTH; rho++) {
			for (unsigned thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT * 4; thetaLogPolar += 4, ++pixel) {

				double logPolarX = 2.0 * std::exp(double(rho / 5.0)) * cos(double(thetaLogPolar * DEG_TO_RAD));
				double logPolarY = 2.0 * std::exp(double(rho / 5.0)) * sin(double(thetaLogPolar * DEG_TO_RAD));

				float yy = (1 * ((logPolarX) / maxRadius)) * fov * D2R;
				float xx = (1 * ((logPolarY) / maxRadius)) * fov * D2R;

				osg::Vec3 eye_direction_vec3(0, 1, 0);
				osg::Quat q3(xx, osg::X_AXIS, 0, osg::Y_AXIS, yy, osg::Z_AXIS);
				osg::Vec3 osg_eye_temp3 = q3 * (eye_direction_vec3);

				osg_eye_temp3.normalize();
				osg_eye_temp2.normalize();

				osg::Vec3 raytrace_orig(osg_eye_center.x() + logPolarX / maxRadius * EYE_SIZE / 2, osg_eye_center.y() + EYE_SIZE / 2, osg_eye_center.z() + logPolarY / maxRadius * EYE_SIZE / 2);
				Vec3f_local raytrace_orig_local(raytrace_orig.x(), raytrace_orig.y(), raytrace_orig.z());
				osg::Vec3 pin_hole_pos = osg_eye_center + osg::Vec3(0, -1, 0) * EYE_SIZE / 2;

				osg::Vec3 raydir = pin_hole_pos - raytrace_orig;
				raydir = q2 * raydir;
				raydir.normalize();
				Vec3f_local raydir_local(raydir.x(), raydir.y(), raydir.z());
				*pixel = trace(raytrace_orig_local, raydir_local, Sphere_locals, 0);

			}
		}

#ifdef SHOW_RAYS
		osg::Vec3 up(0, 0, 1);
		osg::Vec3 pCenter(SysLEye->pupil.center.x, SysLEye->pupil.center.y - 2, SysLEye->pupil.center.z);
		double rad = SysLEye->pupil.radius;

		for (int i = 0; i < 4; i++)
		{
			osg::Quat q(0, osg::X_AXIS, 45 * i * gDTR, osg::Y_AXIS, 0, osg::Z_AXIS);
			osg::Vec3 fin_dir = q * (up);
			osg::Vec3 p1 = pCenter + fin_dir * rad;
			osg::Vec3 p2 = pCenter - fin_dir * rad;

			vertices_eye->push_back(p1);
			vertices_eye->push_back(p2);
			colors_eye->push_back(osg::Vec4(255 / 255.0, 0 / 255.0, 0 / 255.0, 1));
			colors_eye->push_back(osg::Vec4(255 / 255.0, 0 / 255.0, 0 / 255.0, 1));
		}

		vertices_eye->push_back(osg_eye_center + osg_eye_temp2 * 3000);
		vertices_eye->push_back(osg_eye_center - osg_eye_temp2 * 3000);
		colors_eye->push_back(osg::Vec4(206 / 255.0, 206 / 255.0, 206 / 255.0, 1));
		colors_eye->push_back(osg::Vec4(206 / 255.0, 206 / 255.0, 206 / 255.0, 1));


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

		if (raytraceLineGroup->getNumChildren() > 0)
			raytraceLineGroup->removeChild(0, 1);
		raytraceLineGroup->addChild(pat_raytrace_lines);
#endif
	}
	else {
		osg::Vec3 osg_eye_center(ray_orig.x, ray_orig.y, ray_orig.z);


		double cxr, cyr;

		double xangle, yangle;
		xangle = ray_dir.x;
		yangle = ray_dir.z;

		double theta = xangle;
		double phi = yangle;


		//osg::Vec3 osg_eye_temp2(cos(theta)*sin(phi), -cos(theta)*cos(phi), sin(theta));
		osg::Vec3 eye_direction_vec(0, -1, 0);
		osg::Quat q2(theta, osg::X_AXIS, 0, osg::Y_AXIS, phi, osg::Z_AXIS);
		osg::Vec3 osg_eye_temp2 = q2 * (eye_direction_vec);

		osg::Vec3 rotated_eye_dir;

		//double pho = character[0]->head->eyes->get_eye_radius_Z();
		double ray_width = RAYTRACE_WIDTH;
		double maxRadius = std::exp(ray_width);
		//5

		//#pragma omp parallel for num_threads(8) 
		for (int rho = 0; rho < RAYTRACE_WIDTH; rho++) {
			for (unsigned thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT * 4; thetaLogPolar += 4, ++pixel) {

				double logPolarX = std::exp(double(rho)) * cos(double(thetaLogPolar * DEG_TO_RAD));
				double logPolarY = std::exp(double(rho)) * sin(double(thetaLogPolar * DEG_TO_RAD));

				float yy = (1 * ((logPolarX) / maxRadius)) * fov * D2R;
				float xx = (1 * ((logPolarY) / maxRadius)) * fov * D2R;

				osg::Vec3 eye_direction_vec3(0, 1, 0);
				osg::Quat q3(theta + xx, osg::X_AXIS, 0, osg::Y_AXIS, phi + yy, osg::Z_AXIS);
				osg::Vec3 osg_eye_temp3 = q3 * (eye_direction_vec3);

				osg_eye_temp3.normalize();
				osg_eye_temp2.normalize();
				osg::Vec3 raytrace_orig(osg_eye_center.x() + logPolarX / maxRadius * EYE_SIZE / 2, osg_eye_center.y() + EYE_SIZE / 2, osg_eye_center.z() + logPolarY / maxRadius * EYE_SIZE / 2);
				osg::Vec3 pin_hole_pos = osg_eye_center + osg::Vec3(0, -1, 0) * EYE_SIZE / 2;


				Vec3f_local raytrace_orig_local(raytrace_orig.x(), raytrace_orig.y(), raytrace_orig.z());
				osg::Vec3 raydir = pin_hole_pos - raytrace_orig;
				raydir = q2 * raydir;
				raydir.normalize();
				Vec3f_local raydir_local(raydir.x(), raydir.y(), raydir.z());

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
	delete[] image;
}

void render_ultimate(std::vector<Sphere_local>& Sphere_locals, Vec3f_local ray_orig, Vec3f_local ray_dir, Vec3f_local* returnedImagePupil, Vec3f_local* returnedImageLens, Vec3f_local* returnedImageFoveation, int side)
{
	unsigned width = RAYTRACE_WIDTH, height = RAYTRACE_HEIGHT;

	Vec3f_local* imagePupil = new Vec3f_local[width * height], * pixelPupil = imagePupil;
	Vec3f_local* imageLens = new Vec3f_local[width * height], * pixelLens = imageLens;
	Vec3f_local* imageFoveation = new Vec3f_local[width * height], * pixelFoveation = imageFoveation;

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
	if (side == 0)
	{
		osg::Vec3 osg_eye_center(ray_orig.x, ray_orig.y, ray_orig.z);

		gVec3 rot = getcurrent_ori_leye();
		osg::Quat eye_rot(rot.x() * gDTR, osg::X_AXIS, 0, osg::Y_AXIS, rot.z() * gDTR, osg::Z_AXIS);

		double xangle, yangle, zangle;
		xangle = ray_dir.x;
		xangle = ray_dir.y;
		zangle = ray_dir.z;

		double theta = xangle;
		double yrot = yangle;
		double phi = zangle;

		osg::Vec3 up(0, 0, 1);

		osg::Vec3 forward(0, 1, 0);
		osg::Vec3 eye_direction_vec = -forward;
		//osg::Quat q2(theta, osg::X_AXIS, yrot, osg::Y_AXIS,phi, osg::Z_AXIS);

		osg::Vec3 rotated_eye_dir = eye_rot * (eye_direction_vec);

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
		osg::Vec3 pupil_top_orig = osg::Vec3(SysLEye->pupil.center.x, SysLEye->pupil.center.y, SysLEye->pupil.center.z) + pupilUp * SysLEye->pupil.radius;
		osg::Vec3 pupil_forward_vec = eye_rot * forward;

		Vec3f_local pupil_top_orig_local(pupil_top_orig.x(), pupil_top_orig.y(), pupil_top_orig.z());
		Vec3f_local pupil_forward_vec_local(pupil_forward_vec.x(), pupil_forward_vec.y(), pupil_forward_vec.z());
		Vec3f_local near_int_lens;
		Vec3f_local far_int_lens;

		if (!SysLEye->lens.intersect_reverse(pupil_top_orig_local, pupil_forward_vec_local, Vec3f_local(0), far_int_lens, Vec3f_local(0))) { // if no intersection, pupil is too wide and lens is too fat, so just use lens radius instead
#ifdef UNIFORM_LENS
			pupil_top_orig = osg::Vec3(SysLEye->pupil.center.x, SysLEye->pupil.center.y, SysLEye->pupil.center.z) + pupilUp * SysLEye->lens.radius;
#else
			pupil_top_orig = osg::Vec3(SysLEye->pupil.center.x, SysLEye->pupil.center.y, SysLEye->pupil.center.z) + pupilUp * SysLEye->lens.smallRadius;
#endif
			pupil_top_orig_local = Vec3f_local(pupil_top_orig.x(), pupil_top_orig.y(), pupil_top_orig.z());
			SysLEye->lens.intersect_reverse(pupil_top_orig_local, pupil_forward_vec_local, Vec3f_local(0), far_int_lens, Vec3f_local(0));
		}

		Vec3f_local lens_sphere_v = (SysLEye->lens.sphere2.center + pupil_forward_vec_local * SysLEye->lens.sphere2.radius) - SysLEye->lens.sphere2.center;
		Vec3f_local lens_pupil_v = SysLEye->pupil.center - SysLEye->lens.sphere2.center;
		Vec3f_local hypotenuse_vec = far_int_lens - SysLEye->pupil.center;
		Vec3f_local lower_vec = lens_sphere_v - lens_pupil_v;

		double pupil_ang = acos(hypotenuse_vec.dot(lower_vec) / (lower_vec.length() * hypotenuse_vec.length()));
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
		osg::Vec3 lens_far_center(lens_far_sphere_center.x, lens_far_sphere_center.y, lens_far_sphere_center.z);

		std::vector<osg::Vec3> lens_ring_points;
		int total_points = 0;

		//double dphi = 1.0 * gDTR; // TWEAK THIS
		double dphi = lens_arc_increment / SysLEye->lens.sphere2.radius; // CONSTANT ARC LENGTH
		for (double rotation_angle = 0.0; rotation_angle < ang; rotation_angle += dphi)
		{
			osg::Quat qRup(rotation_angle, osg::X_AXIS, 0, osg::Y_AXIS, 0, osg::Z_AXIS);
			osg::Vec3 result_vec = qRup * eye_rot * forward;

			if (rotation_angle == 0.0) {
				osg::Vec3 lens_point = lens_far_center + result_vec * lens_sphere_radius;
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
				double point_increment = 360.0 / (double)num_points;
				total_points += num_points;

				for (unsigned ltheta = 0; ltheta < num_points; ltheta++) {
					osg::Quat qRing(0, osg::X_AXIS, point_increment * ltheta * gDTR, osg::Y_AXIS, 0, osg::Z_AXIS);
					osg::Vec3 fin_dir = qRup * qRing * eye_rot * forward; // for some reason this rotates in the OPPOSITE order of what I expect
					fin_dir.normalize();
					osg::Vec3 lens_point = lens_far_center + fin_dir * lens_sphere_radius;
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
		double maxRadius = 2.0 / 3.0 * std::exp(ray_width / divisor);
		double gaussian_mean = 0;
		double gaussian_spread = (double)RAYTRACE_WIDTH; // Peripheral falls 1 stdev / spread weight of center
		//5
		//#pragma omp parallel for num_threads(8)
		reverseRayBuffer.clear();

		for (int rho = 0; rho < RAYTRACE_WIDTH; rho++) {
			// Change_by_honglin 3: the increment is 1 instead of 4
			for (unsigned int thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT * 1; thetaLogPolar += 1, ++pixelPupil, ++pixelLens, ++pixelFoveation) {
				// Change_by_honglin 4: generate new noise.txt and change the increment from 4 to 1
				double logPolarX = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0 / 3.0 * std::exp(double(rho / divisor) + retina_distribution_noise[rho + RAYTRACE_WIDTH * thetaLogPolar / 1]) * cos(double(thetaLogPolar * DEG_TO_RAD + retina_distribution_noise[rho + RAYTRACE_WIDTH * thetaLogPolar / 1]));
				double logPolarY = (thetaLogPolar == 0 && rho == 0) ? 0 : 2.0 / 3.0 * std::exp(double(rho / divisor) + retina_distribution_noise[rho + RAYTRACE_WIDTH * thetaLogPolar / 1]) * sin(double(thetaLogPolar * DEG_TO_RAD + retina_distribution_noise[rho + RAYTRACE_WIDTH * thetaLogPolar / 1]));

				float yy = (1 * ((logPolarX) / maxRadius)) * spread * D2R;
				float xx = (1 * ((logPolarY) / maxRadius)) * spread * D2R;

				//if (fdistoutput.is_open())
				//	fdistoutput << xx << ',' << yy << endl;

				//osg::Vec3 eye_direction_vec3(0,1,0);
				osg::Vec3 eye_direction_vec3 = -rotated_eye_dir;
				osg::Quat q3(xx, osg::X_AXIS, 0, osg::Y_AXIS, yy, osg::Z_AXIS);
				//osg::Vec3 osg_eye_temp3 = q3*(eye_direction_vec3);
				osg::Vec3 osg_eye_temp3 = eye_rot * q3 * forward;

				osg_eye_temp3.normalize();
				rotated_eye_dir.normalize();

				//osg::Vec3 raytrace_orig(osg_eye_center.x()+logPolarX/maxRadius*EYE_SIZE/2, osg_eye_center.y()+EYE_SIZE/2, osg_eye_center.z()+logPolarY/maxRadius*EYE_SIZE/2);
				osg::Vec3 raytrace_orig = osg_eye_center + osg_eye_temp3 * EYE_SIZE;
				Vec3f_local raytrace_orig_local(raytrace_orig.x(), raytrace_orig.y(), raytrace_orig.z());

#ifdef SHOW_RAYS
				//draw ray trace lines
				vertices_eye->push_back(raytrace_orig - osg_eye_temp3 * 0.1);
				vertices_eye->push_back(raytrace_orig + osg_eye_temp3 * 0.1);
				colors_eye->push_back(osg::Vec4(0 / 255.0, 0 / 255.0, 0 / 255.0, 1));
				colors_eye->push_back(osg::Vec4(0 / 255.0, 0 / 255.0, 0 / 255.0, 1));
#endif

				Vec3f_local cornea_near, cornea_far, cornea_refracted, pupil_near, pupil_far, pupil_refracted, lens_near, lens_far, lens_refracted;
				Vec3f_local raydir_local_sum_pupil(0);
				Vec3f_local raydir_local_sum_lens(0);
				Vec3f_local raydir_local_sum_foveation(0);

				//int total_points = 0;
				//for(std::vector<osg::Vec3>::iterator it = lens_ring_points.begin(); it != lens_ring_points.end(); ++it) {
				for (int i = 0; i < lens_ring_points.size(); i++) {
					//osg::Vec3 raydir = (*it) - raytrace_orig;
					osg::Vec3 raydir = lens_ring_points.at(i) - raytrace_orig;
					raydir.normalize();
					eye_direction_vec.normalize();
					Vec3f_local raydir_local(raydir.x(), raydir.y(), raydir.z());
					//Vec3f_local eye_rot_vec(osg_eye_temp2.x(),osg_eye_temp2.y(),osg_eye_temp2.z());
					Vec3f_local eye_rot_vec(eye_direction_vec.x(), eye_direction_vec.y(), eye_direction_vec.z());

					if (SysLEye->lens.intersect(raytrace_orig_local, raydir_local, lens_near, lens_far, lens_refracted)) {

						if (SysLEye->pupil.intersect(lens_far, lens_refracted, pupil_near, pupil_far, pupil_refracted)) {
							if (SysLEye->cornea.intersect(pupil_near, pupil_refracted, cornea_near, cornea_far, cornea_refracted)) {
								cornea_refracted.normalize();
								raydir_local_sum_pupil += trace(cornea_far, cornea_refracted, Sphere_locals, 0, 0);
								raydir_local_sum_lens += trace(cornea_far, cornea_refracted, Sphere_locals, 0, 2);
								raydir_local_sum_foveation += trace(cornea_far, cornea_refracted, Sphere_locals, 0, 1);
#ifdef SHOW_RAYS
								static int randomIndex1 = (int)randomNumber(0, lens_ring_points.size());
								static int randomIndex2 = (int)randomNumber(0, lens_ring_points.size());
								static int randomIndex3 = (int)randomNumber(0, lens_ring_points.size());
								static int randomIndex4 = (int)randomNumber(0, lens_ring_points.size());
								//static int randomIndex5 = (int) randomNumber(0,lens_ring_points.size());
								if (/*rho == photoreceptor_to_trace_rho && thetaLogPolar == photoreceptor_to_trace_theta*/
									(i == randomIndex1 || i == randomIndex2 || i == randomIndex3 || i == randomIndex4 /*|| i == randomIndex5*/) &&
									show_reverse_trace_rays) {
									osg::Vec3 phit_lens_near_v3(lens_near.x, lens_near.y, lens_near.z);
									osg::Vec3 phit_lens_far_v3(lens_far.x, lens_far.y, lens_far.z);
									osg::Vec3 phit_pupil_near_v3(pupil_near.x, pupil_near.y, pupil_near.z);
									osg::Vec3 phit_pupil_nearref_v3(pupil_refracted.x, pupil_refracted.y, pupil_refracted.z);
									osg::Vec3 phit_cornea_near_v3(cornea_near.x, cornea_near.y, cornea_near.z);
									osg::Vec3 phit_cornea_far_v3(cornea_far.x, cornea_far.y, cornea_far.z);
									osg::Vec3 cornea_final_refracted_v3(cornea_refracted.x, cornea_refracted.y, cornea_refracted.z);

									//draw ray trace lines
									vertices_eye->push_back(raytrace_orig);
									vertices_eye->push_back(phit_lens_near_v3);
									//vertices_eye->push_back(phit_lens_near_v3+osg::Vec3(raydir_local.x,raydir_local.y,raydir_local.z)*1000);
									colors_eye->push_back(osg::Vec4(15 / 255.0, 206 / 255.0, 20 / 255.0, 1));
									colors_eye->push_back(osg::Vec4(15 / 255.0, 206 / 255.0, 20 / 255.0, 1));
									//colors_eye->push_back(osg::Vec4(15/255.0,206/255.0,20/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(15/255.0,206/255.0,20/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));

									vertices_eye->push_back(phit_lens_near_v3);
									vertices_eye->push_back(phit_lens_far_v3);
									//osg::Vec3 templl = phit_lens_far_v3-phit_lens_near_v3;
									//templl.normalize();
									//vertices_eye->push_back(phit_lens_near_v3 + templl*200);
									colors_eye->push_back(osg::Vec4(15 / 255.0, 20 / 255.0, 206 / 255.0, 1));
									colors_eye->push_back(osg::Vec4(15 / 255.0, 20 / 255.0, 206 / 255.0, 1));
									//colors_eye->push_back(osg::Vec4(15/255.0,20/255.0,206/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(15/255.0,20/255.0,206/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));

									vertices_eye->push_back(phit_lens_far_v3);
									vertices_eye->push_back(phit_pupil_near_v3);
									//osg::Vec3 templp = phit_pupil_near_v3-phit_lens_far_v3;
									//templp.normalize();
									//vertices_eye->push_back(phit_lens_far_v3 + templp*200);
									colors_eye->push_back(osg::Vec4(206 / 255.0, 20 / 255.0, 15 / 255.0, 1));
									colors_eye->push_back(osg::Vec4(206 / 255.0, 20 / 255.0, 15 / 255.0, 1));
									//colors_eye->push_back(osg::Vec4(206/255.0,20/255.0,15/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(206/255.0,20/255.0,15/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));

									vertices_eye->push_back(phit_pupil_near_v3);
									vertices_eye->push_back(phit_cornea_near_v3);
									colors_eye->push_back(osg::Vec4(206 / 255.0, 206 / 255.0, 15 / 255.0, 1));
									colors_eye->push_back(osg::Vec4(206 / 255.0, 206 / 255.0, 15 / 255.0, 1));
									//colors_eye->push_back(osg::Vec4(206/255.0,206/255.0,15/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(206/255.0,206/255.0,15/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));

									vertices_eye->push_back(phit_cornea_near_v3);
									vertices_eye->push_back(phit_cornea_far_v3);
									colors_eye->push_back(osg::Vec4(206 / 255.0, 20 / 255.0, 206 / 255.0, 1));
									colors_eye->push_back(osg::Vec4(206 / 255.0, 20 / 255.0, 206 / 255.0, 1));
									//colors_eye->push_back(osg::Vec4(206/255.0,20/255.0,206/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(206/255.0,20/255.0,206/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));

									osg::Vec3 endpoint = phit_cornea_far_v3 + cornea_final_refracted_v3 * 1000;

									vertices_eye->push_back(phit_cornea_far_v3);
									vertices_eye->push_back(endpoint);
									colors_eye->push_back(osg::Vec4(20 / 255.0, 206 / 255.0, 206 / 255.0, 0.1));
									colors_eye->push_back(osg::Vec4(20 / 255.0, 206 / 255.0, 206 / 255.0, 0.1));
									//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));
									//colors_eye->push_back(osg::Vec4(20/255.0,206/255.0,206/255.0,reverse_rays_alpha_channel));

									double p1[6] = { raytrace_orig.x(), raytrace_orig.y(), raytrace_orig.z(), phit_lens_near_v3.x(), phit_lens_near_v3.y(), phit_lens_near_v3.z() };
									double p2[6] = { phit_lens_near_v3.x(), phit_lens_near_v3.y(), phit_lens_near_v3.z(), phit_lens_far_v3.x(), phit_lens_far_v3.y(), phit_lens_far_v3.z() };
									double p3[6] = { phit_lens_far_v3.x(), phit_lens_far_v3.y(), phit_lens_far_v3.z(), phit_pupil_near_v3.x(), phit_pupil_near_v3.y(), phit_pupil_near_v3.z() };
									double p4[6] = { phit_pupil_near_v3.x(), phit_pupil_near_v3.y(), phit_pupil_near_v3.z(), phit_cornea_near_v3.x(), phit_cornea_near_v3.y(), phit_cornea_near_v3.z() };
									double p5[6] = { phit_cornea_near_v3.x(), phit_cornea_near_v3.y(), phit_cornea_near_v3.z(), phit_cornea_far_v3.x(), phit_cornea_far_v3.y(), phit_cornea_far_v3.z() };
									double p6[6] = { phit_cornea_far_v3.x(), phit_cornea_far_v3.y(), phit_cornea_far_v3.z(), endpoint.x(), endpoint.y(), endpoint.z() };

									vector<double> p1vec(p1, p1 + sizeof(p1) / sizeof(p1[0]));
									vector<double> p2vec(p2, p2 + sizeof(p2) / sizeof(p2[0]));
									vector<double> p3vec(p3, p3 + sizeof(p3) / sizeof(p3[0]));
									vector<double> p4vec(p4, p4 + sizeof(p4) / sizeof(p4[0]));
									vector<double> p5vec(p5, p5 + sizeof(p5) / sizeof(p5[0]));
									vector<double> p6vec(p6, p6 + sizeof(p6) / sizeof(p6[0]));

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
					min(1.0, raydir_local_sum_pupil.x * scaling * gaussian_weight),
					min(1.0, raydir_local_sum_pupil.y * scaling * gaussian_weight),
					min(1.0, raydir_local_sum_pupil.z * scaling * gaussian_weight)
				);

				Vec3f_local final_val_lens(
					min(1.0, raydir_local_sum_lens.x * scaling * gaussian_weight),
					min(1.0, raydir_local_sum_lens.y * scaling * gaussian_weight),
					min(1.0, raydir_local_sum_lens.z * scaling * gaussian_weight)
				);

				Vec3f_local final_val_foveation(
					min(1.0, raydir_local_sum_foveation.x * scaling * gaussian_weight),
					min(1.0, raydir_local_sum_foveation.y * scaling * gaussian_weight),
					min(1.0, raydir_local_sum_foveation.z * scaling * gaussian_weight)
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
		if (fdistoutput.is_open())
			fdistoutput << "done" << endl;
		fdistoutput.close();
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

		if (raytraceLineGroup->getNumChildren() > 0)
			raytraceLineGroup->removeChild(0, 1);
		raytraceLineGroup->addChild(pat_raytrace_lines);
#endif
	}
	else {
		osg::Vec3 osg_eye_center(ray_orig.x, ray_orig.y, ray_orig.z);

		double xangle, yangle;
		xangle = ray_dir.x;
		yangle = ray_dir.z;

		double theta = xangle;
		double phi = yangle;

		osg::Vec3 eye_direction_vec(0, -1, 0);
		osg::Quat q2(theta, osg::X_AXIS, 0, osg::Y_AXIS, phi, osg::Z_AXIS);
		osg::Vec3 osg_eye_temp2 = q2 * (eye_direction_vec);

#ifdef SHOW_RAYS
		//draw ray trace lines
		osg::Vec3Array* vertices_eye = new osg::Vec3Array;
		osg::Vec4Array* colors_eye = new osg::Vec4Array;
		vertices_eye->setName("VertexRayTrace");
#endif

		osg::Vec3 rotated_eye_dir;

		double ray_width = RAYTRACE_WIDTH;
		double maxRadius = 2.0 / 3.0 * std::exp(ray_width / 5.0 * 40.0 / RAYTRACE_WIDTH);
		//5

		//#pragma omp parallel for num_threads(8) 
		for (int rho = 0; rho < RAYTRACE_WIDTH; rho++) {
			// Change_by_honglin 5: the increment is 1 instead of 4
			for (unsigned thetaLogPolar = 0; thetaLogPolar < RAYTRACE_HEIGHT * 1; thetaLogPolar += 1, ++pixelPupil, ++pixelLens, ++pixelFoveation) {
				// Change_by_honglin 6
				double logPolarX = 2.0 / 3.0 * std::exp(double(rho / 5.0 * 40.0 / RAYTRACE_WIDTH) + retina_distribution_noise[rho + RAYTRACE_WIDTH * thetaLogPolar / 1]) * cos(double(thetaLogPolar * DEG_TO_RAD + retina_distribution_noise[rho + RAYTRACE_WIDTH * thetaLogPolar / 1]));
				double logPolarY = 2.0 / 3.0 * std::exp(double(rho / 5.0 * 40.0 / RAYTRACE_WIDTH) + retina_distribution_noise[rho + RAYTRACE_WIDTH * thetaLogPolar / 1]) * sin(double(thetaLogPolar * DEG_TO_RAD + retina_distribution_noise[rho + RAYTRACE_WIDTH * thetaLogPolar / 1]));

				float yy = (1 * ((logPolarX) / maxRadius)) * spread * D2R;
				float xx = (1 * ((logPolarY) / maxRadius)) * spread * D2R;

				osg::Vec3 eye_direction_vec3(0, 1, 0);
				osg::Quat q3(xx, osg::X_AXIS, 0, osg::Y_AXIS, yy, osg::Z_AXIS);
				osg::Vec3 osg_eye_temp3 = q3 * (eye_direction_vec3);

				osg_eye_temp3.normalize();
				osg_eye_temp2.normalize();

				osg::Vec3 raytrace_orig(osg_eye_center.x() + logPolarX / maxRadius * EYE_SIZE / 2, osg_eye_center.y() + EYE_SIZE / 2, osg_eye_center.z() + logPolarY / maxRadius * EYE_SIZE / 2);
				osg::Vec3 pin_hole_pos = osg_eye_center + osg::Vec3(0, -1, 0) * EYE_SIZE / 2;

				Vec3f_local raytrace_orig_local(raytrace_orig.x(), raytrace_orig.y(), raytrace_orig.z());
				osg::Vec3 raydir = pin_hole_pos - raytrace_orig;
				raydir = q2 * raydir;
				raydir.normalize();
				Vec3f_local raydir_local(raydir.x(), raydir.y(), raydir.z());


				//*pixel = trace(raytrace_orig_local, raydir_local, Sphere_locals, 0);
				*pixelPupil = trace(raytrace_orig_local, raydir_local, Sphere_locals, 0, 0);
				*pixelLens = trace(raytrace_orig_local, raydir_local, Sphere_locals, 0, 2);
				*pixelFoveation = trace(raytrace_orig_local, raydir_local, Sphere_locals, 0, 1);


#ifdef SHOW_RAYS
				//draw ray trace lines
				vertices_eye->push_back(raytrace_orig);
				vertices_eye->push_back(raytrace_orig + raydir * 4000.0);
				colors_eye->push_back(osg::Vec4(135 / 255.0, 206 / 255.0, 250 / 255.0, 1));
				colors_eye->push_back(osg::Vec4(135 / 255.0, 206 / 255.0, 250 / 255.0, 1));
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

		if (raytraceLineGroup->getNumChildren() > 0)
			raytraceLineGroup->removeChild(0, 1);
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

	delete[] imagePupil;
	delete[] imageLens;
	delete[] imageFoveation;
}

void finish()
{
	//if(recordMuscleAct ) finishRecord();

	if (exportMotion) {
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

	if (fpQ)
	{
		fprintf(fpQ, "];");
		fclose(fpQ);
	}

	if (fpPt)
	{
		fprintf(fpPt, "];");
		fclose(fpPt);
	}

	if (fpPh)
	{
		fprintf(fpPh, "];");
		fclose(fpPh);
	}

	if (snRw) {
		delete[] snRw;
		snRw = 0;
	}
	if (snIw) {
		delete[] snIw;
		snIw = 0;
	}
	if (snCw) {
		delete[] snCw;
		snCw = 0;
	}
}

double theta_vec_conversion(gVec3 v) {
	return gMin(90 * gDTR, gMax(-90 * gDTR, atan2(v.x(), -v.y())));
}

double phi_vec_conversion(gVec3 v) {
	return gMin(90 * gDTR, gMax(-90 * gDTR, atan2(v.z(), sqrt(pow(v.x(), 2) + pow(-v.y(), 2)))));
}