#include "EyeUtils.h"
#include "globals.h"
#include "Utils.h"

void raytracer(vector<Vec3f_local> spos, float radius, Vec3f_local ray_orig, Vec3f_local ray_di,
	Vec3f_local* returnedImagePupil, Vec3f_local* returnedImageLens, Vec3f_local* returnedImageFoveation, int side);

double phi_vec_conversion(gVec3 v);
double theta_vec_conversion(gVec3 v);

static gReal randomNumber(gReal low, gReal high)
{
	return low + ((high - low) * rand()) / RAND_MAX;
}

gVec3 getcurrent_ori_leye()
{
	gVec3 f;
	double orit[3] = { 0.0, 0.0, 0.0 };

	gBallLink* l = (gBallLink*)(SysLEye->eyeball);
	f = gVec3::log(l->coord());

	orit[0] += f.x();
	orit[1] += f.y();
	orit[2] += f.z();

	gVec3 cur_ori_in(orit[0], orit[1], orit[2]);
	return cur_ori_in * gRTD;
}

Vec3f_local compute_focal_point()
{
	osg::Vec3 forward(0, 1, 0);
	osg::Vec3 up(0, 0, 1);
	gVec3 targetPos = visualTarget.get()->getMatrix().getTrans().ptr();
	osg::Vec3 tposv3(targetPos.x(), targetPos.y(), targetPos.z());
	Vec3f_local orig_local(tposv3.x(), tposv3.y(), tposv3.z());

	gVec3 rot = getcurrent_ori_leye();
	osg::Quat eye_rot(rot.x() * gDTR, osg::X_AXIS, 0, osg::Y_AXIS, rot.z() * gDTR, osg::Z_AXIS);

	std::vector<Vec3f_local> final_box_int_points;

	Vec3f_local cornea_near, cornea_far, cornea_refracted, pupil_near, pupil_far, pupil_refracted, lens_near, lens_far, lens_refracted;

#if defined(SHOW_RAYS) || defined(SHOW_LENS_SPHERES) || defined(SHOW_CORNEA_SPHERES)
	//draw ray trace lines
	osg::Vec3Array* vertices_ball = new osg::Vec3Array;
	osg::Vec4Array* colors_ball = new osg::Vec4Array;
	osg::Vec3Array* normals_ball = new osg::Vec3Array;
	osg::Vec3 normalRayVertexDir = osg::Vec3(-1, 0, 0);

	osg::Vec3Array* vertices_ball_allrays = new osg::Vec3Array;
	osg::Vec4Array* colors_ball_allrays = new osg::Vec4Array;
	osg::Vec3Array* normals_ball_allrays = new osg::Vec3Array;
	vertices_ball->setName("VertexRayTraceBall");
#endif

	gLink* base = SysLEye->findLink("Eyeball");
	gVec3 baseWorldPos = base->frame().trn();

	gVec3 tP = visualTarget.get()->getMatrix().getTrans().ptr();
	tP += gVec3(0, SPHERE_SIZE, 0);

	osg::Vec3 osg_eye_center(baseWorldPos.x(), baseWorldPos.y(), baseWorldPos.z());
	Vec3f_local eye_center(osg_eye_center.x(), osg_eye_center.y(), osg_eye_center.z());

	osg::Vec3 rot_f_vec = eye_rot * forward;
	Vec3f_local rot_f_vec_local(rot_f_vec.x(), rot_f_vec.y(), rot_f_vec.z());
	rot_f_vec_local.normalize();

	Vec3f_local ball_lens_ray = SysLEye->lens.center - orig_local;
	ball_lens_ray.normalize();

	//Vec3f_local min_b = eye_center + rot_f_vec_local*10000;
	//Vec3f_local max_b = eye_center - rot_f_vec_local*10000;

	gVec3 blr(ball_lens_ray.x, ball_lens_ray.y, ball_lens_ray.z);
	gVec3 upg(up.x(), up.y(), up.z());
	gVec3 left = upg % blr;
	left.normalize();
	gVec3 dirup = blr % left;
	dirup.normalize();

	Vec3f_local leftlocal(left.x(), left.y(), left.z());
	Vec3f_local uplocal(dirup.x(), dirup.y(), dirup.z());

	Vec3f_local min_b = SysLEye->lens.center + leftlocal * 0.1 - uplocal * 0.1;
	Vec3f_local max_b = orig_local + ball_lens_ray * 10000 - leftlocal * 0.1 + uplocal * 0.1;

	Box_local intersection_plane = Box_local(min_b, max_b);

	double xrot;
	Vec3f_local maxz(0);
	Vec3f_local step_max(0);
	//#pragma omp parallel for num_threads(8)
	if (exportMotion && show_forward_trace_rays)
	{
		fprintf(fileRaytracePoints, "%d \n", frame);
	}
	for (int xrot_ind = -900; xrot_ind <= 900; xrot_ind += 1) {
		xrot = xrot_ind / 10.0;
		//cout << xrot << endl;
		for (double zrot = -90.0; zrot <= 90.0; zrot += 0.1) {

			osg::Quat nrot(xrot * D2R, osg::X_AXIS, 0, osg::Y_AXIS, zrot * D2R, osg::Z_AXIS);
			//osg::Vec3 normal = nrot*forward;
			osg::Vec3 normal = eye_rot * nrot * forward;
			osg::Vec3 forward_relative = eye_rot * forward;

			normal.normalize();

			Vec3f_local normal_raydir_local(normal.x(), normal.y(), normal.z());
			Vec3f_local surface_point = orig_local + normal_raydir_local * SPHERE_SIZE;
			Vec3f_local forward_ray(forward_relative.x(), forward_relative.y(), forward_relative.z());

#ifdef SHOW_RAYS
			if (show_full_ball_trace) {
				if ((int)xrot % 5 == 0 && (int)zrot % 5 == 0) {
					osg::Vec3 meh(surface_point.x, surface_point.y, surface_point.z);
					//draw ray trace lines
					vertices_ball_allrays->push_back(meh);
					vertices_ball_allrays->push_back(meh + normal * 400);
					colors_ball_allrays->push_back(osg::Vec4(255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0, full_ball_alpha_channel));
					colors_ball_allrays->push_back(osg::Vec4(255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0, full_ball_alpha_channel));
					normals_ball_allrays->push_back(normalRayVertexDir);
					normals_ball_allrays->push_back(normalRayVertexDir);
				}
			}
#endif

			if (SysLEye->cornea.intersect_reverse(orig_local, normal_raydir_local, cornea_near, cornea_far, cornea_refracted)) {
				osg::Vec3 cornea_near_v3(cornea_near.x, cornea_near.y, cornea_near.z);
				osg::Vec3 cornea_far_v3(cornea_far.x, cornea_far.y, cornea_far.z);
				osg::Vec3 cornea_refracted_v3(cornea_refracted.x, cornea_refracted.y, cornea_refracted.z);
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

				if (SysLEye->pupil.intersect_reverse(cornea_far, cornea_refracted, pupil_near, pupil_far, pupil_refracted)) {
					/*#ifdef SHOW_RAYS
										osg::Vec3 pupil_near_v3(pupil_near.x,pupil_near.y,pupil_near.z);

										vertices_ball->push_back(cornea_far_v3);
										vertices_ball->push_back(pupil_near_v3);
										colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,1));
										colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,1));
					#endif*/

					if (SysLEye->lens.intersect_reverse(pupil_near, pupil_refracted, lens_near, lens_far, lens_refracted)) {
#ifdef SHOW_RAYS
						if (show_forward_trace_rays) {
							osg::Vec3 pupil_near_v3(pupil_near.x, pupil_near.y, pupil_near.z);
							osg::Vec3 lens_near_v3(lens_near.x, lens_near.y, lens_near.z);
							osg::Vec3 lens_far_v3(lens_far.x, lens_far.y, lens_far.z);
							osg::Vec3 lens_refracted_v3(lens_refracted.x, lens_refracted.y, lens_refracted.z);

							vertices_ball->push_back(tposv3);
							vertices_ball->push_back(cornea_near_v3);
							normals_ball->push_back(up);
							//colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));
							//colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));
							colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 10 / 255.0, forward_rays_alpha_channel));
							colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 10 / 255.0, forward_rays_alpha_channel));
							normals_ball->push_back(normalRayVertexDir);
							normals_ball->push_back(normalRayVertexDir);

							vertices_ball->push_back(cornea_near_v3);
							vertices_ball->push_back(cornea_far_v3);
							normals_ball->push_back(up);
							//colors_ball->push_back(osg::Vec4(10/255.0,10/255.0,206/255.0,1));
							//colors_ball->push_back(osg::Vec4(10/255.0,10/255.0,206/255.0,1));
							colors_ball->push_back(osg::Vec4(10 / 255.0, 10 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
							colors_ball->push_back(osg::Vec4(10 / 255.0, 10 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
							normals_ball->push_back(normalRayVertexDir);
							normals_ball->push_back(normalRayVertexDir);

							vertices_ball->push_back(cornea_far_v3);
							vertices_ball->push_back(pupil_near_v3);
							normals_ball->push_back(up);
							//colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,1));
							//colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,1));
							colors_ball->push_back(osg::Vec4(10 / 255.0, 206 / 255.0, 10 / 255.0, forward_rays_alpha_channel));
							colors_ball->push_back(osg::Vec4(10 / 255.0, 206 / 255.0, 10 / 255.0, forward_rays_alpha_channel));
							normals_ball->push_back(normalRayVertexDir);
							normals_ball->push_back(normalRayVertexDir);

							vertices_ball->push_back(pupil_near_v3);
							vertices_ball->push_back(lens_near_v3);
							normals_ball->push_back(up);
							//colors_ball->push_back(osg::Vec4(206/255.0,206/255.0,10/255.0,1));
							//colors_ball->push_back(osg::Vec4(206/255.0,206/255.0,10/255.0,1));
							colors_ball->push_back(osg::Vec4(206 / 255.0, 206 / 255.0, 10 / 255.0, forward_rays_alpha_channel));
							colors_ball->push_back(osg::Vec4(206 / 255.0, 206 / 255.0, 10 / 255.0, forward_rays_alpha_channel));
							normals_ball->push_back(normalRayVertexDir);
							normals_ball->push_back(normalRayVertexDir);

							vertices_ball->push_back(lens_near_v3);
							vertices_ball->push_back(lens_far_v3);
							normals_ball->push_back(up);
							//colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,1));
							//colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,206/255.0,1));
							colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
							colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
							normals_ball->push_back(normalRayVertexDir);
							normals_ball->push_back(normalRayVertexDir);

							osg::Vec3 endpoint = lens_far_v3 + lens_refracted_v3 * 50;

							vertices_ball->push_back(lens_far_v3);
							vertices_ball->push_back(endpoint);
							normals_ball->push_back(up);
							//colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,206/255.0,1));
							//colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,206/255.0,1));
							colors_ball->push_back(osg::Vec4(10 / 255.0, 206 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
							colors_ball->push_back(osg::Vec4(10 / 255.0, 206 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
							normals_ball->push_back(normalRayVertexDir);
							normals_ball->push_back(normalRayVertexDir);

							/*vertices_ball->push_back(tposv3);
							vertices_ball->push_back(tposv3+osg::Vec3(ball_lens_ray.x,ball_lens_ray.y,ball_lens_ray.z)*10000);
							colors_ball->push_back(osg::Vec4(255/255.0,0/255.0,0/255.0,1));
							colors_ball->push_back(osg::Vec4(255/255.0,0/255.0,0/255.0,1));*/

							if (exportMotion)
							{
								fprintf(fileRaytracePoints, "%g %g %g %g %g %g\n", tposv3.x(), tposv3.y(), tposv3.z(), cornea_near_v3.x(), cornea_near_v3.y(), cornea_near_v3.z());
								fprintf(fileRaytracePoints, "%g %g %g %g %g %g\n", cornea_near_v3.x(), cornea_near_v3.y(), cornea_near_v3.z(), cornea_far_v3.x(), cornea_far_v3.y(), cornea_far_v3.z());
								fprintf(fileRaytracePoints, "%g %g %g %g %g %g\n", cornea_far_v3.x(), cornea_far_v3.y(), cornea_far_v3.z(), pupil_near_v3.x(), pupil_near_v3.y(), pupil_near_v3.z());
								fprintf(fileRaytracePoints, "%g %g %g %g %g %g\n", pupil_near_v3.x(), pupil_near_v3.y(), pupil_near_v3.z(), lens_near_v3.x(), lens_near_v3.y(), lens_near_v3.z());
								fprintf(fileRaytracePoints, "%g %g %g %g %g %g\n", lens_near_v3.x(), lens_near_v3.y(), lens_near_v3.z(), lens_far_v3.x(), lens_far_v3.y(), lens_far_v3.z());
								fprintf(fileRaytracePoints, "%g %g %g %g %g %g\n", lens_far_v3.x(), lens_far_v3.y(), lens_far_v3.z(), endpoint.x(), endpoint.y(), endpoint.z());
							}
						}
#endif
						lens_refracted.normalize();

						gVec3 origin_a(lens_far.x, lens_far.y, lens_far.z);
						//gVec3 origin_b(SysLEye->lens.center.x,SysLEye->lens.center.y,SysLEye->lens.center.z);
						gVec3 origin_b(orig_local.x, orig_local.y, orig_local.z);

						gVec3 direction_a(lens_refracted.x, lens_refracted.y, lens_refracted.z);
						//gVec3 direction_b(forward.x(),forward.y(),forward.z());
						//gVec3 direction_b((eye_rot*forward).x(),(eye_rot*forward).y(),(eye_rot*forward).z());
						gVec3 direction_b(ball_lens_ray.x, ball_lens_ray.y, ball_lens_ray.z);

						gVec3 intersection;

						if (lens_far.z > maxz.z) {
							maxz = lens_far;
							step_max = lens_refracted;
						}

						if ((lens_far.z - base->frame().trn().z()) < 2.0 && compute_vector_intersection(origin_a, origin_b, direction_a, direction_b, intersection))
						{
							Vec3f_local int_point(intersection.x(), intersection.y(), intersection.z());
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
	Vec3f_local approxim_foc = Vec3f_local(base->frame().trn().x(), maxz.y, base->frame().trn().z()) + Vec3f_local(0, 1, 0) * steps;

	Vec3f_local summation_points;
	for (std::vector<Vec3f_local>::iterator it = final_box_int_points.begin(); it != final_box_int_points.end(); ++it) {
		summation_points += (*it);
	}
	double multiplier = 1.0 / ((double)final_box_int_points.size());
	Vec3f_local consensus_intersection_point = summation_points * multiplier;

	//cout << "consensus: " << consensus_intersection_point.x << " " << consensus_intersection_point.y << " " << consensus_intersection_point.z << endl;

#ifdef SHOW_RAYS
	osg::Vec3 int_point_v3(consensus_intersection_point.x, consensus_intersection_point.y, consensus_intersection_point.z);
	osg::Vec3 approxim_foc_v3(approxim_foc.x, approxim_foc.y, approxim_foc.z);
	osg::Vec3 approxim_foc_v3_1(approxim_foc.x, approxim_foc.y + 0.1, approxim_foc.z);
	osg::Vec3 approxim_foc_v3_2(approxim_foc.x, approxim_foc.y - 0.1, approxim_foc.z);
	osg::Quat x_marks_the_spot_rot_1(0, osg::X_AXIS, 45.0 * gDTR, osg::Y_AXIS, 0, osg::Z_AXIS);
	osg::Quat x_marks_the_spot_rot_2(0, osg::X_AXIS, -45.0 * gDTR, osg::Y_AXIS, 0, osg::Z_AXIS);
	//draw ray trace lines
	if (show_forward_trace_rays) {
		/*vertices_ball->push_back(int_point_v3-(eye_rot*up)*200);
		vertices_ball->push_back(int_point_v3+(eye_rot*up)*200);
		colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));
		colors_ball->push_back(osg::Vec4(206/255.0,10/255.0,10/255.0,1));*/

		/*vertices_ball->push_back(int_point_v3_rt-(eye_rot*up)*200);
		vertices_ball->push_back(int_point_v3_rt+(eye_rot*up)*200);
		colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,1));
		colors_ball->push_back(osg::Vec4(10/255.0,206/255.0,10/255.0,1));*/

		double xheight = 15.0;

		osg::Vec3 farXlower1 = approxim_foc_v3_1 - (x_marks_the_spot_rot_1 * eye_rot * up) * xheight;
		osg::Vec3 farXupper1 = approxim_foc_v3_1 + (x_marks_the_spot_rot_1 * eye_rot * up) * xheight;
		osg::Vec3 farXlower2 = approxim_foc_v3_1 - (x_marks_the_spot_rot_2 * eye_rot * up) * xheight;
		osg::Vec3 farXupper2 = approxim_foc_v3_1 + (x_marks_the_spot_rot_2 * eye_rot * up) * xheight;

		osg::Vec3 nearXlower1 = approxim_foc_v3_2 - (x_marks_the_spot_rot_1 * eye_rot * up) * xheight;
		osg::Vec3 nearXupper1 = approxim_foc_v3_2 + (x_marks_the_spot_rot_1 * eye_rot * up) * xheight;
		osg::Vec3 nearXlower2 = approxim_foc_v3_2 - (x_marks_the_spot_rot_2 * eye_rot * up) * xheight;
		osg::Vec3 nearXupper2 = approxim_foc_v3_2 + (x_marks_the_spot_rot_2 * eye_rot * up) * xheight;

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
		colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
		colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
		colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
		colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
		colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
		colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
		colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
		colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 206 / 255.0, forward_rays_alpha_channel));
		normals_ball->push_back(normalRayVertexDir);
		normals_ball->push_back(normalRayVertexDir);
		normals_ball->push_back(normalRayVertexDir);
		normals_ball->push_back(normalRayVertexDir);
		normals_ball->push_back(normalRayVertexDir);
		normals_ball->push_back(normalRayVertexDir);
		normals_ball->push_back(normalRayVertexDir);
		normals_ball->push_back(normalRayVertexDir);

		if (exportMotion)
		{
			fprintf(fileRaytracePoints, "%g %g %g %g %g %g\n", farXlower1.x(), farXlower1.y(), farXlower1.z(), farXupper1.x(), farXupper1.y(), farXupper1.z());
			fprintf(fileRaytracePoints, "%g %g %g %g %g %g\n", farXlower2.x(), farXlower2.y(), farXlower2.z(), farXupper2.x(), farXupper2.y(), farXupper2.z());
			fprintf(fileRaytracePoints, "%g %g %g %g %g %g\n", nearXlower1.x(), nearXlower1.y(), nearXlower1.z(), nearXupper1.x(), nearXupper1.y(), nearXupper1.z());
			fprintf(fileRaytracePoints, "%g %g %g %g %g %g\n", nearXlower2.x(), nearXlower2.y(), nearXlower2.z(), nearXupper2.x(), nearXupper2.y(), nearXupper2.z());

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
	osg::Vec3 sphere_near_center_v3(SysLEye->lens.sphere1.center.x, SysLEye->lens.sphere1.center.y, SysLEye->lens.sphere1.center.z);
	osg::Vec3 sphere_far_center_v3(SysLEye->lens.sphere2.center.x, SysLEye->lens.sphere2.center.y, SysLEye->lens.sphere2.center.z);
	double radius_near = SysLEye->lens.sphere1.radius;
	double radius_far = SysLEye->lens.sphere2.radius;

	for (double zrot = 0.0; zrot < 180.0; zrot += 45.0) {
		for (int xrot = 0.0; xrot < 180.0; xrot += 45.0) {
			osg::Quat steprot(xrot * gDTR, osg::X_AXIS, 0, osg::Y_AXIS, zrot * gDTR, osg::Z_AXIS);
			osg::Vec3 fin_dir = steprot * eye_rot * forward;
			fin_dir.normalize();

			vertices_ball->push_back(sphere_near_center_v3 - fin_dir * radius_near);
			vertices_ball->push_back(sphere_near_center_v3 + fin_dir * radius_near);
			colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 10 / 255.0, 1));
			colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 10 / 255.0, 1));

			vertices_ball->push_back(sphere_far_center_v3 - fin_dir * radius_far);
			vertices_ball->push_back(sphere_far_center_v3 + fin_dir * radius_far);
			colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 10 / 255.0, 1));
			colors_ball->push_back(osg::Vec4(206 / 255.0, 10 / 255.0, 10 / 255.0, 1));
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

	if (show_full_ball_trace) {
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
		state_active1->setRenderBinDetails(5, "DepthSortedBin");
		osg::LineWidth* lw_active1 = new osg::LineWidth;
		lw_active1->setWidth(0.01);
		state_active->setAttribute(lw_active1, osg::StateAttribute::ON);
		osg::BlendFunc* blendfunc_active1 = new osg::BlendFunc();
		state_active1->setAttributeAndModes(blendfunc_active1, osg::StateAttribute::ON);
		pat_raytrace_lines->addChild(geode_active1);
	}

	pat_raytrace_lines->setPosition(osg::Vec3d(0.0, 0, 0.0));

	if (focalPointLineGroup->getNumChildren() > 0)
		focalPointLineGroup->removeChild(0, 1);
	focalPointLineGroup->addChild(pat_raytrace_lines);
#endif

	return approxim_foc;
	//return consensus_intersection_point;
}

void setRandomPupilAndLensLengths()
{
	double pupilRadius = randomNumber(1.69055, 4.50033);
	double lensWidth = randomNumber(1.26359, 5.37659);

	SysLEye->adjust_pupil(pupilRadius);
	SysLEye->adjust_lens(lensWidth);
	SysLEye->adjust_cornea();
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
	if (new_target_ball) {
		acc = randomNumber(0.0, 1.0);
	}
#else
#ifdef LENS_VIDEO
	if (oscillatePupilSphincterActivation) {
		static int dir = 1;
		static int maxPupilTimes = 0;
		double sphincter_act_level = SysLEye->muscle(6)->actLevel();
		if (sphincter_act_level <= 0) {
			dir = 1;
		}
		else if (sphincter_act_level >= 1.0) {
			dir = -1;
			maxPupilTimes++;
			if (maxPupilTimes >= 3) {
				pupilMovementDemoComplete = true;
			}
		}
		acc = min(1.0, max(sphincter_act_level + dir * 0.02, 0.0));
	}
	else {
#endif
		double alpha = 0.01; // TWEAK THIS
		double beta = 0.0003; // TWEAK THIS

		static int pupil_adjusted_delay = 0;

		double sphincter_act_level = SysLEye->muscle(6)->actLevel();
		double sphincter_act_adj = min(alpha * (exp(abs(beta * pupil_utility_diff)) - 1.0), 0.2); // TWEAK THIS

		static double diff_epsilon = 1500;
		static int num_adjustment_frames = 0;

		pupil_adjusted = false;
		if (pupil_utility_diff > diff_epsilon) {
			if (pupil_utility > ideal_utility && sphincter_act_level < 1.0) { // constrict
				pupil_adjusted_delay = 0;
				pupil_adjustment_direction = 1;
				num_adjustment_frames++;
			}
			else if (pupil_utility < ideal_utility && sphincter_act_level > 0.0) { // dilate
				pupil_adjusted_delay = 0;
				pupil_adjustment_direction = -1;
				num_adjustment_frames++;
			}
			else {
				if (pupil_adjusted_delay >= 1) {
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
			if (pupil_adjusted_delay >= 1) {
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

		if (num_adjustment_frames > 0 && (num_adjustment_frames % 40) == 0) { // use 40 as final
			//cout << "too many frames - increasing epsilon: " << endl;
			diff_epsilon += 500;
		}

		//cout << "num adjusted frames, epsilon: " << num_adjustment_frames << " " << diff_epsilon << endl;

		double directional_act_adj = pupil_adjustment_direction * sphincter_act_adj;
#ifdef RECORD_LENS
		acc = 0.18; // REVERT THIS
#else
		acc = min(1.0, max(sphincter_act_level + directional_act_adj, 0.0));
		//acc = 0.18; // REVERT THIS
#endif
	//double directional_act_adj = 0.5;
#ifdef LENS_VIDEO
	}
#endif
#endif
	//cout << "error, curr and delt pupil: " << abs(pupil_utility_diff) << " " << sphincter_act_level << " " << directional_act_adj << endl;

	if (bPlay) {
		for (int i = 6; i < 14; i++) {
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

void adjust_lens_cilliary_muscles()
{
#ifndef ONLINE_LENS
	Vec3f_local lens_focal_point = compute_focal_point();

	record_lens_data = (lens_focal_point.y == lens_focal_point.y);

	gLink* base;
	base = SysLEye->findLink("Eyeball");
	Vec3f_local ray_orig(base->frame().trn().x(), base->frame().trn().y(), base->frame().trn().z());
	osg::Vec3 osg_eye_center(ray_orig.x, ray_orig.y, ray_orig.z);

	osg::Vec3 forward(0, 1, 0);
	gVec3 ori_leye = getcurrent_ori_leye() * gDTR;
	osg::Quat q(ori_leye.x(), osg::X_AXIS, 0, osg::Y_AXIS, ori_leye.z(), osg::Z_AXIS);
	forward = q * (forward);

	osg::Vec3 eye_back_frontier = osg_eye_center + forward * EYE_SIZE;
	osg::Vec3 lfpv3(lens_focal_point.x, lens_focal_point.y, lens_focal_point.z);
	//osg::Vec3 lfpv3(stored_focal_point.x,stored_focal_point.y,stored_focal_point.z);
	osg::Vec3 diffLens = lfpv3 - eye_back_frontier;
	lfpd = diffLens;
#endif

	//cout << "acc: " << SysLEye->muscle(14)->actLevel() << endl;

	static double act = 0.0;
#ifndef ONLINE_LENS
#if !defined(TEST_LENS) && !defined(TEST_PUPIL) && !defined(TEST_FOVEATION)
#if defined(RECORD_PUPIL)
	if (new_target_ball) {
		act = randomNumber(0.0, 1.0);
	}
	int direction = ((diffLens * forward) > 0) ? 1 : -1;
	double diffLensMagnitude = diffLens.length();

	lens_error = diffLensMagnitude;
#else
#ifdef LENS_VIDEO
	if (oscillateLensCilliaryActivation) {
		static int dir = 1;
		static int maxLensTimes = 0;
		double cilliary_act_level = SysLEye->muscle(14)->actLevel();
		if (cilliary_act_level <= 0) {
			dir = 1;
		}
		else if (cilliary_act_level >= 1.0) {
			dir = -1;
			maxLensTimes++;
			if (maxLensTimes >= 2) {
				lensMovementDemoComplete = true;
			}
		}
		acc = min(1.0, max(cilliary_act_level + dir * 0.02, 0.0));
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
		double cilliary_act_adj = min(alpha * (exp(beta * diffLensMagnitude) - 1.0), 0.05);

		//double cilliary_act_adj = 0.03; // constant rate

		//double lens_adjustment_rate = min(alpha*exp(abs(beta*yDiffLens)), 0.8);

		lens_adjusted = false;
		if (diffLens.length() > epsilon) {
			if (direction == 1 && cilliary_act_level < 1.0) {
				lens_adjusted_delay = 0;
				lens_adjustment_direction = 1;
				num_adjustment_frames++;
			}
			else if (direction == -1 && cilliary_act_level > 0.0) {
				lens_adjusted_delay = 0;
				lens_adjustment_direction = -1;
				num_adjustment_frames++;
			}
			else {
				if (lens_adjusted_delay >= 1) {
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
			if (lens_adjusted_delay >= 1) {
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

		if (num_adjustment_frames > 0 && (num_adjustment_frames % 45) == 0) { // change to 40
			//cout << "too many frames - increasing epsilon: " << endl;
			epsilon += 0.15;
		}

		//cout << "num adjusted frames, epsilon: " << num_adjustment_frames << " " << epsilon << endl;

		double directional_act_adj = lens_adjustment_direction * cilliary_act_adj;

		//cout << "error, curr and delt lens: " << diffLensMagnitude << " " << cilliary_act_level << " " << directional_act_adj << endl;

		act = min(1.0, max(cilliary_act_level + directional_act_adj, 0.0));
		//acc = 0.0;
		//acc = 1.0;
#ifdef LENS_VIDEO
	}
#endif
#endif

	//cout << "acc: " << act << endl;

	if (bPlay) {
		for (int i = 14; i < 22; i++) {
			SysLEye->muscle(i)->setActLevel(act);
		}
		for (int i = 38; i < 54; i++) {
			SysLEye->muscle(i)->setActLevel(0.038 * (1 - act));
		}

		//cout << "lens act level from fxn: " << SysLEye->muscle(14)->actLevel() << endl;
	}

#endif
#endif
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
	double w = (double)MIN_LENS_WIDTH + ((d - dlensmin) / (dlensmax - dlensmin)) * ((double)MAX_LENS_WIDTH - (double)MIN_LENS_WIDTH);
	//cout << "W " << w << " d " << d << endl;
	SysLEye->adjust_lens(w);
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

void readStdMeanLeft()
{
	int p = 0;
	cout << "reading mean and std" << endl;
#ifdef OLD_RETINA
	ifstream ifs("meanEyeleft.csv");
#else
	ifstream ifs("meanFoveation.csv");
#endif
	if (!ifs) {
		cout << "入力エラー";
		return;
	}
	string str;
	while (getline(ifs, str)) {
		string token;
		istringstream stream(str);
		while (getline(stream, token, ',')) {
			float temp = stof(token);
			meanValueLeft[p] = temp;
			p++;
		}
	}

	p = 0;
#ifdef OLD_RETINA
	ifstream ifs2("stdEyeleft.csv");
#else
	ifstream ifs2("stdFoveation.csv");
#endif
	if (!ifs2) {
		cout << "入力エラー";
		return;
	}
	while (getline(ifs2, str)) {
		string token;
		istringstream stream(str);
		while (getline(stream, token, ',')) {
			float temp = stof(token);
			stdValueLeft[p] = temp;
			p++;
		}
	}
}

void readStdMeanLEye()
{
	int p = 0;
	cout << "reading mean and std" << endl;
	//csvファイルを1行ずつ読み込む
	ifstream ifs("meanEye.csv");
	if (!ifs) {
		cout << "Could not load mean file";
		return;
	}
	string str;
	while (getline(ifs, str)) {
		string token;
		istringstream stream(str); \
			while (getline(stream, token, ',')) {
				float temp = stof(token);
				meanValueLEye[p] = temp;
				p++;
			}
	}

	p = 0;

	ifstream ifs2("stdEye.csv");
	if (!ifs2) {
		cout << "Could not load std file";
		return;
	}
	while (getline(ifs2, str)) {
		string token;
		istringstream stream(str);
		while (getline(stream, token, ',')) {
			float temp = stof(token);
			stdValueLEye[p] = temp;
			p++;
		}
	}
}

void readStdMeanRight()
{
	int p = 0;
	cout << "reading mean and std" << endl;
	ifstream ifs("meanEyeright.csv");
	if (!ifs) {
		cout << "入力エラー";
		return;
	}
	string str;
	while (getline(ifs, str)) {
		string token;
		istringstream stream(str);
		while (getline(stream, token, ',')) {
			float temp = stof(token);
			meanValueRight[p] = temp;
			p++;
		}
	}

	p = 0;

	ifstream ifs2("strEyeright.csv");
	if (!ifs2) {
		cout << "入力エラー";
		return;
	}
	while (getline(ifs2, str)) {
		string token;
		istringstream stream(str);
		while (getline(stream, token, ',')) {
			float temp = stof(token);
			stdValueRight[p] = temp;
			p++;
		}
	}
}

void readStdMeanPupil()
{
	int p = 0;
	cout << "reading mean and std pupil" << endl;
	ifstream ifs("meanPupil.csv");
	if (!ifs) {
		cout << "入力エラー";
		return;
	}
	string str;
	while (getline(ifs, str)) {
		string token;
		istringstream stream(str);
		while (getline(stream, token, ',')) {
			float temp = stof(token);
			meanValuePupil[p] = temp;
			p++;
		}
	}

	p = 0;
	ifstream ifs2("stdPupil.csv");
	if (!ifs2) {
		cout << "入力エラー";
		return;
	}
	while (getline(ifs2, str)) {
		string token;
		istringstream stream(str);
		while (getline(stream, token, ',')) {
			float temp = stof(token);
			stdValuePupil[p] = temp;
			p++;
		}
	}
}

void readStdMeanLens()
{
	int p = 0;
	cout << "reading mean and std lens" << endl;
	ifstream ifs("meanLens.csv"); // physical
	if (!ifs) {
		cout << "入力エラー";
		return;
	}
	string str;
	while (getline(ifs, str)) {
		string token;
		istringstream stream(str);
		while (getline(stream, token, ',')) {
			float temp = stof(token);
			meanValueLens[p] = temp;
			p++;
		}
	}

	p = 0;
	ifstream ifs2("stdLens.csv"); // physical
	if (!ifs2) {
		cout << "入力エラー";
		return;
	}
	while (getline(ifs2, str)) {
		string token;
		istringstream stream(str);
		while (getline(stream, token, ',')) {
			float temp = stof(token);
			stdValueLens[p] = temp;
			p++;
		}
	}
}

void myComputeThetaPhi()
{
	gVec3 ori_leye = getcurrent_ori_leye();
	gLink* base = SysLEye->findLink("Eyeball");

	osg::Vec3 eye_direction_vec(0, -1, 0);
	osg::Quat q2(eye_cxl, osg::X_AXIS, 0, osg::Y_AXIS, eye_cyl, osg::Z_AXIS);
	eye_direction_vec = q2 * (eye_direction_vec);

	osg::Vec3 eye_direction_vec_new(
		base->frame().trn().x() + eye_direction_vec.x(), 
		base->frame().trn().y() + eye_direction_vec.y(), 
		base->frame().trn().z() + eye_direction_vec.z()
	);
	osg::Vec3 eye_up_vec(0, 0, 1);
	//sceneViewLeft->setViewMatrixAsLookAt(
	//	osg::Vec3(base->frame().trn().x(), base->frame().trn().y(), base->frame().trn().z()), // eye pos

	//	eye_direction_vec_new,    // gaze at 

	//	eye_up_vec);   // usual up vector//somehow this up vector does not affect.
	osg::Vec3 eye, centre, up;

	if (visualTarget.get() != nullptr) {

		//raytrace_counter++;
		//raytrace_counter = 0;
		osg::Vec3d spherePos1 = visualTarget.get()->getMatrix().getTrans();
		Vec3f_local sherePos_local1 = Vec3f_local(float(spherePos1.x()), float(spherePos1.y()), float(spherePos1.z()));

		vector<Vec3f_local> spehere_poss;
		spehere_poss.push_back(sherePos_local1);

		Vec3f_local returnedImagePupil[RAYTRACE_WIDTH * RAYTRACE_HEIGHT];
		Vec3f_local returnedImageLens[RAYTRACE_WIDTH * RAYTRACE_HEIGHT];
		Vec3f_local returnedImageFoveation[RAYTRACE_WIDTH * RAYTRACE_HEIGHT];
		//raytracer(sherePos_local, SPHERE_SIZE, Vec3f_local(eye.x(),eye.y(), eye.z()), Vec3f_local(ori_leye.x()*gDTR,0,ori_leye.z()*gDTR),returnedImage,0);
		raytracer(spehere_poss, SPHERE_SIZE, Vec3f_local(eye.x(), eye.y(), eye.z()), Vec3f_local(ori_leye.x() * gDTR, 0, ori_leye.z() * gDTR), returnedImagePupil, returnedImageLens, returnedImageFoveation, 0);

#if defined(ONLINE_FOVEATION) || defined(ONLINE_SNN)
		vector<float> ray_vec;
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
		for (int i = 0; i < RAYTRACE_WIDTH * RAYTRACE_HEIGHT; i++)
		{
			Vec3f_local vp = returnedImagePupil[i];
			Vec3f_local vl = returnedImageLens[i];
			Vec3f_local vf = returnedImageFoveation[i];

			if (vp.x > 0.0 || vp.y > 0.0 || vp.z > 0.0)
			{
				targetExist = true;
				active_idx_pupil.push_back(i);
			}

			if (vl.x > 0.0 || vl.y > 0.0 || vl.z > 0.0)
			{
				targetExist = true;
				active_idx_lens.push_back(i);
			}

			if (vf.x > 0.0 || vf.y > 0.0 || vf.z > 0.0)
			{
				targetExist = true;
				active_idx_foveation.push_back(i);
			}

			active_idx_color_pupil.push_back(vp);
			active_idx_color_lens.push_back(vl);
			active_idx_color_foveation.push_back(vf);

#if defined(ONLINE_FOVEATION) || defined(ONLINE_SNN)
			ray_vec.push_back(vf.x);
#endif
		}

		double tyl, txl;
		static double prior_lens_error = FLT_MAX;
		static int prior_step_lens = 0;
		static bool firstTimeLens = true;
		
		if (targetExist) {
			cout << " HIIIIII" << endl;
#ifdef ONLINE_FOVEATION
		//cout << "current estimated theta, phi (degrees): " << eye_cyl*gRTD << " " << eye_cxl*gRTD << endl;
		sampleRayKeras->set_data(ray_vec);
		vector<float> returned = mRayKeras->compute_output(sampleRayKeras);
		ray_vec.clear();

#endif

				// TODO: call executable here
#ifdef ONLINE_SNN
		cout << " HI " << endl;
		// write ONV to file
		for (int i = 0; i < (RAYTRACE_WIDTH * RAYTRACE_HEIGHT) - 1; i++) {
			fonvOut << returnedImageFoveation[i].x << ",";
		}
		fonvOut << returnedImageFoveation[(RAYTRACE_WIDTH * RAYTRACE_HEIGHT) - 1].x << endl;

		system("C:\\Users\\taasi\\Desktop\\RunSNN\\dist\\main\\main.exe --ft");

		vector<float> returned(2);

		int p1 = 0;
		//cout << "reading foveation result" << endl;
		ifstream ifsResult(snnPrefix + "\\resultOut.csv", ios::app);
		if (!ifsResult) {
			cout << "Could not load foveation result file";
		}
		string str2;
		while (getline(ifsResult, str2)) {
			string token;
			istringstream stream(str2);
			while (getline(stream, token, ',')) {
				float temp = stof(token);
				if (p1 == 0)
					returned[0] = temp;
				else
					returned[1] = temp;
				// cout << temp << " ";
				p1++;
			}
		}
		ifsResult.close();

		// clear file
		ofstream result(snnPrefix + "\\resultOut.csv");
		result.close();
#endif

#if defined(ONLINE_FOVEATION) || defined(ONLINE_SNN)
		tyl = returned[0] * stdValueLeft[0] + meanValueLeft[0] + eye_cyl;
		txl = returned[1] * stdValueLeft[1] + meanValueLeft[1] + eye_cxl;
		tyl_temp = tyl;
		txl_temp = txl;
				// cout << "new estimated raw theta, phi (degrees): " << tyl*gRTD << " " << txl*gRTD << endl;
#endif

		} else {

			if (counter_start_L < counter_total)
			{
				tyl = tyl_temp * (counter_total - counter_start_L) / double(counter_total);
				txl = txl_temp * (counter_total - counter_start_L) / double(counter_total);
				counter_start_L++;
			}
			else {
				tyl = 0.000f;
				txl = 0.000f;
			}
		}
		cout << "nn tyl, txl: " << tyl * gRTD << " " << txl * gRTD << endl;
		cout << "actual tyl, txl: " << theta_vec_conversion(targeting_location) * gRTD << " " << -phi_vec_conversion(targeting_location) * gRTD << endl;

		double angle = atan2(tyl - eye_cyl, txl - eye_cxl);
		double gaze_error = sqrt((txl - eye_cxl) * (txl - eye_cxl) + (tyl - eye_cyl) * (tyl - eye_cyl));

		double param = 1.0;
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

		param = 1.0 / (1.0 + exp(-10 * (gaze_error - 0.38)));

		//param=1.0;
		double maxEyeVelocity = 3.0;//*10;
		double maxEyeLength = 0.610865 * 10;

		double timestep = displayTimeStep;
		double len = gMin(gaze_error, maxEyeVelocity * timestep * param);

		eye_cxl += len * cos(angle);
		eye_cyl += len * sin(angle);
		angle = atan2(eye_cyl, eye_cxl);
		len = gMin(sqrt(eye_cxl * eye_cxl + eye_cyl * eye_cyl), maxEyeLength);

		eye_cxl = len * cos(angle);
		eye_cyl = len * sin(angle);

		//eye_cxl = txl;
		//eye_cyl = tyl;
	}
}

void myComputeThetaPhi2()
{

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

	active_idx_foveation.clear();
	active_idx_color_foveation.clear();

	// for the retina image dump
	for (int i = 0; i < RAYTRACE_WIDTH * RAYTRACE_HEIGHT; i++)
	{
		Vec3f_local vf = returnedImageFoveation[i];
		if (vf.x > 0.0 || vf.y > 0.0 || vf.z > 0.0)
		{
			active_idx_foveation.push_back(i);
		}
		active_idx_color_foveation.push_back(vf);
	}

	ofstream fonvOut("C:\\Users\\taasi\\Desktop\\RunSNN\\files\\onvOut.csv", std::fstream::app);

	for (int i = 0; i < (RAYTRACE_WIDTH * RAYTRACE_HEIGHT) - 1; i++) {
		fonvOut << returnedImageFoveation[i].x << ",";

	}
	fonvOut << returnedImageFoveation[(RAYTRACE_WIDTH * RAYTRACE_HEIGHT) - 1].x << endl;
	fonvOut.close();

	//  -d normal -m FC -n FC_normal_100epoch
	//  -d normal -m LCN -n LCN_normal_100epoch_k25

	// -d delta -m FC -n FC_delta_100epoch_k25
	// -d delta -m LCN -n LCN_delta_100epoch_k25
	// -d delta -m LCNSpikingHybrid -n LCNSpikingHybrid_delta_100epoch_k25_L1 
	// -d delta -m LCNSpikingHybrid -n LCNSpikingHybrid_delta_100epoch_k25_L4

	// -d delta -m LCNChannelStack -n LCNChannelStack_delta_100epoch_k25

	system("C:\\Users\\taasi\\Desktop\\RunSNN\\dist\\main\\main.exe -d delta -m LCNSpikingHybrid -n LCNSpikingHybrid_delta_100epoch_k25_L1");

	vector<float> returned(2);
	ifstream fanglesOut("resultOut.csv");
	if (!fanglesOut) {
		cout << "Er Here" << endl;
		cerr << "Error code: " << strerror(errno) << endl; // Get some info as to why
	}
	int p = 0;
	string str;
	while (getline(fanglesOut, str)) {
		string token;
		istringstream stream(str);
		while (getline(stream, token, ',')) {
			float temp = stof(token);
			returned[p] = temp;
			p++;
		}
	}
	fanglesOut.close();
	
	double tyl, txl;
	tyl = returned[0] * stdValueLeft[0] + meanValueLeft[0] + eye_cyl;
	txl = returned[1] * stdValueLeft[1] + meanValueLeft[1] + eye_cxl;
	tyl_temp = tyl;
	txl_temp = txl;

	cout << "nn tyl, txl: " << tyl * gRTD << " " << txl * gRTD << endl;
	cout << "actual tyl, txl: " << theta_vec_conversion(targeting_location) * gRTD << " " << -phi_vec_conversion(targeting_location) * gRTD << endl;

#ifdef RECORD_ANGLES
	frecordAngles << theta_vec_conversion(targeting_location)* gRTD << ", " << -phi_vec_conversion(targeting_location) * gRTD << endl;
	frecordAngles1 << tyl * gRTD << ", " << txl * gRTD << endl;
#endif

	double angle = atan2(tyl - eye_cyl, txl - eye_cxl);
	double gaze_error = sqrt((txl - eye_cxl) * (txl - eye_cxl) + (tyl - eye_cyl) * (tyl - eye_cyl));

	double param = 1.0;

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


	param = 1.0 / (1.0 + exp(-10 * (gaze_error - 0.38)));

	//param=1.0;
	double maxEyeVelocity = 3.0;//*10;
	double maxEyeLength = 0.610865 * 10;

	double timestep = displayTimeStep;
	double len = gMin(gaze_error, maxEyeVelocity * timestep * param);

	eye_cxl += len * cos(angle);
	eye_cyl += len * sin(angle);
	angle = atan2(eye_cyl, eye_cxl);
	len = gMin(sqrt(eye_cxl * eye_cxl + eye_cyl * eye_cyl), maxEyeLength);

	eye_cxl = len * cos(angle);
	eye_cyl = len * sin(angle);


}

/*
	TODO: 
		write actual tyl txl to file
		nn tyl txl to diff files
*/