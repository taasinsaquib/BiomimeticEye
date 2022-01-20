#ifndef UTILS_H
#define UTILS_H

#include "HuMuLEye.h"
#include "gkd/gkd_general.h"

class Utils {
public:
	void raytracer(vector<Vec3f_local> spos, float radius, Vec3f_local ray_orig, Vec3f_local ray_di, 
		Vec3f_local* returnedImagePupil, Vec3f_local* returnedImageLens, Vec3f_local* returnedImageFoveation, int side);
	Vec3f_local trace(const Vec3f_local& rayorig, Vec3f_local& raydir,
		std::vector<Sphere_local>& Sphere_locals, const int& depth, int filterOption = 0);

	void render(std::vector<Sphere_local>& Sphere_locals, Vec3f_local ray_orig, Vec3f_local ray_dir,
		Vec3f_local* returnedImage, int side);
	void render_ultimate(std::vector<Sphere_local>& Sphere_locals, Vec3f_local ray_orig, Vec3f_local ray_dir, 
		Vec3f_local* returnedImagePupil, Vec3f_local* returnedImageLens, Vec3f_local* returnedImageFoveation, int side);

	void finish();

	double phi_vec_conversion(gVec3 v);
	double theta_vec_conversion(gVec3 v);
private:
};

#endif // !UTILS_H

