#ifndef EYE_UTILS_H
#define EYE_UTILS_H

#include "gkd/gkd_multibody_system.h"
#include "HuMuLEye.h"

class EyeUtils {
public:

	gVec3 getcurrent_ori_leye();

	Vec3f_local compute_focal_point();
	
	void setRandomPupilAndLensLengths();

	void adjust_pupil_sphincter_muscles();
	void compute_pupil_size();

	void adjust_lens_cilliary_muscles();
	void compute_lens_size();

	void resetToInitialPoseLeye();

	void readStdMeanLeft();
	void readStdMeanLEye();
	void readStdMeanRight();
	void readStdMeanPupil();
	void readStdMeanLens();

	void myComputeThetaPhi();
	void myComputeThetaPhi2();
private:
};

#endif // !EYE_UTILS_H

