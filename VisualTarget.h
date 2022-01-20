#ifndef VISUAL_TARGET_H
#define VISUAL_TARGET_H

#include <osg/MatrixTransform>

class VisualTarget {
public:
	void create_visual_target(void);
	void create_visual_target2(void);
	void move__visual_target_eye_training(osg::MatrixTransform* trans_visual_target, double time);
	void movevisual_target(osg::MatrixTransform* trans_visual_target, double time);
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

private:
};

#endif
