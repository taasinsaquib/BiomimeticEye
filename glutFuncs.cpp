#include <iostream>

#include <GLUT/glut.h>
#include <osg/Timer>
#include <osgUtil/SceneView>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Quat>
#include <osg/Light>
#include <osg/LightSource>

#include "constants.h"
#include "helpers.h"
#include "dataIO.h"
#include "glutFuncs.h"

// test

#include <omp.h> 
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>  
#include <math.h>
#include "snopt/snopt_c.h"

#include "HuMuLEye.h"

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
	// why are x and y commented out like that
	// gXMat T; // not used, so I commented it out

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