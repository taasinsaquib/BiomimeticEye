#ifndef TRAINING_DATASET_H
#define TRAINING_DATASET_H

class TrainingDataset {
public:
	void generateTrainingDatasetByPoseBasedTargetInverseDynamicsWithDisplay();
	void generateTrainingDatasetPupil();
	void generateTrainingDatasetLens();
	void generateTrainingDatasetFoveation();
	//void generateTrainingDatasetMotor();
	//void generateTrainingDatasetMotor_fixaton_dataset();
	//void generateTrainingDatasetMotorWithDisplay();
	//void generateTrainingDatasetMotorWithDisplay_fixaton();
	
	//void testLoopKinematicAndDNNFixatonsDisplay();

	void testGeneratedDatasetFromTrainingLeyeMotor();
	void testGeneratedDatasetFromTrainingLeyePupil();
	void testGeneratedDatasetFromTrainingLeyeLens();
	void testGeneratedDatasetFromTrainingLeyeFoveation();

private:
};

#endif