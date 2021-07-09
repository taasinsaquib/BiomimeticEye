#ifndef DATAIO_H
#define DATAIO_H

void writeBallPoses(int frame, FILE* fp);
void generateTrainingDatasetPupil();
void generateTrainingDatasetLens();
void generateTrainingDatasetFoveation();

#endif