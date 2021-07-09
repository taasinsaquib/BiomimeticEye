#include <iostream>

#include "constants.h"
#include "helpers.h"

void finish()
{
	//if(recordMuscleAct ) finishRecord();

	if(exportMotion){
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


    // uncomment later if needed
    /*
	if(fpQ)
	{
		fprintf(fpQ,"];");
		fclose(fpQ);
	}

	if(fpPt)
	{
		fprintf(fpPt,"];");
		fclose(fpPt);
	}

	if(fpPh)
	{
		fprintf(fpPh,"];");
		fclose(fpPh);
	}

	if(snRw){
		delete[] snRw;
		snRw = 0;
	}
	if(snIw) {
		delete[] snIw;
		snIw = 0;
	}
	if(snCw) {
		delete[] snCw;
		snCw = 0;
	}
    */
}