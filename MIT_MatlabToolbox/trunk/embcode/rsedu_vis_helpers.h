
#ifndef RSEDU_VIS_HELPERS_H_
#define RSEDU_VIS_HELPERS_H_

#include <math.h>

extern double ldnmrk_pinv[4][4];
extern double intrMatrx_inv[3][3];

//function declarations
void reconstructCameraPose(float camerapos[3],float *camerayaw,float feature_pps[3][4],double reconrightMatrix[4][4],double iPMHF[3][3]);

#endif
