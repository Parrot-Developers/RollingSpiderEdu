//File: rsedu_of.h
/*
* AUTHOR Parrot
* CREATE DATE 2015/08/25
* SPECIAL NOTES: don't need to update this (Fabian)
* ===============================
* Change History
* 2015/08/25 created
* ==================================
*/

//frameworkparameters
#ifndef RSEDU_PARAMS_H_
#include "rsedu_params.h"
#endif

#ifndef RSEDU_OF_H_
#define RSEDU_OF_H_


//communication
#include <sys/stat.h>
#include <fcntl.h>

//function declarations
void RSEDU_optical_flow(float vx, float vy, float vz,int defined,float qualityIndicator);

#endif /* RSEDU_OF_H_ */
