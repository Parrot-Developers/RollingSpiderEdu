/*
 * rsedu_params.h
 *
 *  Created on: Aug 16, 2015
 *      Author: ra
 */
#ifndef RSEDU_PARAMS_H_
#define RSEDU_PARAMS_H_

//indicator flags
#define  NO_VIS_X -99.0

//timing
#include "ptimer.h"

//set of existing features to be dis/enabled
extern int FEAT_TIME;
extern int FEAT_OF_ACTIVE;
extern int FEAT_POSVIS_RUN;
extern int FEAT_POSVIS_USE;
extern int FEAT_IMSAVE;
extern int FEAT_NOLOOK;
extern int FEAT_NOSAFETY;

//Flight time, takeoff, calibration
extern int onCycles;
extern int calibCycles;
extern int takeoffCycles;

#endif /* RSEDU_PARAMS_H_ */
