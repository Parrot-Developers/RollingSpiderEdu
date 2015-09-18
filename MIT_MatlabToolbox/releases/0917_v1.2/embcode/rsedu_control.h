//File: rsedu_control.h
/*
* AUTHOR Fabian Riether
* CREATE DATE 2015/08/25
* ===============================
* Change History
* 2015/08/25 created
* ==================================
*/

#ifndef RSEDU_CONTROL_H_INCLUDED
#define RSEDU_CONTROL_H_INCLUDED

#include "HAL.h"

#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include <inttypes.h>
#include <stddef.h>

#include "DroneRS_Compensator.h"                     /* Model's header file */
#include "rtwtypes.h"



/* function declarations*/

void RSEDU_control(HAL_acquisition_t* hal_sensors_data, HAL_command_t* hal_sensors_cmd);

#endif
