//File: image_module.h
/*
* AUTHOR Fabian Riether
* CREATE DATE 2015/08/25
* ===============================
* Change History
* 2015/08/25 created
* ==================================
*/

#ifndef RSEDU_IMAGE_H_INCLUDED
#define RSEDU_IMAGE_H_INCLUDED



#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include <inttypes.h>
#include <stddef.h>

#include "../../embcode/rsedu_vis.h"

/* function declarations*/
void createMatchLookup();
void RSEDU_image_processing_OFFBOARD(void * buffer,int matchResult[160][120],int kimg);

#endif
