//File: rsedu_vis.h
/*
* AUTHOR Fabian Riether
* CREATE DATE 2015/08/25
* ===============================
* Change History
* 2015/08/25 created
* ==================================
*/

//frameworkparameter
#ifndef RSEDU_PARAMS_H_
#include "rsedu_params.h"
#endif

#ifndef RSEDU_IMAGE_H_
#define RSEDU_IMAGE_H_

//communication
#include <sys/stat.h>
#include <fcntl.h>

//image processing
#include <math.h>



typedef signed char s8;
typedef unsigned char u8;

typedef struct {
	u8 y1;
	u8 u;
	u8 y2;
	u8 v;
} __attribute__((packed)) pixel2_t;  /* Defines a couple of 2 consecutive pixels in YUV422 interlaced format */

typedef struct {
	float descr[3];  //hue,green,blue values of landmark
	float X;  		 //landmark position in real world x
	float Y;  		 //landmark position in real world y
	float px; 		 //average image coordinate [x pixel]
	float py; 		 //average image coordinate [y pixel]
	int n;	   		 //number of pixel lndmrks to this ldnmark
	int weights;
} __attribute__((packed)) lndmrk_t;  /* Defines a 3D real-world landmark */


//function declarations
void reconstructCameraPose(float camerapos[3],float *camerayaw,float feature_pps[3][4],double reconrightMatrix[4][4],double iPMHF[3][3]);

//void RSEDU_vision(void * buffer);
void RSEDU_image_processing(void * buffer);

#endif /* RSEDU_IMAGE_H_ */
