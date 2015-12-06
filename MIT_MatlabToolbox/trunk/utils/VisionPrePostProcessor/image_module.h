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


#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <inttypes.h>
#include <stddef.h>

#include "../../embcode/rsedu_vis.h"


/* function declarations*/
void createMatchLookup(lndmrk_t* lndmrks,int lndmrk_nr);
void RSEDU_image_processing_OFFBOARD(void * buffer,int matchResult[160][120],int kimg,lndmrk_t *lndmrks,int lndmrk_nr);

//allocates storage for matrix of size nx * ny
void alloc_matrix
     (float ***matrix,  /* matrix */
      long  nx,         /* size in x-direction */
      long  ny);         /* size in y-direction */

//disallocates storage for matrix of size nx * ny
void disalloc_matrix
     (float **matrix,   /* matrix */
      long  nx,         /* size in x-direction */
      long  ny);         /* size in y-direction */

//calculates minimum, maximum, mean and variance of a RGB image
void analyse_matrix
     (float  **myMatrix,          /* Red channel of RGB image */
      long    nx,          /* pixel number in x-direction */
      long    ny);

//rgb to yuv conversion of image
void RGB_to_YUV
     (float  **R,          /* Red channel of RGB image */
      float  **G,          /* Green channel of RGB image */
      float  **B,          /* Blue channel of RGB image */
      float  **Y,          /* Y channel of YCbCr image */
      float  **U,         /* Cb channel of YCbCr image */
      float  **V,         /* Cr channel of YCbCr image */
      long    nx,          /* pixel number in x-direction */
      long    ny);          /* pixel number in y-direction */

// yuv to hsv. conversion 0<h<360
void YUV_to_HSVimg(float  **Y,
	      float  **U,
	      float  **V,
	      float  **H,
	      float  **S,
	      float  **Va,
	      long    nx,          /* pixel number in x-direction */
	      long    ny);          /* pixel number in y-direction */

/*
 *conversion yuv to hsv. 0<h<360
 */
void YUVtoHSV(float* yuv,float* rgb,float* hsv);

//write image
void writeImgChannelstoFile(char *outfilename, float **C1,float**C2,float **C3,int nx,int ny,char *in);


//write matching result as image
void writeIntArrtoFile(char *outfilename, int C1[80][120],int C2[80][120],int C3[80][120],int nx,int ny,char *in);

/*
 * compute feature-to-landmark descriptor-distance
 */
float featuredist(float* descr_hsv, float* descr_rgb, float* descr_lndmark);

#endif
