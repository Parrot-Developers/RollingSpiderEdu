//File: image_module.c
/* ===============================
* AUTHOR Fabian Riether
* CREATE DATE 2015/08/25
* PURPOSE This module provides code to create a lookup-table for classifying and matcing pixels to a database of landmarks,
* code for pose reconstruction and color conversion
* SPECIAL NOTES
* ===============================
* Change History
* 2015/08/25 created
* ==================================
*/

//#include "HAL.h"
#include "image_module.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


typedef signed char s8;
typedef unsigned char u8;

#define NANOS 1000000000LL

int FEAT_NOLOOKUP = 1;

//Pixel-Landmark Matching - Thresholds
float s_thrshld = 0.35; 	//saturation threshold to detect colored balls
float va_thrshld = 0.28; 	//value min threshold to detect colored balls
float sva_sum_thrshld = 0.7;
float featdist_thrshld=300;
int pxls_ftr_min = 5;		//minimum required nr of detected pixels per landmark


//----------------------------------
// Image processing helpers
//----------------------------------


/*
 *conversion yuv to hsv. 0<h<360
 */
void YUVtoHSV(float* yuv,float* rgb,float* hsv)
{

float red,green,blue;
float rgbmin,rgbmax;

//https://innovativesolution.wordpress.com/2009/09/10/yuv-to-rgb-and-rgb-to-yuv-conversions/

red  = 1.1644*(yuv[0] - 16.0) + 1.596*(yuv[2] - 128.0);
green= 1.1644*(yuv[0] - 16.0) - 0.813*(yuv[1] - 128.0) - 0.391*(yuv[2] - 128.0);
blue = 1.1644*(yuv[0] - 16.0) + 2.018*(yuv[1] - 128.0);

rgbmax = red;
(rgbmax < green) && (rgbmax = green);
(rgbmax < blue) && (rgbmax = blue);

rgbmin = red;
(rgbmin > green) && (rgbmin = green);
(rgbmin > blue) && (rgbmin = blue);

//H
if (rgbmax == rgbmin)
	{
	hsv[0] = 0.0;
	}
else if (rgbmax==red)
	{
	hsv[0] = 60*(0+(green-blue)/(rgbmax-rgbmin));
	}
else if (rgbmax==green)
	{
	hsv[0] = 60*(2+(blue-red)/(rgbmax-rgbmin));
	}
else
	{
	hsv[0] = 60*(4+(red-green)/(rgbmax-rgbmin));
	}

if  (hsv[0]<0) {hsv[0]=hsv[0]+360.0;}

//S, V
(rgbmax==0) ? hsv[1]=0 : (hsv[1]=(rgbmax-rgbmin)/rgbmax);
hsv[2] = rgbmax/255;

rgb[0] = red;
rgb[1] = green;
rgb[2] = blue;

}

/*
 * compute feature-to-landmark descriptor-distance
 */
float featuredist(float* descr_hsv, float* descr_rgb, float* descr_lndmark)
{
	double h_dist,h_dist_cnd,g_dist,b_dist;

	//descr_landmark = [hue green blue]

	//h-distance is on 360group
	h_dist = pow((descr_hsv[0]-descr_lndmark[0]),2);
	h_dist_cnd = pow((descr_hsv[0]-descr_lndmark[0]-360),2);
	if (h_dist_cnd<h_dist) h_dist = h_dist_cnd;
	h_dist_cnd = pow((descr_hsv[0]-descr_lndmark[0]+360),2);
	if (h_dist_cnd<h_dist) h_dist = h_dist_cnd;

	//color distance
	//g_dist = pow(descr_lndmark[1]-descr_rgb[1],2);
	//b_dist = pow(descr_lndmark[2]-descr_rgb[2],2);

	//printf("h: %f, landmark h: %f, dist: %f \n",descr_hsv[0],descr_lndmark[0],h_dist);
	return h_dist;
}


//----------------------------------
// Create lookup-table for pixel-to-marker matching
//----------------------------------

void createMatchLookup()
{
	int i,j,k;
	float yuv[3],rgb[3],hsv[3];
	unsigned char matchLookuptable[128][128][128];

	//matching-Thresholds, global

	//landmarks
	int lndmrk_nr = 5,lndmrk_best=0,lndmrk_cnd=0;
	float featuredist_best = featdist_thrshld;
	float featuredist_cnd = featdist_thrshld;
	static lndmrk_t *lndmrks;

	//Init landmarks
	lndmrks = malloc(lndmrk_nr*sizeof(lndmrk_t));

	//yellow - not used for localization
	lndmrks[0].X = 0.0;
	lndmrks[0].Y = 0.0;
	lndmrks[0].descr[0] = 70.0;	//HUE
	lndmrks[0].descr[1] = 170.0;	//green
	lndmrks[0].descr[2] = 100.0;	//blue

	//green
	lndmrks[1].X = 0.0;
	lndmrks[1].Y = 0.3;
	lndmrks[1].descr[0] = 130.0;
	lndmrks[1].descr[1] = 160.0;
	lndmrks[1].descr[2] = 110.0;

	//pink
	lndmrks[2].X = 0.5;
	lndmrks[2].Y = 0.28;
	lndmrks[2].descr[0] = 345.0;
	lndmrks[2].descr[1] = 100.0;
	lndmrks[2].descr[2] = 135.0;

	//red
	lndmrks[3].X = 0.25;
	lndmrks[3].Y = 0.20;
	lndmrks[3].descr[0] = 25.0;
	lndmrks[3].descr[1] = 80.0;
	lndmrks[3].descr[2] = 70.0;

	//blue
	lndmrks[4].X = 0.5;
	lndmrks[4].Y = 0.0;
	lndmrks[4].descr[0] = 220.0;
	lndmrks[4].descr[1] = 120.0;
	lndmrks[4].descr[2] = 160.0;

	//create table

	for (i=0;i<255;i=i+2)
	{
			 for (j=0;j<255;j=j+2)
			{
				 for (k=0;k<255;k=k+2)
				 {

					 yuv[0] = i;
					 yuv[1] = j;
					 yuv[2] = k;

					 YUVtoHSV(yuv,rgb,hsv);

					 //Find marchting landmark
					 lndmrk_best = 0;
					 //check if marker is candidate for colored marker by sat, match and store
					 if ((hsv[1]>=s_thrshld) && (hsv[2]>=va_thrshld) && ((hsv[1]+hsv[2])>=sva_sum_thrshld))
					 {
						 //find closest matching landmark

						 featuredist_best = featuredist(hsv,rgb,&(lndmrks[lndmrk_best].descr[0]));

						 for (lndmrk_cnd=1;lndmrk_cnd<lndmrk_nr;lndmrk_cnd++)
						 {
							 featuredist_cnd = featuredist(hsv,rgb,&(lndmrks[lndmrk_cnd].descr[0]));
							 if  (featuredist_cnd < featuredist_best)
							 {
									 featuredist_best = featuredist_cnd;
									 lndmrk_best = lndmrk_cnd;
							 }
						 }


						 if (featuredist_best >= featdist_thrshld) //@TODO add more outlier handling like ransac
						 	 {
							//set invalid (marker 0)
							 lndmrk_best = 0;
						 	 }
					 	 }

					 matchLookuptable[(int)(i/2)][(int)(j/2)][(int)(k/2)] = lndmrk_best;
				 	 }//innermost forloop
			 	 }
			 }

	//save lookuptable
	FILE * lookupfile ;

    if ( (lookupfile = fopen("../../DroneExchange/params/lookuptable.dat", "wb")) == NULL )
    {
        printf("matchlookupgenerator(): Error opening file\n");
    }

    fwrite(matchLookuptable, sizeof(matchLookuptable), 1, lookupfile);

}


//----------------------------------
 // reconstruct camera pose
 //----------------------------------

void reconstructCameraPose(float camerapos[3],float *camerayaw,float feature_pps[3][4],double reconrightMatrix[4][4],double intrMatrx_inv[3][3])
{
    double LiMatrix[3][4];
    double LiiMatrix[3][4];
    double scalefactor;
    int c,d,k;
    int m=3;
    int q=4;
    int p=4;
    double sum=0.0;

    for (c = 0; c < m; c++)
   {
      for (d = 0; d < q; d++)
      {
        for (k = 0; k < p; k++)
        {
          sum = sum + feature_pps[c][k]*reconrightMatrix[k][d];
        }
        LiMatrix[c][d] = sum;
        sum = 0;
      }
    }

    for (c = 0; c < m; c++)
   {
      for (d = 0; d < q; d++)
      {

        for (k = 0; k < m; k++)
        {
          sum = sum + intrMatrx_inv[c][k]*LiMatrix[k][d];
        }
        LiiMatrix[c][d] = sum;
        sum = 0;
      }
    }




   scalefactor = 1/sqrt(pow(LiiMatrix[0][0],2)+pow(LiiMatrix[0][1],2));

   camerapos[0] = (float)(LiiMatrix[1][3]*scalefactor);
   camerapos[1] = (float)(-LiiMatrix[0][3]*scalefactor);
   camerapos[2] = (float)(LiiMatrix[2][3]*scalefactor);

   //Yaw: corrected to output x-axis(RS)alignment with x-axis of landmark field. z-axis facing down, rotation +-pi
   float cosa = (LiiMatrix[0][0])*scalefactor;
   float sina = -(LiiMatrix[0][1])*scalefactor;
   if (cosa>0)
   {
	   if (sina<0) {*camerayaw    = (float)(acos(cosa)-1.571);}
	   else 	   {*camerayaw    = (float)(-acos(cosa)-1.571);};

   }
   else
   {
	   if (sina<0) {*camerayaw    = (float)(acos(cosa)-1.571);}
	   else 	   {*camerayaw    = (float)(-acos(cosa)+1.571*3);};
   };



}




//----------------------------------
// Image processing / Vision-based pose estimation
//----------------------------------
/*
 * @input buffer Pointer to the current picture seen by the vertical camera
 * Picture is 160x120 pixels in YUYV format, i.e. 80x120 elements of type 'pixel2_t'
 *
 */

void RSEDU_image_processing_OFFBOARD(void * buffer, int matchResult[80][120],int kimg)
{

	struct timeval now;
	static int counter = 0;

	int i,j;
	int row,col;

	float hsv[3];
	float yuv[3];
	float rgb[3];
	int nx=80, ny=120;
	float feature_pps[3][4];
	unsigned char matchLookuptable[128][128][128];


	float camerapos[3];
	float camerayaw;

	//matching-Thresholds, global

	//Landmarks
	int lndmrk_nr = 5,lndmrk_best=0,lndmrk_cnd=0;
	float featuredist_best = featdist_thrshld;
	float featuredist_cnd = featdist_thrshld;
	static lndmrk_t *lndmrks;
	static double ldnmrk_pinv[4][4] =
			 {{-3.093396787626414,   2.082093991671625                   ,0,   0.766805472932779},
			   {2.320047590719810 ,  3.438429506246282    ,               0  , 0.174895895300416},
			  {-1.737061273051754 , -2.676977989292088    ,               0  , 0.299821534800714},
			   {2.510410469958359 , -2.843545508625819     ,              0 , -0.241522903033909}}; //watch out: dimensions x-y switched compared to image matrices!

	static double intrMatrx_inv[3][3] =       {{ 0.006551831768882                   ,0,  -0.550082487527771},
            {0,   0.006546559888686,  -0.399495805347318},
            {0,                   0,   1.000000000000000}};

	//communication
	pixel2_t *image = buffer;  /* Picture is a 160x120 pixels, stored in YUV422 interlaced format - TO BE CHECKED */


	//time
	 struct timespec begin, current;
	    long long start, elapsed, microseconds;
	    /* set up start time data */
	    if (clock_gettime(CLOCK_MONOTONIC, &begin)) {
	        /* Oops, getting clock time failed */
	        exit(EXIT_FAILURE);
	    }
	    /* Start time in nanoseconds */
	    start = begin.tv_sec*NANOS + begin.tv_nsec;


	counter++;
	gettimeofday(&now,NULL);
	
	if (counter==1)
	{
		//setup landmark database
		lndmrks = malloc(lndmrk_nr*sizeof(lndmrk_t));

		//yellow - not used for localization
		lndmrks[0].X = 0.0;
		lndmrks[0].Y = 0.0;
		lndmrks[0].descr[0] = 70.0;		//hue
		lndmrks[0].descr[1] = 170.0;	//green
		lndmrks[0].descr[2] = 100.0;	//blue

		//green
		lndmrks[1].X = 0.0;
		lndmrks[1].Y = 0.3;
		lndmrks[1].descr[0] = 130.0;
		lndmrks[1].descr[1] = 160.0;
		lndmrks[1].descr[2] = 110.0;

		//pink
		lndmrks[2].X = 0.5;
		lndmrks[2].Y = 0.28;
		lndmrks[2].descr[0] = 345.0;
		lndmrks[2].descr[1] = 100.0;
		lndmrks[2].descr[2] = 135.0;

		//red
		lndmrks[3].X = 0.25;
		lndmrks[3].Y = 0.20;
		lndmrks[3].descr[0] = 25.0;
		lndmrks[3].descr[1] = 80.0;
		lndmrks[3].descr[2] = 70.0;

		//blue
		lndmrks[4].X = 0.5;
		lndmrks[4].Y = 0.0;
		lndmrks[4].descr[0] = 220.0;
		lndmrks[4].descr[1] = 120.0;
		lndmrks[4].descr[2] = 160.0;



		//reset posefile
		fclose(fopen("../../DroneExchange/imgs/processed/poses.txt","w"));

		//load lookuptable for matching process
		if (FEAT_NOLOOKUP==0)
		{

			 FILE* data;
			  if ((data = fopen("../../DroneExchange/params/lookuptable.dat", "rb")) == NULL)
				{
					printf("Error opening lookupfile \n");
					error(0);
				}

				fread(matchLookuptable, sizeof(matchLookuptable), 1, data);
				fclose(data);
		}


	}

	//reset landmark matching data
	for (i=0;i<lndmrk_nr;i++)
	{
		lndmrks[i].n  = 0;
		lndmrks[i].weights  = 0;
		lndmrks[i].px = 0;
		lndmrks[i].py = 0;
		printf("matches reset!\n");
	}

	

	//marker detection
	//cycle through image, convert each pixel to hsv, threshold by s and v to find point, match h value to color-closest in "database", save average pixellocation with database-lndmrksID
	//reconstruct position (3d pos with yaw that mininimzes errors seen with predicted marker position)

	int filtersize = 5; //odd numbered! dummy for potential gaussian filter mask, etc.
	int margin = (int)(filtersize-1)/4;
	int weight = 1;

	 if(NULL!=image)
	{
		 for (j=0;j<ny-2*margin;j++)
		 {
			 for (i=0;i<nx-2*margin;i++)
			 {
				 row = margin+j;
				 col = margin+i;
				 yuv[0] = (float)image[nx*row+col].y1;
				 yuv[1] = (float)image[nx*row+col].u;
				 yuv[2] = (float)image[nx*row+col].v;

				 //find best matching landmark
				 if (FEAT_NOLOOKUP)
				 {
					 YUVtoHSV(yuv,rgb,hsv);


					 //find matching landmark
					 lndmrk_best = 0;
					 //check if marker is candidate for colored marker by sat, match and store
					 if ((hsv[1]>=s_thrshld) && (hsv[2]>=va_thrshld) && ((hsv[1]+hsv[2])>=sva_sum_thrshld))
					 {
						 //find closest matching landmark

						 featuredist_best = featuredist(hsv,rgb,&(lndmrks[lndmrk_best].descr[0]));

						 for (lndmrk_cnd=1;lndmrk_cnd<lndmrk_nr;lndmrk_cnd++)
						 {
							 featuredist_cnd = featuredist(hsv,rgb,&(lndmrks[lndmrk_cnd].descr[0]));
							 if  (featuredist_cnd < featuredist_best)
									 {
									 featuredist_best = featuredist_cnd;
									 lndmrk_best = lndmrk_cnd;
									 }
						 }
						 //update centerpoint of marker if matching succeeded

						 if (featuredist_best >= featdist_thrshld) //@TODO add more outlier handling like ransac
						 {
							//set invalid (marker 0)
							 lndmrk_best = 0;
						 }
					 }
				 }
				 else
				 //use lookuptable
				 {
					 lndmrk_best = (int)matchLookuptable[(int)(yuv[0]/2)][(int)(yuv[1]/2)][(int)(yuv[2]/2)];
				 }


				 //Store matched landmarks
				 //----
				 if (lndmrk_best>0)
						 {
						 //increase weight if close to an existing, highly weighted cluster
						 if ( (lndmrks[lndmrk_best].weights>15) && (abs((col+1)-lndmrks[lndmrk_best].px)<20) && (abs((row+1)-lndmrks[lndmrk_best].py)<20))
								{
								 weight=8;
								}
						 else  {
								 weight=1;
								}

						lndmrks[lndmrk_best].px = lndmrks[lndmrk_best].px*lndmrks[lndmrk_best].weights/(lndmrks[lndmrk_best].weights+weight) + (((double)(col+1))*2)*weight/(lndmrks[lndmrk_best].weights+weight); //@TODO is it row?
						lndmrks[lndmrk_best].py = lndmrks[lndmrk_best].py*lndmrks[lndmrk_best].weights/(lndmrks[lndmrk_best].weights+weight) + ((double)(row+1))*weight/(lndmrks[lndmrk_best].weights+weight); //@TODO is it row?
						 //save that one more pixel lndmrks was added to this landmark
						 lndmrks[lndmrk_best].n += 1;
						 lndmrks[lndmrk_best].weights += weight;

						 matchResult[col][row] = lndmrk_best*40+80;
						 //if (lndmrk_best==0) {matchResult[col][row]=255;} else {matchResult[col][row]=0;}
						 }

			 }//end inner for
		 }//end outer for

		printf("Result of matching: \n");
		for (i=1;i<lndmrk_nr;i++)
		{
			printf("Lndmark %i: nr pxls matched: %i, pixelcoordinates: (%f,%f) \n",i,lndmrks[i].n,lndmrks[i].px,lndmrks[i].py);
		}

		//reconstruct pose
		//choose lndmrks with some minimum number of pixels, add them to the feature_pps (ui,vi,1)

		int features_valid=0;
		int k;

		for (k=1;k<lndmrk_nr;k++) //start from 1 as yellow not used!
		{
			if (lndmrks[k].n>=pxls_ftr_min)
			{
				features_valid+=1;
				feature_pps[0][features_valid-1] = lndmrks[k].px;
				feature_pps[1][features_valid-1] = lndmrks[k].py;
				feature_pps[2][features_valid-1] = 1.0;
			}
		
		}

		//reconstruct camera pose (3D position + yaw) //@TODO if want to be flexible with matrix size depending on found features, need to swap dimensions of matrix (first dim can be variable when passing 2D-array as parameter to a function);

		if (features_valid==4)
		{
			reconstructCameraPose(camerapos,&camerayaw,feature_pps,ldnmrk_pinv,intrMatrx_inv);
			printf("reconstructed cam pos: (%f, %f, %f,%f)\n",camerapos[0],camerapos[1],camerapos[2],camerayaw*180/3.1415);

			FILE * posefile = fopen("../../DroneExchange/imgs/processed/poses.txt","a");

			if (posefile == NULL)
			{
				printf("image_proc(): Error opening pose file! \n");
			}
			else
			{
				fprintf(posefile,"%i, %f, %f, %f,%f \n",kimg, camerapos[0],camerapos[1],camerapos[2],camerayaw*180/3.1415);
			}

		}
		else printf("ERROR not enough distinct features found! \n")	;


	}

	//out time
	/* get elapsed time */
	  if (clock_gettime(CLOCK_MONOTONIC, &current)) {
	      /* getting clock time failed, what now? */
	       exit(EXIT_FAILURE);
	   }
	   /* Elapsed time in nanoseconds */
	  elapsed = current.tv_sec*NANOS + current.tv_nsec - start;
	  microseconds = elapsed / 1000 + (elapsed % 1000 >= 500); // round up halves

	  /* Display time in microseconds or something */
	  printf("elapsed time %lld \n",microseconds);

}
