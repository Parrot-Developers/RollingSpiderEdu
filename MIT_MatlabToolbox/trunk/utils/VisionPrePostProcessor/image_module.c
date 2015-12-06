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
 * helper functions
 */

//allocates storage for matrix of size nx * ny
void alloc_matrix

     (float ***matrix,  /* matrix */
      long  nx,         /* size in x-direction */
      long  ny)         /* size in y-direction */

{
long i;

*matrix = (float **) malloc (nx * sizeof(float *));
if (*matrix == NULL)
   {
   printf("alloc_matrix: not enough storage available\n");
   exit(1);
   }
for (i=0; i<nx; i++)
    {
    (*matrix)[i] = (float *) malloc (ny * sizeof(float));
    if ((*matrix)[i] == NULL)
       {
       printf("alloc_matrix: not enough storage available\n");
       exit(1);
       }
    }
return;
}

//disallocates storage for matrix of size nx * ny
void disalloc_matrix

     (float **matrix,   /* matrix */
      long  nx,         /* size in x-direction */
      long  ny)         /* size in y-direction */

{
long i;
for (i=0; i<nx; i++)
    free(matrix[i]);
free(matrix);
return;
}


//calculates minimum, maximum, mean and variance of a RGB image
void analyse_matrix

     (float  **myMatrix,          /* Red channel of RGB image */
      long    nx,          /* pixel number in x-direction */
      long    ny)


{
long    i, j;       /* loop variables */
float   help;       /* auxiliary variable */
double  help2;      /* auxiliary variable */
double min, max;

min  = myMatrix[1][1];
max  = myMatrix[1][1];



for (i=0; i<nx; i++)
 for (j=0; j<ny; j++)
     {
     if (myMatrix[i][j] < min) min = myMatrix[i][j];
     if (myMatrix[i][j] > max) max = myMatrix[i][j];
     }

printf("min: %f, max: %f \n",min,max);

return;

}

//rgb to yuv conversion of image
void RGB_to_YUV

     (float  **R,          /* Red channel of RGB image */
      float  **G,          /* Green channel of RGB image */
      float  **B,          /* Blue channel of RGB image */
      float  **Y,          /* Y channel of YCbCr image */
      float  **U,         /* Cb channel of YCbCr image */
      float  **V,         /* Cr channel of YCbCr image */
      long    nx,          /* pixel number in x-direction */
      long    ny)          /* pixel number in y-direction */

{
long    i, j;        /* loop variables */

for (j=0;j<ny;j++)
  for (i=0;i<nx;i++)
  {		
	      /*		
	      Y[i][j] =        (0.1277   * R[i][j]
	                   +   0.5212   * G[i][j]
	                   +   0.21   * B[i][j]) + 16.0;

	      U[i][j] =   ((-0.0737 * R[i][j] - 0.3007 * G[i][j] + 0.3744 * B[i][j])) + 128.0;
	      V[i][j] =   (( 0.5334 * R[i][j] - 0.3802 * G[i][j] - 0.1532 * B[i][j])) + 128.0;
	      */
	       
	      Y[i][j] =        (0.1509   * R[i][j]
	                   +   0.6052   * G[i][j]
	                   +   0.2439   * B[i][j]);

	      U[i][j] =   ((-0.0748 * R[i][j] - 0.3000 * G[i][j] + 0.3748 * B[i][j])) + 128.0;
	      V[i][j] =   (( 0.5320 * R[i][j] - 0.3792 * G[i][j] - 0.1528 * B[i][j])) + 128.0;
	      	

  }
 return;
}


// yuv to hsv. conversion 0<h<360

void YUV_to_HSVimg(float  **Y,
	      float  **U,
	      float  **V,
	      float  **H,
	      float  **S,
	      float  **Va,
	      long    nx,          /* pixel number in x-direction */
	      long    ny)          /* pixel number in y-direction */
{

float red,green,blue;
float rgbmin,rgbmax;
long    i, j;        /* loop variables */
float yuv[3], rgb[3], hsv[3];

for (j=0;j<ny;j++)
  for (i=0;i<nx;i++)
  {

	yuv[0] = Y[i][j];
	yuv[1] = U[i][j];
	yuv[2] = V[i][j];


    YUVtoHSV(&(yuv[0]),&(rgb[0]),&(hsv[0]));


  }

    H[i][j] = hsv[0];
    S[i][j] = hsv[1];
    Va[i][j] = hsv[2];

}


//write image
void writeImgChannelstoFile(char *outfilename, float **C1,float**C2,float **C3,int nx,int ny,char *in)
{
	int i,j;
	unsigned char byte;

	FILE * outimage = fopen (outfilename, "w");
	fprintf (outimage, "P6 \n");
	fprintf (outimage, "# Filtered Image\n");
	fprintf (outimage, "# initial image:  %s\n", in);
	fprintf (outimage, "%ld %ld \n255\n", nx, ny);

	// channels as RGB channels
	for (j=0; j<ny; j++)
	 for (i=0; i<nx; i++)
		 {


		 byte = (unsigned char)(C1[i][j]);
		 //byte = (unsigned char)(matchResult[i][j]);
		 //byte = (unsigned char)(H[i][j]/360*255);
		 fwrite (&byte, sizeof(unsigned char), 1, outimage);

		 byte = (unsigned char)(C2[i][j]);
		 //byte = (unsigned char)(S[i][j]*255);
		 fwrite (&byte, sizeof(unsigned char), 1, outimage);

		 byte = (unsigned char)(C3[i][j]);
		 //byte = (unsigned char)(V[i][j]*255);
		 fwrite (&byte, sizeof(unsigned char), 1, outimage);


		 //  if (H[i][j] < 0.0)
		 //byte = (unsigned char)(0.0);
		 //  else if (H[i][j] > 360.0)
		 //byte = (unsigned char)(255.0);
		 //  else
		 //byte = (unsigned char)(H[i][j]/360*255);
		 //  fwrite (&byte, sizeof(unsigned char), 1, outimage);

		 //  if (S[i][j] < 0.0)
		 //byte = (unsigned char)(0.0);
		 //  else if (S[i][j] > 1.0)
		 //byte = (unsigned char)(1.0*255);
		 //  else
		 //byte = (unsigned char)(S[i][j]*255);
		 //  fwrite (&byte, sizeof(unsigned char), 1, outimage);

		 //  if (Va[i][j] < 0.0)
		 //byte = (unsigned char)(0.0);
		 //  else if (Va[i][j] > 1.0)
		 //byte = (unsigned char)(1.0*255.0);
		 //  else
		 //byte = (unsigned char)(Va[i][j]*255);
		 //  fwrite (&byte, sizeof(unsigned char), 1, outimage);

		 }
	fclose(outimage);
	printf("output image %s successfully written\n\n", outfilename);
}

//write matching result as image
void writeIntArrtoFile(char *outfilename, int C1[80][120],int C2[80][120],int C3[80][120],int nx,int ny,char *in)
{
	int i,j;
	unsigned char byte;

	FILE * outimage = fopen (outfilename, "w");
	fprintf (outimage, "P6 \n");
	fprintf (outimage, "# Filtered Image\n");
	fprintf (outimage, "# initial image:  %s\n", in);
	fprintf (outimage, "%ld %ld \n255\n", nx, ny);

	// channels as RGB channels
	for (j=0; j<ny; j++)
	 for (i=0; i<nx; i++)
		 {


		 byte = (unsigned char)(C1[i][j]);
		 //byte = (unsigned char)(matchResult[i][j]);
		 //byte = (unsigned char)(H[i][j]/360*255);
		 fwrite (&byte, sizeof(unsigned char), 1, outimage);

		 byte = (unsigned char)(C2[i][j]);
		 //byte = (unsigned char)(S[i][j]*255);
		 fwrite (&byte, sizeof(unsigned char), 1, outimage);

		 byte = (unsigned char)(C3[i][j]);
		 //byte = (unsigned char)(V[i][j]*255);
		 fwrite (&byte, sizeof(unsigned char), 1, outimage);


		 //  if (H[i][j] < 0.0)
		 //byte = (unsigned char)(0.0);
		 //  else if (H[i][j] > 360.0)
		 //byte = (unsigned char)(255.0);
		 //  else
		 //byte = (unsigned char)(H[i][j]/360*255);
		 //  fwrite (&byte, sizeof(unsigned char), 1, outimage);

		 //  if (S[i][j] < 0.0)
		 //byte = (unsigned char)(0.0);
		 //  else if (S[i][j] > 1.0)
		 //byte = (unsigned char)(1.0*255);
		 //  else
		 //byte = (unsigned char)(S[i][j]*255);
		 //  fwrite (&byte, sizeof(unsigned char), 1, outimage);

		 //  if (Va[i][j] < 0.0)
		 //byte = (unsigned char)(0.0);
		 //  else if (Va[i][j] > 1.0)
		 //byte = (unsigned char)(1.0*255.0);
		 //  else
		 //byte = (unsigned char)(Va[i][j]*255);
		 //  fwrite (&byte, sizeof(unsigned char), 1, outimage);

		 }
	fclose(outimage);
	printf("output image %s successfully written\n\n", outfilename);
}




/*
 *conversion yuv to hsv. 0<h<360
 */
void YUVtoHSV(float* yuv,float* rgb,float* hsv)
{

float red,green,blue;
float rgbmin,rgbmax;

//https://innovativesolution.wordpress.com/2009/09/10/yuv-to-rgb-and-rgb-to-yuv-conversions/

red  = (yuv[0] - 0.0) + 1.596*(yuv[2] - 128.0);
green= (yuv[0] - 0.0) - 0.813*(yuv[1] - 128.0) - 0.3981*(yuv[2] - 128.0);
blue = (yuv[0] - 0.0) + 2.0172*(yuv[1] - 128.0);

if (red>255)   red=255;
if (green>255) green=255;
if (blue>255)  blue=255;
if (red<0)   red=0;
if (green<0) green=0;
if (blue<0)  blue=0;

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

void createMatchLookup(lndmrk_t* lndmrks,int lndmrk_nr)
{
	int i,j,k;
	float yuv[3],rgb[3],hsv[3];
	unsigned char matchLookuptable[128][128][128];

	//matching-Thresholds, global

	//landmarks
	int lndmrk_best=0,lndmrk_cnd=0;
	float featuredist_best = featdist_thrshld;
	float featuredist_cnd = featdist_thrshld;

	

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
// Image processing / Vision-based pose estimation
//----------------------------------
/*
 * @input buffer Pointer to the current picture seen by the vertical camera
 * Picture is 160x120 pixels in YUYV format, i.e. 80x120 elements of type 'pixel2_t'
 *
 */

void RSEDU_image_processing_OFFBOARD(void * buffer, int matchResult[80][120],int kimg,lndmrk_t *lndmrks,int lndmrk_nr)
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
	int lndmrk_best=0,lndmrk_cnd=0;
	float featuredist_best = featdist_thrshld;
	float featuredist_cnd = featdist_thrshld;

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
		//reset posefile
	  	mkdir("../../DroneExchange/imgs/",S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		mkdir("../../DroneExchange/imgs/processed/",S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
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
		//printf("matches reset!\n");
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
						 if ( (lndmrks[lndmrk_best].weights>15) && (fabs((col+1)-lndmrks[lndmrk_best].px)<20) && (fabs((row+1)-lndmrks[lndmrk_best].py)<20))
								{
								 weight=8;
								}
						 else  {
								 weight=1;
								}

						lndmrks[lndmrk_best].px = lndmrks[lndmrk_best].px*lndmrks[lndmrk_best].weights/(lndmrks[lndmrk_best].weights+weight) + (((double)(col+1))*2)*weight/(lndmrks[lndmrk_best].weights+weight);
						lndmrks[lndmrk_best].py = lndmrks[lndmrk_best].py*lndmrks[lndmrk_best].weights/(lndmrks[lndmrk_best].weights+weight) + ((double)(row+1))*weight/(lndmrks[lndmrk_best].weights+weight);
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
		else printf("WARNING not enough distinct landmarks (colored markers) found for vision-based pose reconstruction! \n")	;


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
