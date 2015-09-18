//File: main_offboard_image.c
/*
* AUTHOR Fabian Riether
* CREATE DATE 2015/08/25
* PURPOSE This code generates images from the binary-file images saved on the drone and reconstructs the camera pose if enough landmarks where found.
* The lookuptable to match pixels to landmarks can also be generated.
* SPECIAL NOTES helper functions are taken from lecture material for "Imaging Science" by Prof. A. Bruhn, University of Stuttgart, Summer Term 2012
* ===============================
* Change History
* 2015/08/25 created
* ==================================
*/

#include <stdio.h>
#include "image_module.h"
#include <stdlib.h>


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

	      Y[i][j] =        (0.1277   * R[i][j]
	                   +   0.5212   * G[i][j]
	                   +   0.21   * B[i][j]) + 16.0;

	      U[i][j] =   ((-0.0737 * R[i][j] - 0.3007 * G[i][j] + 0.3744 * B[i][j])) + 128.0;
	      V[i][j] =   (( 0.5334 * R[i][j] - 0.3802 * G[i][j] - 0.1532 * B[i][j])) + 128.0;

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

	/*
	red 	=  1.1644*(Y[i][j]-16.0) + 1.596*(V[i][j]-128.0);
	green 	=  1.1644*(Y[i][j]-16.0) - 0.813*(U[i][j]-128.0) - 0.391*(V[i][j]-128.0);
	blue 	=  1.1644*(Y[i][j]-16.0) + 2.018*(U[i][j]-128.0);

	rgbmax = red;
	(rgbmax < green) && (rgbmax = green);
	(rgbmax < blue) && (rgbmax = blue);

	rgbmin = red;
	(rgbmin > green) && (rgbmin = green);
	(rgbmin > blue) && (rgbmin = blue);

	//H
	if (rgbmax == rgbmin)
		{
		H[i][j] = 0.0;
		}
	else if (rgbmax==red)
		{
		H[i][j] = 60*(0+(green-blue)/(rgbmax-rgbmin));
		}
	else if (rgbmax==green)
		{
		H[i][j] = 60*(2+(blue-red)/(rgbmax-rgbmin));
		}
	else
		{
		H[i][j] = 60*(4+(red-green)/(rgbmax-rgbmin));
		}

	if  (H[i][j]<0) {H[i][j]=H[i][j]+360.0;}

	//S, V
	(rgbmax==0) ? S[i][j]=0 : (S[i][j]=(rgbmax-rgbmin)/rgbmax);
	Va[i][j] = rgbmax/255;
	*/
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







int main() {

printf("Simple test environment for image processing on Rolling spider \n");

 //image fake data
 pixel2_t bufferimage[160*120];
 pixel2_t fileimage[80*120];

 //optical flow fake data
float vx = 2;
float vy = 3;
float vz = 4;
float qualityIndicator = 1;
int defined = 1;

//image variables
char   row[80];              /* for reading data */
char   in[80];               /* for reading data */
char   out[80];              /* for writing data */
float  **R;                  /* Red channel of RGB image */
float  **G;                  /* Green channel of RGB image */
float  **B;                  /* Blue channel of RGB image */
float  **Y;                  /* Y channel of YUV image */
float  **U;                  /* U channel of YUV image */
float  **V;                  /* V channel of YUV image */
float  **H;                  /* H channel of HSV image */
float  **S;                  /* S channel of HSV image */
float  **Va;                 /* Value channel of HSV image */
long   nx =0, ny=0;          /* image size in x, y direction */
int kimg;					 /*current image k*/

float yuv[3];
float rgb[3];
float hsv[3];
FILE   *inimage, *outimage;  /* input file, output file */
unsigned char byte;          /* for data conversion */

char filename[20];
char filetype[20];
char ffilename[20];


//helper variables
long   i, j;                 /* loop variables */
int lastImg;
int programMode;





//generate Lookup table for Pixel-Landmark-matching
//---------
printf("Choose program mode: (1) generate lookup table only, (2) process downloaded images only, (3) do 1 & 2...\n");
scanf("%i", &programMode);


if ((programMode==1) || (programMode==3))
{
	printf("Generate lookup...\n");
	createMatchLookup();
	printf("Generate lookup... DONE \n");
}


//read test images
//---------

if ((programMode==2) || (programMode==3))
{

if (programMode==2) {
	printf("Note that lookuptable.dat must be available in DroneExchange/params/! \n");
	}

printf("ID of last image to process ? \n");
scanf("%i", &lastImg);
printf ("ID: %d \n",lastImg);


for (kimg=6;kimg<=lastImg;kimg=kimg+6)
	{


	sprintf(filename,"%s%i","../../DroneExchange/imgs/img",kimg);
	sprintf(filetype,"%s","bin");
	sprintf(ffilename,"%s.%s",filename,filetype);
	printf(": %s \n",ffilename);



	//open pgm file and read header

	if (filetype[0] == 'p')
	{
		inimage = fopen(ffilename,"r");

		if (inimage = NULL)
			{
			printf("ERROR: no image %s  not found!\n",ffilename);
			}


		fgets (row, 80, inimage);
		fgets (row, 80, inimage);
		while (row[0]=='#') fgets(row, 80, inimage);
		sscanf (row, "%ld %ld", &nx, &ny);
		printf("image size (nx,ny)=(%i,%i) \n",nx,ny);
		fgets (row, 80, inimage);

		if (nx==160)
			printf("WARNING: Image should be similar to 160x120 YUV422 interlaced, u.e. 80x120!\n");

		//allocate storage
		alloc_matrix (&R, nx+2, ny+2);
		alloc_matrix (&G, nx+2, ny+2);
		alloc_matrix (&B, nx+2, ny+2);
		alloc_matrix (&Y, nx+2, ny+2);
		alloc_matrix (&U, nx+2, ny+2);
		alloc_matrix (&V, nx+2, ny+2);
		alloc_matrix (&H, nx+2, ny+2);
		alloc_matrix (&S, nx+2, ny+2);
		alloc_matrix (&Va, nx+2, ny+2);

		//read image data
		for (j=0; j<ny; j++)
		 for (i=0; i<nx; i++)
		   {
			 R[i][j] = (float) getc (inimage);
			 G[i][j] = (float) getc (inimage);
			 B[i][j] = (float) getc (inimage);
		   }
		fclose(inimage);

	}

	//bin image, saved from drone
	else
	{
		 FILE* data;
		  if ((data = fopen(ffilename, "rb")) == NULL)
			{
				printf("ERROR opening file!\n");
				return 1;
			}

			nx=80;
			ny=120;


			fread(fileimage, sizeof(pixel2_t) *nx*ny, 1, data);

			fclose(data);

			//allocate storage

			alloc_matrix (&R, nx+2, ny+2);
			alloc_matrix (&G, nx+2, ny+2);
			alloc_matrix (&B, nx+2, ny+2);
			alloc_matrix (&Y, nx+2, ny+2);
			alloc_matrix (&U, nx+2, ny+2);
			alloc_matrix (&V, nx+2, ny+2);
			alloc_matrix (&H, nx+2, ny+2);
			alloc_matrix (&S, nx+2, ny+2);
			alloc_matrix (&Va, nx+2, ny+2);

			//read image data
			for (j=0; j<ny; j++)
			{
			 for (i=0; i<nx; i++)
			   {
				 Y[i][j] = (float)fileimage[nx*j+i].y1; //noneg yuv! transform to 0 centerd uav by (-16,-128,-128)
				 U[i][j] = (float)fileimage[nx*j+i].u;  //noneg yuv!
				 V[i][j] = (float)fileimage[nx*j+i].v;  //noneg yuv!


				 //conversion

				 yuv[0] = Y[i][j];
				 yuv[1] = U[i][j];
				 yuv[2] = V[i][j];

				 YUVtoHSV(yuv,rgb,hsv); //this function takes non-zero yuv!

				 H[i][j] = hsv[0];
				 S[i][j] = hsv[1];
				 Va[i][j] = hsv[2];

				 R[i][j] = rgb[0];
				 G[i][j] = rgb[1];
				 B[i][j] = rgb[2];
			   }

			}

	}



	// analysis image

	printf("YUV Analysis \n");
	analyse_matrix(Y,nx,ny);
	analyse_matrix(U,nx,ny);
	analyse_matrix(V,nx,ny);
	printf("RGB Analysis \n");
	analyse_matrix(R,nx,ny);
	analyse_matrix(G,nx,ny);
	analyse_matrix(B,nx,ny);
	printf("HSV Analysis \n");
	analyse_matrix(H,nx,ny);
	analyse_matrix(S,nx,ny);
	analyse_matrix(Va,nx,ny);


	// generate all channels for read image

	RGB_to_YUV(R, G, B, Y, U, V, nx, ny);
	YUV_to_HSVimg(Y, U, V, H,S,Va,nx, ny);

	// check conversions

	printf("YUV Analysis \n");
	analyse_matrix(Y,nx,ny);
	analyse_matrix(U,nx,ny);
	analyse_matrix(V,nx,ny);
	printf("RGB Analysis \n");
	analyse_matrix(R,nx,ny);
	analyse_matrix(G,nx,ny);
	analyse_matrix(B,nx,ny);
	printf("HSV Analysis \n");
	analyse_matrix(H,nx,ny);
	analyse_matrix(S,nx,ny);
	analyse_matrix(Va,nx,ny);


	// generate test buffer image from YUV channel of image read from file

	for (j=0;j<ny;j++)
	{
		 for (i=0;i<nx;i++)
		 {
			 bufferimage[nx*j+i].y1 = (u8)(Y[i][j]);
			 bufferimage[nx*j+i].u  = (u8)(U[i][j]);
			 bufferimage[nx*j+i].v  = (u8)(V[i][j]);
		 }
	}


	 // init variable to store pixelmatches to landmarks

	int matchResult[80][120] = { {0} };

	if ((nx==80) || (ny==120))
	{
		RSEDU_image_processing_OFFBOARD(bufferimage,matchResult,kimg);
	}

	//save images
	char outfilename[20];

	//...	rgb
	sprintf(&outfilename,"../../DroneExchange/imgs/processed/img_rgb_%i.ppm",kimg);
	writeImgChannelstoFile(outfilename,R,G,B,nx,ny,in);

	//...	match result
	sprintf(&outfilename,"../../DroneExchange/imgs/processed/img_matched_%i.ppm",kimg);
	writeIntArrtoFile(outfilename,matchResult,matchResult,matchResult,nx,ny,in);


	// disallocate storage

	disalloc_matrix (R, nx+2, ny+2);
	disalloc_matrix (G, nx+2, ny+2);
	disalloc_matrix (B, nx+2, ny+2);
	disalloc_matrix (Y, nx+2, ny+2);
	disalloc_matrix (U, nx+2, ny+2);
	disalloc_matrix (V, nx+2, ny+2);
	disalloc_matrix (H, nx+2, ny+2);
	disalloc_matrix (S, nx+2, ny+2);
	disalloc_matrix (Va, nx+2, ny+2);

} //end for over multiple images

//Run dummy 60Hz call of oc loop
/* while (1)
{
 //RSEDU_optical_flow(vx, vy, vz,defined,qualityIndicator);
 RSEDU_image_processing(bufferimage);
 usleep(16700);
}
*/
}
 return 0;

}
