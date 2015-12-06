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
#include <stdlib.h>
#include "image_module.h"




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

//Init landmarks
int lndmrk_nr = 5;


lndmrk_t* lndmrks = malloc(lndmrk_nr*sizeof(lndmrk_t));

//yellow - not used for localization
lndmrks[0].X = 0.0; 		//landmark positions not needed here; see MATLAB script findPoseReconstructionParameters, it is needed there;
lndmrks[0].Y = 0.0;
lndmrks[0].descr[0] = 70.0;	//HUE
lndmrks[0].descr[1] = 170.0;	//green
lndmrks[0].descr[2] = 100.0;	//blue

//green
lndmrks[1].X = 0.0;
lndmrks[1].Y = 0.0;
lndmrks[1].descr[0] = 130.0;
lndmrks[1].descr[1] = 160.0;
lndmrks[1].descr[2] = 110.0;

//pink
lndmrks[2].X = 0.0;
lndmrks[2].Y = 0.0;
lndmrks[2].descr[0] = 345.0;
lndmrks[2].descr[1] = 100.0;
lndmrks[2].descr[2] = 135.0;

//red
lndmrks[3].X = 0.0;
lndmrks[3].Y = 0.0;
lndmrks[3].descr[0] = 25.0;
lndmrks[3].descr[1] = 80.0;
lndmrks[3].descr[2] = 70.0;

//blue
lndmrks[4].X = 0.0;
lndmrks[4].Y = 0.0;
lndmrks[4].descr[0] = 220.0;
lndmrks[4].descr[1] = 120.0;
lndmrks[4].descr[2] = 160.0;



//generate Lookup table for Pixel-Landmark-matching
//---------
printf("Choose program mode: (1) generate lookup table only, (2) process downloaded images only, (3) do 1 & 2...\n");
scanf("%i", &programMode);


if ((programMode==1) || (programMode==3))
{
	printf("Generate lookup...\n");
	createMatchLookup(lndmrks,lndmrk_nr);
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


for (kimg=6;kimg<=lastImg;kimg=kimg+6)
	{


	sprintf(filename,"%s%i","../../DroneExchange/imgs/img",kimg);
	sprintf(filetype,"%s","bin");
	sprintf(ffilename,"%s.%s",filename,filetype);
	printf("ffilename: %s \n",ffilename);
	


	//open pgm file and read header
	
	if (filetype[0] == 'p')
	{		
		inimage = fopen(ffilename,"r");

		
		if (inimage = NULL)
			{
			printf("ERROR: no image %s  not found!\n",ffilename);
			exit(0);
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
		RSEDU_image_processing_OFFBOARD(bufferimage,matchResult,kimg,lndmrks,lndmrk_nr);
	}
	

	//save images
	mkdir("../../DroneExchange/imgs/processed/",S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	char outfilename[20];

	//...	rgb
	sprintf(&outfilename,"../../DroneExchange/imgs/processed/img_rgb_%04i.ppm",kimg);
	writeImgChannelstoFile(outfilename,R,G,B,nx,ny,in);

	//...	match result
	sprintf(&outfilename,"../../DroneExchange/imgs/processed/img_matched_%04i.ppm",kimg);
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


}
 return 0;

}
