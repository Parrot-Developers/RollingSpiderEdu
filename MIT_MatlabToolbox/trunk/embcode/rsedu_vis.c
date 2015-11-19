//File: rsedu_vis.c
/*
* AUTHOR Fabian Riether
* CREATE DATE 2015/08/25
* PURPOSE This module takes care of processing images to reconstruct the drone's pose (needs specific landmark setup)
* SPECIAL NOTES
* ===============================
* Change History
* 2015/08/25 created
* ==================================
*/
#include "rsedu_vis.h"

/*
 * reconstruct camera pose
 */
void reconstructCameraPose(float camerapos[3], float *camerayaw, float feature_pps[3][4], double reconrightMatrix[4][4], double intrMatrx_inv[3][3])
{
    double LiMatrix[3][4];
    double LiiMatrix[3][4];
    double scalefactor;
    int c, d, k;
    int m = 3;
    int q = 4;
    int p = 4;
    double sum = 0.0;

    for(c = 0; c < m; c++)
    {
        for(d = 0; d < q; d++)
        {
            for(k = 0; k < p; k++)
            {
                sum = sum + feature_pps[c][k] * reconrightMatrix[k][d];
            }
            LiMatrix[c][d] = sum;
            sum = 0;
        }
    }

    for(c = 0; c < m; c++)
    {
        for(d = 0; d < q; d++)
        {

            for(k = 0; k < m; k++)
            {
                sum = sum + intrMatrx_inv[c][k] * LiMatrix[k][d];
            }
            LiiMatrix[c][d] = sum;
            sum = 0;
        }
    }



    scalefactor = 1 / sqrt(pow(LiiMatrix[0][0], 2) + pow(LiiMatrix[0][1], 2));

    camerapos[0] = (float)(LiiMatrix[1][3] * scalefactor);
    camerapos[1] = (float)(-LiiMatrix[0][3] * scalefactor);
    camerapos[2] = (float)(LiiMatrix[2][3] * scalefactor);

    //Yaw: correct to output x-axis(RS)alignment iwth x-axis landmarkfield. z-axis facing down, rotation +-pi
    float cosa = (LiiMatrix[0][0]) * scalefactor;
    float sina = -(LiiMatrix[0][1]) * scalefactor;
    if(cosa > 0)
    {
        if(sina < 0)
        {
            *camerayaw    = (float)(acos(cosa) - 1.571);
        }
        else
        {
            *camerayaw    = (float)(-acos(cosa) - 1.571);
        };

    }
    else
    {
        if(sina < 0)
        {
            *camerayaw    = (float)(acos(cosa) - 1.571);
        }
        else
        {
            *camerayaw    = (float)(-acos(cosa) + 1.571 * 3);
        };
    };


}


//----------------------------------
// Image processing / Vision-based pose estimation
//----------------------------------

/*
 * @input buffer Pointer to the current picture seen by the vertical camera
 * Picture is 160x120 pixels in YUYV format, ie 80x120 elements of type 'pixel2_t'
 * This functions then dumps a planar camera position estimate relative to a colored landmark setup into a fifo to make them available to the control code
 *
 * Called 60 times per second.
 */

void RSEDU_image_processing(void * buffer)
{


    //process control
    static int counter = 0;

    //communication
    static float vis_data[4];
    int status;
    static int vis_fifo;

    //image variables
    int i, j;
    int row, col;
    float yuv[3];
    int nx = 80, ny = 120;
    float feature_pps[3][4];
    static unsigned char matchLookup[128][128][128];

    //camera variables
    float camerapos[3]; //@TODO static?
    float camerayaw; 	//@TODO static?

    //Image and Matching Thresholds
    static int filtersize = 5; 			//odd numbered! dummy for potential gaussian filter mask, etc.
    int pxls_ftr_min 	  = 5; 			//minimum required nr of detected pixels per landmark

    //Landmarks, Pose Estimation Matrices
    int lndmrk_nr = 5, lndmrk_best = 0;
    static lndmrk_t lndmrks[5];
    //matrices for reconstructing camera pose by intrMatrx_inv*landmark-pixellocation*ldnmrk_pinv
    static double ldnmrk_pinv[4][4] =
    {
        { -3.093396787626414,   2.082093991671625                   , 0,   0.766805472932779},
        {2.320047590719810 ,  3.438429506246282    ,               0  , 0.174895895300416},
        { -1.737061273051754 , -2.676977989292088    ,               0  , 0.299821534800714},
        {2.510410469958359 , -2.843545508625819     ,              0 , -0.241522903033909}
    };

    static double intrMatrx_inv[3][3] =       {{ 0.006551831768882                   , 0,  -0.550082487527771},
        {0,   0.006546559888686,  -0.399495805347318},
        {0,                   0,   1.000000000000000}
    };

    //wait on first call
    if(counter == 1)
    {
        usleep(20000);
    }

    //Init Communication, Streaming
    int fifo;
    if(FEAT_IMSAVE == 2)
    {
        u8 * raw = buffer;
    };
    pixel2_t *image = buffer;  /* Picture is a 160x120 pixels, stored in YUV422 interlaced format - TO BE CHECKED */


    /*
     * PROGRAM
     */


    //ptiming - declare and start
    //------------
    long long start;
    static FILE *ptfile;
    ptimer_start(FEAT_TIME, counter, &(start));
    //------------


    //process control
    counter++;

    if(counter == 1)
    {
        //ptiming - init file
        //------------
        ptimer_init(FEAT_TIME, __func__, &(ptfile), NULL);
        //------------

        printf("rsedu_vis(): Init fifo-communication...\n");

        //open fifo to dump visual position estimates to control code

        if(access("/tmp/vis_fifo", F_OK) != -1)
        {
            printf("rsedu_vis(): SUCCESS POSVIS FIFO exists! \n");

            vis_fifo = open("/tmp/vis_fifo", O_WRONLY);
            if(vis_fifo)
            {
                vis_data[0] = -99.0;
                write(vis_fifo, (float*)(&vis_data), sizeof(vis_data));
                close(vis_fifo);
                printf("rsedu_vis(): SUCCESS opening POSVIS-fifo!\n");
            }
            else
            {
                printf("rsedu_vis(): ERROR opening POSVIS-fifo!\n");
            }
        }
        else
        {
            printf("rsedu_vis(): ERROR opening POSVIS-fifo!\n");
        }


        //load lookuptable for matching process
        if(FEAT_NOLOOK == 0)
        {
            FILE* data;
            if((data = fopen("/data/edu/params/lookuptable.dat", "rb")) == NULL)
            {
                printf("rsedu_vis(): ERROR opening lookupfile \n");
            }

            fread(matchLookup, sizeof(matchLookup), 1, data);
            fclose(data);
        }


    }



    //landmark detection
    //------------

    //1 cycle through image, convert each pixel to hsv, threshold by s and v to find colored balls, match h value to color-closest in "database" (use precomputed lookup), save weighted average pixellocation with database-lndmrkID
    //2 reconstruct pose of camera (3D-position with yaw)


    if((FEAT_POSVIS_RUN) && ((counter % 15) == 0) && (NULL != image)) //@pseudo4Hz
    {
        //reset landmark matching data
        for(i = 0; i < lndmrk_nr; i++)
        {
            lndmrks[i].n  = 0;
            lndmrks[i].weights  = 0;
            lndmrks[i].px = 0;
            lndmrks[i].py = 0;
        }


        int margin = (int)(filtersize - 1) / 4;
        int weight = 1;	// weight used to emphasize pixellocation that is close to existing estimate for sufficiently robust, identified landmark

        //cycle through image
        for(j = 0; j < ny - 2 * margin; j++)
        {
            for(i = 0; i < nx - 2 * margin; i++)
            {
                row = margin + j;
                col = margin + i;
                //Get YUV-values for pixel
                yuv[0] = (float)image[nx * row + col].y1;
                yuv[1] = (float)image[nx * row + col].u;
                yuv[2] = (float)image[nx * row + col].v;

                //find best matching landmark for current pixel
                if(FEAT_NOLOOK)
                {
                    printf("rsedu_vis(): ERROR Unfortunately, color conversion, etc. too slow onboard. Please set FEAT_NOLOOK = 0. \n");
                }
                else
                    //use pre-computed lookup table to find match
                {
                    //printf("here uselu \n");
                    lndmrk_best = (int)matchLookup[(int)(yuv[0] / 2)][(int)(yuv[1] / 2)][(int)(yuv[2] / 2)];
                }


                //Store matched landmarks
                //----
                if(lndmrk_best > 0)
                {
                    //increase weight if close to an existing, highly weighted cluster
                    if((lndmrks[lndmrk_best].weights > 15) && (abs((col + 1) - lndmrks[lndmrk_best].px) < 20) && (abs((row + 1) - lndmrks[lndmrk_best].py) < 20))
                    {
                        weight = 8;
                    }
                    else
                    {
                        weight = 1;
                    }

                    lndmrks[lndmrk_best].px = lndmrks[lndmrk_best].px * lndmrks[lndmrk_best].weights / (lndmrks[lndmrk_best].weights + weight) + (((double)(col + 1)) * 2) * weight / (lndmrks[lndmrk_best].weights + weight); //@TODO is it row?
                    lndmrks[lndmrk_best].py = lndmrks[lndmrk_best].py * lndmrks[lndmrk_best].weights / (lndmrks[lndmrk_best].weights + weight) + ((double)(row + 1)) * weight / (lndmrks[lndmrk_best].weights + weight); //@TODO is it row?
                    //save that one more pixel to this landmark
                    lndmrks[lndmrk_best].n += 1;
                    lndmrks[lndmrk_best].weights += weight;

                    //if (lndmrk_best==0) {matchResult[col][row]=255;} else {matchResult[col][row]=0;}
                }

            }//end inner image-for
        }//end outer image-for

        /*
        printf("image_proc(): Result of matching: \n");
        for (i=0;i<lndmrk_nr;i++)
        {
        	printf("image_proc(): Lndmrk %i: nr pxls matched: %i, pixelcoordinates: (%f,%f) \n",i,lndmrks[i].n,lndmrks[i].px,lndmrks[i].py);
        }
        */

        //reconstruct pose
        //-------------

        // choose lndmrks with some minimum number of pixels, add them to the feature_pixelposition-matrix (ui,vi,1). TODO: remove hardcoded number of landmarks
        int features_valid = 0;
        int k;

        for(k = 1; k < lndmrk_nr; k++) //start from 1 as yellow (landmark 0) not used!
        {
            if(lndmrks[k].n >= pxls_ftr_min)
            {
                features_valid += 1;
                feature_pps[0][features_valid - 1] = lndmrks[k].px;
                feature_pps[1][features_valid - 1] = lndmrks[k].py;
                feature_pps[2][features_valid - 1] = 1.0;
            }

        }

        if(features_valid == 4)
        {
            reconstructCameraPose(camerapos, &camerayaw, feature_pps, ldnmrk_pinv, intrMatrx_inv);
            printf("rsedu_vis(): SUCCESS reconstructed camera pose: (%f, %f, %f,%f)\n", camerapos[0], camerapos[1], camerapos[2], camerayaw * 180 / 3.1415);


            /*
             * dump pose into FIFO to make available to controls code
             */
            //compile data
            vis_data[0] = camerapos[0];
            vis_data[1] = camerapos[1];
            vis_data[2] = camerapos[2];
            vis_data[3] = camerayaw;

            vis_fifo = open("/tmp/vis_fifo", O_WRONLY);
            if(vis_fifo)
            {
                write(vis_fifo, (float*)(&vis_data), sizeof(vis_data));
                close(vis_fifo);
            }
        }
        else
        {
            printf("rsedu_vis(): WARNING not enough distinct markers (colored balls) found! \n")	;
        }

    }

    //-----------
    //STREAMING INSTRUCTIONS
    //-----------

    /* Enabling image streaming, copies the picture into a named FIFO. Picture can then be sent to a remote Ubuntu computer using standard commands:

    Run this one-liner in a shell on the RollingSpider (open terminal, log onto drone via telnet 192.168.1.1) :
    (remember to connect via the Bluetooth link, since pluging the USB cable deactivates the camera !!!)

      while [ 1 ]; do cat /tmp/picture | nc 192.168.1.2 1234; done

    Run these two commands in two different shells on the remote Ubuntu computer:

      mkfifo /tmp/rollingspiderpicture ; while [ 1 ]; do nc -l 1234 > /tmp/rollingspiderpicture; done
      mplayer -demuxer rawvideo -rawvideo w=160:h=120:format=yuy2 -loop 0 /tmp/rollingspiderpicture

    */

    //stream image
    //-----------

    if((FEAT_IMSAVE == 2) && ((counter % 60) == 0) && (NULL != image)) //@pseudo1Hz
    {
        printf("image_proc(): Write image to fifo...\n");
        mkfifo("/tmp/picture", 0777);
        fifo = open("/tmp/picture", O_WRONLY);
        if(fifo)
        {
            //char word = "asd";
            //write(fifo,word,320*120);
            write(fifo, buffer, 320 * 120);
            close(fifo);
            usleep(5000);
        }

    }

    //save image
    //-----------

    if((FEAT_IMSAVE == 1) && ((counter % 6) == 0) && (NULL != image)) //@10Hz
    {
        FILE* data;
        char filename[15];

        //sprintf(filename,"/data/edu/imgs/img%i.bin",counter);
        sprintf(filename, "/tmp/edu/imgs/img%i.bin", counter);
        //sprintf(filename,"/tmp/imgs/img.bin");
        //printf("image_proc(): img name: %s \n",filename);

        mkdir("/tmp/edu", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        mkdir("/tmp/edu/imgs", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if((data = fopen(filename, "wb")) == NULL)
        {
            printf("rsedu_vis(): ERROR opening img file\n");
        }

        fwrite(image, sizeof(pixel2_t) * 80 * 120, 1, data);
        fclose(data);
        usleep(5000);

    }




    usleep(4000);

    //ptiming - store
    //----------
    ptimer_stopstore(FEAT_TIME, counter, start, ptfile);
    //----------


}
