//File: rsedu_of.c
/*
* AUTHOR Fabian Riether
* CREATE DATE 2015/08/25
* PURPOSE This module takes care of dumping the input optical flow into a FIFO to enable the controls process to read it
* SPECIAL NOTES
* ===============================
* Change History
* 2015/08/25 created
* ==================================
*/

#include "rsedu_of.h"

//----------------------------------
// OPTICAL FLOW
//----------------------------------

/*
 * @input is the result of optical flow computations, this functions then dumps these into a fifo to make them available to the control code
 *
 * Called 60 times per second.
 */
void RSEDU_optical_flow(float vx, float vy, float vz, int defined, float qualityIndicator)
{


    //process control
    static int counter = 0;

    //communication
    static float of_data[5];
    int status;
    static int of_fifo;

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

    //Compile data
    of_data[0] = vx;
    of_data[1] = vy;
    of_data[2] = vz;
    of_data[3] = (float)(defined * 1.0);
    of_data[4] = qualityIndicator;

    //printf("rsedu_of(): flow: (%f, %f, %f, %f, %f)\n",of_data[0],of_data[1],of_data[2],of_data[3],of_data[4]);

    //Setup thread communication
    if(counter == 1)
    {
        //create fifo to transmit optical flow to control code - now done by shell-script that starts a flight
        /*if ((status = mkfifo("/tmp/of_fifo", S_IRWXU)) < 0) {
        	//printf("optical_flow(): ERROR makingf fifo!\n");

        }
        */

        if(access("/tmp/of_fifo", F_OK) != -1)
        {
            printf("rsedu_of(): SUCCESS optical flow FIFO exists! \n");
        }
        else
        {
            printf("rsedu_of(): ERROR opening optical flow-fifo!\n");
        }

        //ptiming - init file
        //------------
        ptimer_init(FEAT_TIME, __func__, &(ptfile), NULL);
        //------------

        //usleep(10000);

    }

    //FIFO handling and writing
    of_fifo = open("/tmp/of_fifo", O_WRONLY);
    if(of_fifo)
    {
        write(of_fifo, (float*)(&of_data), sizeof(of_data));
        close(of_fifo);
    }


    usleep(4000);


    //ptiming - store
    //----------
    ptimer_stopstore(FEAT_TIME, counter, start, ptfile);
    //----------
    //printf("rsedu_of(): cycle %d\n", counter);

}
