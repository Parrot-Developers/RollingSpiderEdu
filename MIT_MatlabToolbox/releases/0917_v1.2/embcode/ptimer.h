//File: ptimer.h
/*
* AUTHOR Fabian Riether
* CREATE DATE 2015/08/25
* PURPOSE include this headerfile to enable timing functionality for functions repeatedly called by the drone's operating system
* SPECIAL NOTES
* ===============================
* Change History
* 2015/08/25 created
* ==================================
*/

 /* USAGE
 * -------
 *1) After variable declarations in function, add:

 		//ptiming - declare and start
		//------------
		long long start;
		static FILE *ptfile;
		ptimer_start(FEAT_TIME,counter,start);
		//------------

*2) In first function cycle (counter=1), add

 		//ptiming - init file
		//------------
		 ptimer_init(FEAT_TIME,__func__,&(ptfile),NULL);
		//------------

 *3) At end of function, add

 	 	 //ptiming - store
		 //----------
		 ptimer_stopstore(FEAT_TIME, counter,start, ptfile);
		 //----------

 */


#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>
#include <stdio.h>

#define NANOS 1000000000LL


static void ptimer_init(int FEAT_TIME, const char funname[], FILE **ptfilepntr,int *run_flag)
{
	char fullfile[50];

	if (FEAT_TIME)
					{
					mkdir("/tmp/edu", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
					mkdir("/tmp/edu/ptimes", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
					sprintf(fullfile,"/tmp/edu/ptimes/pt_%s.txt",funname);
					//printf("Fullpath for ptimes: %s \n",fullfile);
					*ptfilepntr = fopen(fullfile,"wb");


					if (*ptfilepntr == NULL)
					{
						printf("Error opening ptimes file! \n");
						if (run_flag!=NULL)
							{
								*run_flag=0;
							}
						return;
					}
					else
					{
						fclose(*ptfilepntr);
						*ptfilepntr = fopen(fullfile,"a");
					}


				}
	
}


static void ptimer_start(int FEAT_TIME, int counter,long long *start)
{
	struct timespec begin;

	if ((FEAT_TIME) && (counter>1))
	{
			/* set up start time data */
			if (clock_gettime(CLOCK_MONOTONIC, &begin)) {
			/* Oops, getting clock time failed */
			exit(EXIT_FAILURE);
			}
			/* Start time in nanoseconds */
		    *start = begin.tv_sec*NANOS + begin.tv_nsec;
	}
}



static void ptimer_stopstore(int FEAT_TIME, int counter,long long start, FILE *ptfilepntr)
{
	struct timespec current;
	long long end, elapsed, microseconds;



	if ((FEAT_TIME) && (counter>1))
			{
				//-------------
				/* get elapsed time */
				  if (clock_gettime(CLOCK_MONOTONIC, &current)) {
				      /* getting clock time failed, what now? */
				       exit(EXIT_FAILURE);
				   }
				   /* Elapsed time in nanoseconds */
				  end = current.tv_sec*NANOS + current.tv_nsec;
				  elapsed =  end - start;
				  microseconds = elapsed / 1000 + (elapsed % 1000 >= 500); // round up halves

				  /* Display/print time in microseconds or something */
				  fprintf(ptfilepntr,"%lld, %lld, %lld \n",start,end,microseconds);
			}
}

