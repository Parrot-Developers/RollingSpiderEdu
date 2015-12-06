//File: PackEmbeddedCode.c
/*
* AUTHOR Fabian Riether
* CREATE DATE 2015/09/04
* PURPOSE This program takes the SIMULINK auto-generated c-files for the DroneRS_Compensator (in trunk/matlab/Simulation/DronesRS_Compensator_ert_shrlib_rtw and merges them with the existing c-code for control
* optical flow and image processing (in trunk/embcode)
* SPECIAL NOTES
* ===============================
* Change History
* 2015/09/04 created
* ==================================
*/
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>


int main(int argc, char *argv[])
{

int status;

FILE * fp_rscontrol, *fp_rscontrol_copy, *fp_ertmain_inemb;
char * line = NULL;
size_t len = 0;
ssize_t read;

printf("C-files are being packed...\n");

//1 Copy all files from trunk/matlab/Simulation/DronesRS_Compensator_ert_shrlib_rtw to trunk/embcode
char rtwFilesFolder[100], embFolder[100], rtwmainFileName[20], rtwmainFilePath_inemb[300], rscontrolcopyFilePath_inemb[300], str[300];

sprintf(embFolder,"%s","../../trunk/embcode");
sprintf(rtwFilesFolder,"%s","../../trunk/matlab/Simulation/Drone_Compensator_ert_shrlib_rtw");
sprintf(rtwmainFileName,"%s","ert_main.c");
sprintf(str,"%s/%s",rtwFilesFolder,rtwmainFileName);

if( access( str, F_OK ) != -1 )
	{
		sprintf(str,"cp %s/* %s",rtwFilesFolder,embFolder);
		status = system(str);

		if (status!=0)
			{
			printf("ERROR Copying SIMULINK auto-generated files! Exiting. \n");
					exit(EXIT_FAILURE);
			}
	}

else
	{
	    printf("ERROR auto-generated c-files not found in %s!\n",str);
	    exit(EXIT_FAILURE);
	}


//2 Replace parameter set in rsedu_control.c with new parameters from ert_main.c
sprintf(rscontrolcopyFilePath_inemb,"%s/rsedu_control_copy.c",embFolder);
//create copy of rsedu_control.c
sprintf(str,"cp %s/rsedu_control.c %s",embFolder,rscontrolcopyFilePath_inemb);
status = system(str);

//empty rsedu_control.c,
sprintf(str,"%s/rsedu_control.c",embFolder);
fp_rscontrol = fopen(str, "w"); fclose(fp_rscontrol);
fp_rscontrol = fopen(str, "a");
if (fp_rscontrol_copy == NULL)
	exit(EXIT_FAILURE);


//read rs_control_copy line by line and put into empty s_control until parameter section starts
sprintf(str,"%s/rsedu_control_copy.c",embFolder);
fp_rscontrol_copy = fopen(str, "r");

if (fp_rscontrol_copy == NULL)
	exit(EXIT_FAILURE);

int read_flag = 1;
while  ( ((read = getline(&line, &len, fp_rscontrol_copy)) != -1) && read_flag)
{
	read_flag = (strstr(line,"static RT_MODEL_")==NULL);
	if (read_flag) fprintf(fp_rscontrol,"%s", line);
}

//Open ert_main.c for read, find first line of parameter block, start appending parameters to rsedu_control.
sprintf(rtwmainFilePath_inemb,"%s/%s",embFolder,rtwmainFileName);
fp_ertmain_inemb = fopen(rtwmainFilePath_inemb, "r");

if (fp_ertmain_inemb == NULL)
	exit(EXIT_FAILURE);

int paramSection_found = 0;
while  ( ((read = getline(&line, &len, fp_ertmain_inemb)) != -1))
{

	if (!paramSection_found)
		{
		paramSection_found = (strstr(line,"static RT_MODEL_")!=NULL);
		}
	if (paramSection_found)
		{
		fprintf(fp_rscontrol,"%s", line);
		}

	if	(paramSection_found)
		{
			paramSection_found = (strstr(line,"};                                     /* Modifiable parameters */")==NULL);
		}


}


// Now read copy of rseducontrol till end of parameter section, then start appending rest of that code to rsedu_control
int sectionAfterParams=0;
while  ( ((read = getline(&line, &len, fp_rscontrol_copy)) != -1) )
{
	if (sectionAfterParams) fprintf(fp_rscontrol,"%s", line);
	if (!sectionAfterParams) sectionAfterParams = (strstr(line,"};                                     /* Modifiable parameters */")!=NULL);

}

//close all, delete rsedu_control_copy, delete ert_main.c
fclose(fp_rscontrol_copy);
fclose(fp_rscontrol);
fclose(fp_ertmain_inemb);

remove(rtwmainFilePath_inemb);
remove(rscontrolcopyFilePath_inemb);

printf("C-files packed and ready to be built! \n");

if (line)
	free(line);
exit(EXIT_SUCCESS);


return 0;
}
