#include "rsedu_vis_helpers.h"


double ldnmrk_pinv[4][4] ={
  {-1.332832232793943,   1.444352015477173,                   0,   0.248354229056776},
  { 1.333819842166676,   1.331270742299216,                   0,   0.376598175511799},
  {-1.298265904748279,  -1.408851462349195,                   0,   0.121688388956872},
  { 1.297278295375545,  -1.366771295427195,                   0,   0.253359206474554}}; //watch out: dimensions x-y switched compared to image matrices!

double intrMatrx_inv[3][3] =           {
		   {0.006551832083928,                   0,  -0.550082531110669},
                   {0,   0.006546560202763,  -0.399495818956720},
                   {0,                   0,   1.000000000000000}};



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

