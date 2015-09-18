//File: main_testenviron.c
/*
* AUTHOR Fabian Riether
* CREATE DATE 2015/06/02
* PURPOSE This module is a super simple test environment for the RSEDU_control-function. The function is called at 200Hz with dummy sensor input.
* SPECIAL NOTES
* ===============================
* Change History
* 2015/06/02 created
* ==================================
*/

#include <stdio.h>
#include "rsedu_control.h"

int main() {

printf("Simple test environment for RSedu-code\n");

HAL_acquisition_t in;
HAL_command_t     out;


//Define and init dummy sensor inputs
 in.HAL_acc_SI.x = 0.1;
 in.HAL_acc_SI.y = 0.01;
 in.HAL_acc_SI.z = -9.81;
 in.HAL_gyro_SI.x = 0.01;
 in.HAL_gyro_SI.y = 0.01;
 in.HAL_gyro_SI.z = 0.01;
 in.HAL_magn_mG.x = 0.0;
 in.HAL_magn_mG.y = 0.0;
 in.HAL_magn_mG.z  = 0.0;
 in.HAL_ultrasound_SI.altitude  = 0.5;
 in.HAL_pressure_SI.pressure  = 10000.0;
 in.HAL_pressure_SI.temperature  = 45.0;
 in.timestamp = 5;

 //init out
 out.command = 0;
 int16_t tmp[4] = {0,0,0,0};
 int16_t* first = &tmp[1];
 out.motors_speed = first;


 //Run dummy 200Hz call of control loop
 while (1)
{
  RSEDU_control(&in, &out);
  usleep(5000);
}

 return 0;

}
