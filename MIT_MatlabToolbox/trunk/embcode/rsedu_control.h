#ifndef RSEDU_CONTROL_H_
#define RSEDU_CONTROL_H_

//control
//-------------------
#include "HAL.h"


#include "Drone_Compensator.h"

#include "rtwtypes.h"
#include <math.h>
#include <inttypes.h>
#include <stddef.h>

//frameworkparameters
//-------------------
#include "rsedu_globalparams.h"

//data logging
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "rt_logging.h"
#define QUOTE1(name)                   #name
#define QUOTE(name)                    QUOTE1(name)              /* need to expand name */
#ifndef SAVEFILE
# define MATFILE2(file)                #file ".mat"
# define MATFILE1(file)                MATFILE2(file)
# define MATFILE                       MATFILE1(/data/edu/RSdata)
#else
# define MATFILE                       QUOTE(SAVEFILE)
#endif

//communication
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>

/* function declarations*/

void RSEDU_control(HAL_acquisition_t* hal_sensors_data, HAL_command_t* hal_sensors_cmd);

#endif /* RSEDU_PARAMS_H_ */
