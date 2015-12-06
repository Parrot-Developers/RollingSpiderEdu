CC=/opt/arm-2012.03/bin/arm-none-linux-gnueabi-gcc

all:
	$(CC) -c -fPIC -Wall -Werror -o Drone_Compensator.o ../Drone_Compensator.c -lm
	#$(CC) -c -fPIC -Wall -Werror -o Drone_Compensator_capi.o ../Drone_Compensator_capi.c -lm
	$(CC) -c -DDELOS -DDELOS_EDU -fPIC -Wall -Werror -o rsedu_control.o ../rsedu_control.c  -lm
	$(CC) -c -DDELOS -DDELOS_EDU -fPIC -Wall -Werror -o rsedu_vis_helpers.o ../rsedu_vis_helpers.c  -lm
	$(CC) -c -DDELOS -DDELOS_EDU -fPIC -Wall -Werror -o rsedu_vis.o ../rsedu_vis.c  -lm
	$(CC) -c -DDELOS -DDELOS_EDU -fPIC -Wall -Werror -o rsedu_of.o ../rsedu_of.c  -lm
	$(CC) -c -fPIC  -Wall -Werror -o rtGetInf.o ../rtGetInf.c
	$(CC) -c -fPIC  -Wall -Werror -o rtGetNaN.o ../rtGetNaN.c  
	$(CC) -c -fPIC -Wall -Werror -o rt_nonfinite.o ../rt_nonfinite.c
	$(CC) -c -fPIC -Wall -Werror -DDEFAULT_BUFFER_SIZE=4600 -o rt_logging.o ../rt_logging.c 
	#$(CC) -c -fPIC -Wall -Werror -o rt_logging_mmi.o ../rt_logging_mmi.c
	#$(CC) -c -fPIC -Wall -Werror -o rtw_modelmap_utils.o ../rtw_modelmap_utils.c

	#
	$(CC) rsedu_control.o  rsedu_vis_helpers.o rsedu_vis.o rsedu_of.o Drone_Compensator.o rtGetInf.o rtGetNaN.o rt_nonfinite.o rt_logging.o -lm -DDELOS -DDELOS_EDU -fPIC -shared -o ../../../DroneExchange/librsedu.so -I./ 
