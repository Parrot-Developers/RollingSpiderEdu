#!/bin/sh

#1 Make sure incomming connection allowed 
route add -net 0.0.0.0 netmask 0.0.0.0 gw 192.168.1.1 dev bnep

#2 Make sure folders for storing data exist
if ! [ -p /tmp/of_fifo ]
then
   mkfifo /tmp/of_fifo
fi
   
if ! [ -p /tmp/vis_fifo ]
then
   mkfifo /tmp/vis_fifo
fi
      
if [ ! -d "/data/edu/imgs" ]
then
  mkdir /data/edu/imgs
fi

if [ ! -d "/data/edu/ptimes" ]
then      
  mkdir /data/edu/ptimes    
fi

if [ ! -d "/data/ftp/internal_000/imgs" ]
then      
  mkdir /data/ftp/internal_000/imgs    
fi

#3 Run dragon-prog
dragon-prog

#4 Shut down and clean up
echo " "
echo "Shut down and clean up"    
echo "----------------------" 
test-SIP6_pwm -S 3 0 0 0 0
gpio 39 -d ho 1
      
#red light on
#green light off
gpio 33 -d ho 1
gpio 30 -d ho 1
gpio 31 -d ho 0
gpio 32 -d ho 0
      
# Make sure all data produced by Dragon are written
echo "Flushing filesystems..."
sync; echo "3" > /proc/sys/vm/drop_caches
echo "    flush done."
      

echo "Dragon returned = $PROG_RESULT"

echo "############################"

if [ -f /tmp/edu/imgs/img12.bin ]
then
  mv /tmp/edu/imgs/* /data/edu/imgs
  echo "Recorded imgs moved from tmp to ftp-folder"
  # mv /tmp/edu/imgs/* /data/ftp/internal_000/imgs
  # echo "Recorded imgs moved from tmp to usb-folder"		
fi

if [ -f /tmp/edu/ptimes/pt_RSEDU_control.txt ]
then
  mv /tmp/edu/ptimes/* /data/edu/ptimes
  echo "Recorded ptimes moved from tmp to ftp-folder"
fi

if [ -f /data/edu/RSdata.mat ]
then
  cp /data/edu/RSdata.mat /data/ftp/internal_000/
  echo "Recorded RSdata.mat copied from ftp-folder to usb-folder"
fi

