#!/bin/bash

# 1. change folder to where this script is
cd `dirname $0`

# 2. ftp into the drone and get the data
echo "Drone: Trying to download images from drone to DroneExchange..."
echo "> Drone: FTP into the drone"
/usr/bin/expect <<SCRIPT
set timeout -1;
spawn ftp 192.168.1.1;
expect "(192.168.1.1:$USER):";
send "\r";
expect "ftp>";
send "cd imgs/\r";
expect "ftp>";
send "prompt n\r";
expect "ftp>";
send "mget img* . \r";
expect "ftp>";
send "exit\r";
expect eof
SCRIPT



# 3. Move the data file to the correct location
IMGSDIR="../DroneExchange/imgs"

if [ ! -d "$IMGSDIR" ]; then
  mkdir $IMGSDIR
fi

mv img* $IMGSDIR

echo "Drone: imgs downloaded from drone to DroneExchange/imgs!"
