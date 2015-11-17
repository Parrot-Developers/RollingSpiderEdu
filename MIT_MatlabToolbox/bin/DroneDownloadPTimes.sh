#!/bin/bash

# 1. change folder to where this script is
cd `dirname $0`

# 2. ftp into the drone and get the data
echo "Drone: Trying to download ptimes from drone to DroneExchange..."

echo "> Drone: FTP into the drone"
/usr/bin/expect <<SCRIPT
set timeout -1;
spawn ftp 192.168.1.1;
expect "(192.168.1.1:$USER):";
send "\r";
expect "ftp>";
send "cd ptimes/\r";
expect "ftp>";
send "prompt n\r";
expect "ftp>";
send "mget pt* . \r";
expect "ftp>";
send "exit\r";
expect eof
SCRIPT


# 3. Move the data file to the correct location
PTDIR="../DroneExchange/ptimes"

if [ ! -d "$PTDIR" ]; then
  mkdir $PTDIR
fi

mv pt_* $PTDIR

echo "Drone: ptimes downloaded from drone to DroneExchange/ptimes!"
