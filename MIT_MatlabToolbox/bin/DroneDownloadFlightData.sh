#!/bin/bash

# 1. change folder to where this script is
cd `dirname $0`

# 2. ftp into the drone and get the data
echo "Drone: Trying to download RSdata.mat from drone to DroneExchange..."

echo "> Drone: FTP into the drone"
/usr/bin/expect <<SCRIPT
set timeout -1;
spawn ftp 192.168.1.1;
expect "(192.168.1.1:$USER):";
send "\r";
expect "ftp>";
send "get RSdata.mat\r";
expect "ftp>";
send "exit\r";
expect eof
SCRIPT


# 3. Move the data file to the correct location
mv RSdata.mat ../DroneExchange/RSdata__$(date '+%Y_%m_%d__%H_%M_%S').mat

echo "Drone: RSdata.mat downloaded from drone to DroneExchange!"
