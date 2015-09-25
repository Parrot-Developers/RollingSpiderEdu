#!/bin/bash

# 1. change folder to where this script is
cd `dirname $0`

# 2. ftp into the drone and get the data
echo "> Drone: FTP into the drone"
/usr/bin/expect -c 'set timeout -1; spawn ftp 192.168.1.1; expect "(192.168.1.1:'"$USER"'):";  send "\r"; expect "ftp>";send "cd imgs/\r"; expect "ftp>"; send "prompt n\r"; expect "ftp>"; send "mget img* . \r"; expect "ftp>"; send "exit\r"; expect eof'


# 3. Move the data file to the correct location
mv img* ../DroneExchange/imgs
