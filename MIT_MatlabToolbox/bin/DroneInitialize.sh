#!/bin/bash

# 1. change folder to where this script is
cd `dirname $0`

# 2. Findout this computer's IP
myip="$(ifconfig eth0 2>/dev/null|awk '/inet addr:/ {print $2}'|sed 's/addr://')"
echo "> Drone: This computer's IP is: $myip."

# 3. Telnet into drone, and set IP variable in param file
echo "> Drone: Uploading the IP parameter into the drone."
/usr/bin/expect -c 'set timeout 10; spawn telnet 192.168.1.1; expect "RS.edu] \$ "; send "cd /data/edu/params; sed \"1s/.*/IP : '$myip';/\" paramsEDU.dat > paramsEDU.dat.bc; mv paramsEDU.dat.bc paramsEDU.dat; exit\r"; expect eof'

