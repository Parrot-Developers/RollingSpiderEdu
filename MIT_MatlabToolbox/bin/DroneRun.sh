#!/bin/bash

# 1. change folder to where this script is
cd `dirname $0`

# 2. Telnet into drone, adjust PowerGain in paramsEDU.dat and start the SpiderFlight.sh
/usr/bin/expect <<SCRIPT
set timeout -1;
spawn telnet 192.168.1.1;
expect "RS.edu] \$ ";
send "sed -i \"s/POWERGAIN : \[1-9\]\[0-9\]*;/POWERGAIN : 100;/\" /data/edu/params/paramsEDU.dat\r";
expect "RS.edu] \$ ";
send "killall dragon-prog\r";
expect "RS.edu] \$ ";
send "SpiderFlight.sh\r";
expect "RS.edu] \$ ";
send "exit\r";
expect eof
SCRIPT
