#!/bin/bash

# 1. Change directory to where this script is
cd `dirname $0`

# 2. Wait for DroneTest or run to be started once
while [ -z "$(ps ax | grep '[D]roneTest')" ] && [ -z "$(ps ax | grep '[D]roneRun')" ]; do
 sleep 0.01
done

# 3. Loop checking for breakdown of DroneTest or DroneRun
while /bin/true; do
  if [ -z "$(ps ax | grep '[D]roneTest')" ] && [ -z "$(ps ax | grep '[D]roneRun')" ] && [ -n "$(ps ax | grep '[D]roneKeyboardPilot')" ]; then
	/usr/bin/expect -c 'set timeout -1; spawn telnet 192.168.1.1; expect "RS.edu] \$ "; send "killall -s SIGKILL dragon-prog; gpio 39 -d ho 1; test-SIP6_pwm -S\r"; expect "RS.edu] \$ ";send "exit\r"; expect eof'

	echo "Drone: Drone-script on drone got killed, shutting down motors and programs!"
	pkill -f "DroneKeyboardPilot*"
	pkill -f "ReferenceValueServer*"
	exit
  fi
done
