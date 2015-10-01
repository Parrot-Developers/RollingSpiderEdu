#!/bin/bash

# 0. Initialize
# 0.1 Exit the script if a command fails
set -e

# 1. change folder to where this script is
cd `dirname $0`

# 2. Telnet into drone, move firmware files and set permissions
echo "Drone: Trying to initialize Custom firmware on the drone..."

echo "> Drone: Moving files and setting permissions."
/usr/bin/expect -c 'set timeout 10; spawn telnet 192.168.1.1; expect "RS.edu] \$ "; send "mv /data/edu/dragon-prog /usr/bin/; chmod +x /usr/bin/dragon-prog; mv /data/edu/SpiderFlight.sh /bin/; chmod +x /bin/SpiderFlight.sh; exit\r"; expect eof'

echo "Drone: Custom firmware initialized on the drone!"
