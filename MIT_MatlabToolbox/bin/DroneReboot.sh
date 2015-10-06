#!/bin/bash

# 1.1. Activate superuser
if [ $EUID != 0 ]; then
    sudo "$0" "$@"
    exit $?
fi
# 1.2. change folder to where this script is
cd `dirname $0`

# 2. telnet into the drone and reboot
echo "> Drone: Rebooting the drone remotely."
/usr/bin/expect <<SCRIPT
set timeout 10;
spawn telnet 192.168.1.1;
expect "RS.edu] \$ ";
send "reboot; exit\r";
expect eof
SCRIPT

# 3. Close all bluetooth connections
echo "> Drone: Closing the bluetooth connection..."
sudo pand --killall
echo "> Drone: Bluetooth connection closed!"


