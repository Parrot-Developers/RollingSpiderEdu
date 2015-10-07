#!/bin/bash

# 1. Initialize
# 1.1 Activate super user
if [ $EUID != 0 ]; then
    sudo "$0" "$@"
    exit $?
fi

# 2. Change directory to where this script is
cd `dirname $0`

# 3. Check if Drone already connected
IP='192.168.1.1'
ping -c1 -t1 $IP &> /dev/null ;
if [ "$?" = 0 ]
then
  echo "Drone: Drone seems to be connected. You might want to try a DroneDisconnect.sh first."; exit 0;
else
  echo " "
fi

# 3. Read the Drone MAC address
echo "> Drone: Trying to read drone MAC address from file..."
read line < ../params/DroneMACaddress.txt
if [[ -z "$line" ]]
then
	echo "> Drone: ERROR: No drone MAC address was set."
	exit 1
fi

# 4. Establish Bluetooth connection
echo "> Drone: Attempting connection with address: $line..."
sudo pand --connect $line -dGN -n || { echo "> Drone: ERROR: Bluetooth connection could was not established."; exit 1; }
ifconfig bnep0 192.168.1.2 up || { echo "> Drone: ERROR: Bluetooth connection could was not established."; exit 1; }
echo "> Drone: Connected to Drone with MAC address $line."

