#!/bin/bash

# 1. Initialize
# 1.1 Exit the script if a command fails
set -e

# 2. change to dir where this script is
cd `dirname $0`

# 3. Check if drone connected via USB
if [ ! -d "/media/${USER}/Parrot_RS/" ]; then
	echo "ERROR Please connect drone via USB!"
	exit 1
fi

# 3. copy RSdata.mat from drone USB to DroneExchange
echo "Drone: Trying to download RSdata.mat from drone to DroneExchange!"

cp /media/${USER}/Parrot_RS/RSdata.mat ../DroneExchange 

echo "Drone: RSdata.mat downloaded from drone to DroneExchange!"
