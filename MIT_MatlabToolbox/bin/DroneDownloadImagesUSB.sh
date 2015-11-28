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

# 3. copy move images from drone USB to DroneExchange
echo "Drone: Trying to download images from drone to DroneExchange!"

mkdir ../DroneExchange/imgs
mv /media/${USER}/Parrot_RS/imgs/* ../DroneExchange/imgs

echo "Drone: images downloaded from drone to DroneExchange!"
