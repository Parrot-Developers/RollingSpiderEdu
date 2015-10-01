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

# 3. copy custom firmware to drone
echo "Drone: Trying to upload custom firmware system file to drone..."

cp ../../libs/EDUfirmwareSYS/rollingspider.edu.plf /media/${USER}/Parrot_RS/

echo "Drone: Custom firmware system file uploaded to drone!"
