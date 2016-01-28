#!/bin/bash

set -e
# 1. change folder to where this script is
cd `dirname $0`
SCRIPTPATH=$(pwd)

# 2. Build ReferenceValueServer
echo "Drone: Trying to build ReferenceValueServer..."
cd ../../trunk/utils/ReferenceValueServer/Debug/
make clean
make
cd $SCRIPTPATH
echo "Drone: ReferenceValueServer built!"

# 3. Build VisionPrePostProcessor
echo "Drone: Trying to build VisionPrePostProcessor..."
cd ../../trunk/utils/VisionPrePostProcessor/Debug/
make clean
make
cd $SCRIPTPATH
echo "Drone: VisionPrePostProcessor built!"

# 3. Build PackEmbeddedCode
echo "Drone: Trying to build PackEmbeddedCode..."
cd ../../trunk/utils/PackEmbeddedCode/Debug/
make clean
make
cd $SCRIPTPATH
echo "Drone: PackEmbeddedCode built!"

echo "Drone: Utils built!"
