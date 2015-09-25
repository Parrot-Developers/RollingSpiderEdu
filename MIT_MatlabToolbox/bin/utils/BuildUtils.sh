#!/bin/bash

# 1. change folder to where this script is
cd `dirname $0`
SCRIPTPATH=$(pwd)

# 2. Build ReferenceValueServer
echo $PWD
cd ../../trunk/utils/ReferenceValueServer/Debug/
make clean
make
cd $SCRIPTPATH

# 3. Build VisionPrePostProcessor
echo $PWD
cd ../../trunk/utils/VisionPrePostProcessor/Debug/
make clean
make
cd $SCRIPTPATH

# 3. Build PackEmbeddedCode
echo $PWD
cd ../../trunk/utils/PackEmbeddedCode/Debug/
make clean
make
cd $SCRIPTPATH
