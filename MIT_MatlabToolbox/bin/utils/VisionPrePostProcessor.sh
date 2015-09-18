#!/bin/bash

# 1. Change directory to where this script is
cd `dirname $0`

# 2. Start run VisionPrePostProcessor app
cd utils/
./VisionPrePostProcessor
cd ..
