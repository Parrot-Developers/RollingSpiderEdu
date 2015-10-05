#!/bin/bash

# 1. Change directory to where this script is
cd `dirname $0`

# 2. Cleanup
pkill -f "DroneWatchDog*"
pkill -f "ReferenceValue*"

# 3. Start Watchdog
./DroneWatchDog.sh&

# 4. Start the server app
./utils/ReferenceValueServer
