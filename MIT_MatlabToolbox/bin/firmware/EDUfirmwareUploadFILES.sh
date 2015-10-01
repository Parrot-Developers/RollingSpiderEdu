#!/bin/bash

# 1. Initialize
# 1.1 Exit the script if a command fails
set -e

# 2. change to dir where this script is
cd `dirname $0`

# 3. upload custom firmware files to drone using ftp
echo "Drone: Trying to upload custom firmware files..."

echo "> Drone: FTP into the drone"


/usr/bin/expect -c 'set timeout -1; spawn lftp -e "mirror -R ../../libs/EDUfirmwareFILES/ /" -u anonymous, 192.168.1.1; expect "anonymous@192.168.1.1:/>"; send "exit\r"; expect eof'

echo "Drone: Custom firmware files uploaded to drone!"
