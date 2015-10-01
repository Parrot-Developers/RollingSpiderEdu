#!/bin/bash

# 1. Activate superuser
if [ $EUID != 0 ]; then
    sudo "$0" "$@"
    exit $?
fi

# 2. Close all bluetooth connections
sudo pand --killall

echo "Drone: Killed all bluetooth connections!"
