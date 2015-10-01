cd `dirname $0`
# 1.1 Exit the script if a command fails
set -e

echo "Drone: Trying to write drone MAC address to DroneMACaddress.txt..."
echo $1 > ../params/DroneMACaddress.txt
echo "Drone: drone MAC address written to DroneMACaddress.txt!"
