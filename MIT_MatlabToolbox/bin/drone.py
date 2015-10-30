#! /usr/bin/python
"""
Usage:
    drone.py connect
    drone.py disconnect [--all]
    drone.py scan
    drone.py telnet
"""
from __future__ import print_function
import subprocess
import os

import docopt

class DroneError(Exception):
    pass

DEVNULL = open(os.devnull, 'wb')
params = os.path.join(os.path.dirname(__file__), '../params')

mac_address_fname = os.path.join(params, 'DroneMACaddress.txt')
drone_ip = '192.168.1.1'

def get_mac_address():
    try:
        with open(mac_address_fname) as f:
            mac_address = f.read().strip()
    except IOError:
        mac_address = None

    if not mac_address:
        raise DroneError("No drone MAC address was set.")

    return mac_address

def connect():
    if subprocess.call(['ping', '-c1', '-t1', drone_ip], stdout=DEVNULL) == 0:
        print("Drone: Drone seems to be connected. You might want to try a DroneDisconnect.sh first.")
        return

    mac_address = get_mac_address()

        # 4. Establish Bluetooth connection
    print("> Drone: Attempting connection with address: {}...".format(mac_address))

    try:
        subprocess.check_call(['sudo', 'pand', '--connect', mac_address, '-dGN', '-n'])
    except subprocess.CalledProcessError:
        raise DroneError("Bluetooth connection could was not established.")

    try:
        subprocess.check_call(['sudo', 'ifconfig', 'bnep0', '192.168.1.2', 'up'])
    except subprocess.CalledProcessError:
        raise DroneError("Bluetooth connection could was not established.")

    print("> Drone: Connected to Drone with MAC address {}.".format(mac_address))


def scan():
    output = subprocess.check_output(['hcitool', 'scan'])
    lines = output.split('\n')[1:-1]
    devices = [line.split() for line in lines]

    try:
        curr_mac_address = get_mac_address()
    except:
        curr_mac_address = null

    print("Found {} devices".format(len(devices)))
    for i, (mac_address, name) in enumerate(devices, 1):
        line = '  {}. {:17} {:20}'.format(i, mac_address, name)
        if mac_address == curr_mac_address:
            line += '    SELECTED'
        print(line)


    res = raw_input("Enter a number to change the selected drone:")
    try:
        mac_address, name = devices[int(res) - 1]
    except (ValueError, IndexError):
        return


    with open(mac_address_fname) as f:
        f.write(mac_address)

    print('Future connections will go to {} ({})'.format(name, mac_address))



def disconnect(all):
    mac_address = get_mac_address()

    if all:
        subprocess.all(['sudo', 'pand', '--killall'])
        return

    try:
        subprocess.check_call(['sudo', 'pand', '--kill', mac_address, '-dGN', '-n'])
    except subprocess.CalledProcessError:
        raise DroneError("Could not disconnect")

def telnet():
    subprocess.call(['telnet', drone_ip])

if __name__ == '__main__':
    opts = docopt.docopt(__doc__)

    try:
        if opts['connect']:
            connect()
        elif opts['disconnect']:
            disconnect(opts['--all'])
        elif opts['scan']:
            scan()
        elif opts['telnet']:
            telnet()
    except DroneError as e:
        raise SystemExit(e.message)
