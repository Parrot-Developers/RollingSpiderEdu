import socket
import time
import termios
import sys
import tty

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    new = termios.tcgetattr(fd)
    new[tty.LFLAG] &= ~(termios.ECHO | termios.ICANON)
    new[tty.CC][termios.VMIN] = 1
    new[tty.CC][termios.VTIME] = 0
    try:
        termios.tcsetattr(fd, tty.TCSANOW, new)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, tty.TCSADRAIN, old)

# state
roll_ref = 0
pitch_ref = 0
alt_ref = -1.1
yaw_ref = 0

# Saturation parameters?
SATU_angle = 0.5
SATU_he_min = -1.1
SATU_he_max = -2.7

# connect to socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

s.bind(('', 12345))

print("Waiting for connection to drone...")
s.listen(10)

conn, end_addr = s.accept()
conn.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# process inputs
print("Fly drone with w-s-a-d :: i-k-j-l :: e(xit)!");

runcmd = 1
while runcmd:
    keybch = getch();

    if keybch == 'e': runcmd = 0

    if keybch == 'd': roll_ref += 0.05
    if keybch == 'a': roll_ref -= 0.05
    if keybch == 'w': pitch_ref -= 0.05
    if keybch == 's': pitch_ref += 0.05
    if keybch == 'i': alt_ref -= 0.2
    if keybch == 'y': alt_ref -= 0.6
    if keybch == 'k': alt_ref += 0.2
    if keybch == 'h': alt_ref += 0.6
    if keybch == 'j': yaw_ref -= 0.2
    if keybch == 'l': yaw_ref += 0.2


    # Saturation
    roll_ref_RS = roll_ref
    pitch_ref_RS = pitch_ref

    if roll_ref_RS > SATU_angle: roll_ref_RS = SATU_angle
    if roll_ref_RS < -SATU_angle: roll_ref_RS = -SATU_angle
    if pitch_ref_RS > SATU_angle: pitch_ref_RS = SATU_angle
    if pitch_ref_RS < -SATU_angle: pitch_ref_RS = -SATU_angle

    if alt_ref > SATU_he_min: alt_ref = SATU_he_min
    if alt_ref < SATU_he_max: alt_ref = SATU_he_max


    conn.send("%i %i %i %i %i" % (
        runcmd,
        (int)(pitch_ref_RS*1000 + 10000),
        (int)(roll_ref_RS*1000 + 10000),
        (int)(yaw_ref*1000 + 10000),
        (int)(alt_ref*100.0)
    ))

    time.sleep(0.01)


s.shutdown(2)
conn.shutdown(2)
s.close()
conn.close()