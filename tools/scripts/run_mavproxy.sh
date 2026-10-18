#!/bin/bash
set -x
#For forwarding PIXHAWK to GroundControl via IP
#192.168.0.155 is outside minipc device (laptopnya jalu)
# mavproxy.py --master=/dev/ttyACM0,115200 --out=udp:192.168.0.155:14550 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14552 --daemon
mavproxy.py --master=/dev/pixhawk --out=udp:192.168.0.155:14550 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14552 --daemon
