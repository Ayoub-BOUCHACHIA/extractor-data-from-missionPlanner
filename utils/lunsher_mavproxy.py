import os

def os_lunsher_mavproxy():
    os.system("xterm -e mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out 127.0.0.1:14552 &")

os_lunsher_mavproxy()
