
from utils.lunsher import *
import time
import os, signal
from subprocess import check_output

def get_pid(name):
    return check_output(["pidof",name])

def mission(id,pos_start,pos_end,wrong_way,max_dist):
    os.system('clear')
    print('lunsh drone kit sitl ...')
    proc_drone_kit_sitl = lunsh_drone_kit_sitl(pos_start)
    time.sleep(5)

    os.system('clear')
    print('lunsh mavproxy ...')
    proc_mavproxy = lunsh_mavproxy()
    time.sleep(10)

    os.system('clear')
    print('lunsh mission planner ...')
    proc_mission_planner = lunsh_mission_planner()
    time.sleep(35)

    os.system('clear')
    print('lunsh mission ...')
    lunsh_mission(id,pos_end,wrong_way,max_dist)

    print('terminate process ...')

    list_pis = get_pid('xterm')
    for pid in list_pis.split(" "):
        pid_ls = pid.replace('\n','')
        print("pid : ", pid_ls)
        os.system('kill '+pid_ls)

    print('Done ...')


import random
import math
region = {
    'long_top': 37,
    'long_bottom': 36,
    'lat_left':3,
    'lat_right': 4,
    'alt_min':11,
    'alt_max':14
}
"""
    Latitude: 1 deg => 110.574 km
    Longitude: 1 deg => 111.320*cos(latitude) km
"""

max_dist = 600
"""lat_deg = dist / 110.574
long_deg =  dist / (111.320 * math.cos(lat_deg))
dist_deg = math.sqrt(lat_deg**2+long_deg**2)
"""
i = 185
while i < 1000:
    pos_start = ",".join([
        str(random.uniform(region['long_top'], region['long_bottom'])),
        str(random.uniform(region['lat_left'], region['lat_right']))
        ])+ ",0," + str(random.randint(region['alt_min'], region['alt_max']))

    pos_end = ",".join([
        str(random.uniform(region['long_top'], region['long_bottom'])),
        str(random.uniform(region['lat_left'], region['lat_right']))
        ])+ ",0," + str(random.randint(region['alt_min'], region['alt_max']))

    if i % 2 == 0:
        wrong_way = True
    else:
        wrong_way = False

    print("pos_start : ",pos_start)
    print("pos_end : ",pos_end)
    print('wrong_way : ',wrong_way)
    mission(i,pos_start,pos_end,wrong_way,max_dist)
    i+=1