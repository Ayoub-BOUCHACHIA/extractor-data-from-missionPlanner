
from utils.lunsher import *
import time
import os, signal
from subprocess import check_output

def get_pid(name):
    return check_output(["pidof",name])

def mission(id,pos_list):
    os.system('clear')
    print('lunsh drone kit sitl ...')
    pos_start = pos_list[0]
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
    lunsh_mission(id,pos_list)

    print('terminate process ...')

    list_pis = get_pid('xterm')
    for pid in list_pis.split(" "):
        pid_ls = pid.replace('\n','')
        print("pid : ", pid_ls)
        os.system('kill '+pid_ls)

    print('Done ...')


import random
import math
alt_range = {
    'alt_min':10,
    'alt_max':15
}

i = 1



pos_start_1 = "36.715946684015414,3.1843998095598245,0," + str(random.randint(alt_range['alt_min'], alt_range['alt_max']))
pos_start_2 = "36.7447548596623,3.0699704012954188,0,"  + str(random.randint(alt_range['alt_min'], alt_range['alt_max']))
pos_start_3 = "36.774196019679444,3.056318760155851,0," + str(random.randint(alt_range['alt_min'], alt_range['alt_max']))


region_1 = {
    'long_top': 36.7125,
    'long_bottom': 36.7175,
    'lat_left':3.1815,
    'lat_right': 3.1865
}

region_2 = {
    'long_top': 36.7415,
    'long_bottom': 36.7465,
    'lat_left':3.0665,
    'lat_right': 3.0715
}

region_3 = {
    'long_top': 36.7715,
    'long_bottom': 36.7765,
    'lat_left':3.0535,
    'lat_right': 3.0585
}



list_of_region = [region_1,region_2,region_3]

list_of_pos_start = [pos_start_1,pos_start_2,pos_start_3]

print(list_of_pos_start)

k= 0

while i < 10000:
    
    if i >=3000:
        k = 1

    if i >=6000:
        k = 2

    pos_list = [list_of_pos_start[k]]
    
    region = list_of_region[k]

    number_of_points = random.randint(2,5)

    for j in range(number_of_points):

        pos = ",".join([
            str(random.uniform(region['long_top'], region['long_bottom'])),
            str(random.uniform(region['lat_left'], region['lat_right']))
        ])+ ",0," + str(random.randint(alt_range['alt_min'], alt_range['alt_max']))

        pos_list.append(pos)
    

    print("pos_start : ",list_of_pos_start[k])
    mission(i,pos_list)
    i+=1