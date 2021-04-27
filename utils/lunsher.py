from __future__ import print_function

from dronekit import connect, VehicleMode
import time
import os 
from utils.process import Drone_manager
import subprocess
import csv

def lunsh_mission_planner():
    #os.system('xterm -e mono ~/Desktop/MissionPlanner-latest/MissionPlanner.exe &')
    proc = subprocess.Popen(['python','/home/abroot/Desktop/extractData/utils/lunsher_mission_planner.py']) 
    return proc

def lunsh_mavproxy():
    #os.system("xterm -e mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out 127.0.0.1:14552 &")
    proc = subprocess.Popen(['python','/home/abroot/Desktop/extractData/utils/lunsher_mavproxy.py'])
    return proc

def lunsh_drone_kit_sitl(location='36.714032117441164,3.181058984340110,0,1'):
    #os.system('xterm -e dronekit-sitl copter --home='+location+" &")
    proc = subprocess.Popen(['python','/home/abroot/Desktop/extractData/utils/lunsher_drone_kit_sitl.py',location])
    return proc


def lunsh_mission(id,pos_end,wrong_way,max_dist):
    connection_string = "udp:127.0.0.1:14552"
    # Connect to the Vehicle
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)

    drone_manager = Drone_manager(vehicle)

    print('Create a new mission (for current location)')
    drone_manager.adds_square_mission(pos_end,50,wrong_way)


    # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
    drone_manager.arm_and_takeoff(10)

    print("Starting mission")
    # Reset mission set to first (0) waypoint
    vehicle.commands.next=0

    # Set mode to AUTO to start mission
    vehicle.mode = VehicleMode("AUTO")


    # Monitor mission. 
    # Demonstrates getting and setting the command number 
    # Uses distance_to_current_waypoint(), a convenience function for finding the 
    # distance to the next waypoint.

    data_trace = []
    distance_waypoint = None
    while True:

        nextwaypoint=vehicle.commands.next
        distance = drone_manager.distance_to_current_waypoint()
        if distance_waypoint is None:
            if distance is not None:
                distance_waypoint = distance

        if distance_waypoint :
            print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_waypoint - drone_manager.distance_to_current_waypoint()))    

        """
        if nextwaypoint==3: #Skip to next waypoint
            print('Skipping to Waypoint 5 when reach waypoint 3')
            vehicle.commands.next = 5
        if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break
        """
        
        instance = [
            vehicle.location.global_relative_frame.alt,
            vehicle.attitude.pitch,
            vehicle.attitude.yaw,
            vehicle.attitude.roll,
            vehicle.location.global_relative_frame.lon,
            vehicle.location.global_relative_frame.lat
        ]
        data_trace.append(instance)
        print(instance)

        time.sleep(1)
        if distance ==  drone_manager.distance_to_current_waypoint():
            break
        
        if distance_waypoint:
            if max_dist <= (distance_waypoint - drone_manager.distance_to_current_waypoint()) :
                break


    #Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()


    #save data in csv file
    with open("legitimate/data_trace_%d.csv" % id,"w") as result:
        writer = csv.writer(result)
        writer.writerow(['position_start_x','position_start_y','position_end_x','position_end_y','type'])
        #writer.writerow([pos_start_x,pos_start_y,pos_end_x,pos_end_y, 1 ])
        writer.writerow([data_trace[0][4],data_trace[0][5],data_trace[-1][4],data_trace[-1][5]])
        line = ['attitude','pitch','yaw','roll','lon','lat']
        writer.writerow(line)

        for line in data_trace:
            writer.writerow(line)




