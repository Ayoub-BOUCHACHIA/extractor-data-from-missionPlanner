from dronekit import VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

class Drone_manager:

    def __init__(self,vehicle):
        self.vehicle = vehicle

    def get_location_metres(self,original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned Location has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        return LocationGlobal(newlat, newlon,original_location.alt)


    def get_distance_metres(self,aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        print('lat1 : ',aLocation1.lat,'lon1 : ',aLocation1.lon,'alt1 : ',aLocation1.alt)
        print('lat2 : ',aLocation2.lat,'lon2 : ',aLocation2.lon,'alt2 : ',aLocation2.alt)
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



    def distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint. 
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint = self.vehicle.commands.next
        if nextwaypoint==0:
            return None
        missionitem = self.vehicle.commands[nextwaypoint-1] #commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)

        distancetopoint = self.get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint


    def download_mission(self):
        """
        Download the current mission from the vehicle.
        """
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready() # wait until download is complete.



    def adds_square_mission(self,pos_list, aSize):
        """
        Adds a takeoff command and four waypoint commands to the current mission. 
        The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

        The function assumes vehicle.commands matches the vehicle mission state 
        (you must have called download at least once in the session and after clearing the mission)
        """	

        cmds = self.vehicle.commands

        print(" Clear any existing commands")
        cmds.clear() 
        
        print(" Define/add new commands.")
        # Add new commands. The meaning/order of the parameters is documented in the Command class. 
        
        #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
        
        #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
        
        for i in range(1,len(pos_list)):
            [newlat,newlon,_,alt] = pos_list[i].split(",")
            point = LocationGlobal(float(newlat), float(newlon),int(alt))
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point.lat, point.lon, int(alt)))
            
        [newlat,newlon,_,alt] = pos_list[0].split(",")
        point = LocationGlobal(float(newlat), float(newlon),int(alt))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point.lat, point.lon, int(alt)))

        [newlat,newlon,_,alt] = pos_list[0].split(",")
        point = LocationGlobal(float(newlat), float(newlon),int(alt))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point.lat, point.lon, int(alt)))
        
        print(" Upload new commands to vehicle")
        cmds.upload()


    def arm_and_takeoff(self,aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

            
        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:      
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:

            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)      
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)