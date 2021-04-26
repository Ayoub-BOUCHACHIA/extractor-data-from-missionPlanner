import os,sys



def os_lunsher_drone_kit_sitl():
    location=sys.argv[1]
    os.system('xterm -e dronekit-sitl copter --home='+location+" &") 

os_lunsher_drone_kit_sitl()