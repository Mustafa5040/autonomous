from pymavlink import mavutil
import math, numpy,cv2
from idaMavUtil import idaMavUtil
from idaMavDefinitions import *

master = mavutil.mavlink_connection("tcp:localhost:5762")
mu = idaMavUtil(master)

print(mu.waitHeartbeat())
currLoc = mu.getCurrentGPSLocation()
print(currLoc)
_lat = currLoc.lat / math.pow(10,7)
_long = currLoc.lon / math.pow(10,7)
_alt = currLoc.alt / math.pow(10,3)
print("LATITUDE: ",_lat)
print("LONGITUDE: ",_long)
print("ALTITUDE: ",_alt)

mu.setModeA("GUIDED")
mu.setMode(MAV_MODE_ENUM.MAV_MODE_AUTO_ARMED)
mu.armVehicle()
mu.takeOff()
mu.transportThatLoc(36.5, 34.0, 10)