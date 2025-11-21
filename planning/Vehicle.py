from math import pow
from pathPlanning import WGS84GPS
class Vehicle:
    def __init__(self,mav_con,mavutil,width,height):
        self.mav_con = mav_con
        self.mavHandler = mavutil
        self.w = 1.2
        self.h = 0.7
    def getCenterGPS(self,unit='gps'):
        currLoc = self.mav_con.getCurrentGPSLocation()
        _lat = currLoc.lat / pow(10,7)
        _lon = currLoc.lon / pow(10,7)
        _alt = currLoc.alt / pow(10,3)
        _yaw = currLoc.hdg
        return WGS84GPS(_lon,_lat,_alt,_yaw)