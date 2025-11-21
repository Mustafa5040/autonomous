from pymavlink import mavutil
from idaMavDefinitions import *
import time
                                                                                                                                                                                                                                               
class idaMavUtil():
    def __init__(self,master):
        self.master = master

    def getCurrentGPSLocation(self):
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT",blocking=True);
        if msg is not None:
            return msg
        return None
    
    def waitHeartbeat(self):
        msg = self.master.recv_match(type="HEARTBEAT",blocking=True)
        if msg is not None:
            return msg
        return None
    def setModeA(self, mode_name):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.set_mode(mode_id)
    def takeOff(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            0  # final altitude
        )
        time.sleep(6)

    def transportThatLoc(self,lat,lot,alt):
        self.master.mav.set_position_target_global_int_send(
            (int(time.time() * 1e3) % 4294967295),
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7),
            int(lot * 1e7),
            alt,
            0,0,0,
            0,0,0,
            0,0
        )
    def stopImmmediately(self):
        self.master.mav.set_position_target_local_ned_send(
            int(time.time() * 1e3) % 4294967295,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,  
            0, 0, 0,            
            0, 0, 0,              
            0, 0, 0,0,0               
)
    
    def setMode(self, mode:MAV_MODE_ENUM ):
        self.master.mav.command_long_send(
            self.master.target_system,
            0,
            176,
            0,
            mavutil.mavlink.MAV_MODE_GUIDED_ARMED,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0,0,0,0,0
        )
        
    def armVehicle(self,isForce=0):
        self.master.mav.command_long_send(
            self.master.target_system,
            0,
            400,
            0,
            1,
            isForce,
            0,0,0,0,0
        )
    def disarmVehicle(self,isForce=0):
        self.master.mav.command_long_send(
            self.master.target_system,
            0,
            400,
            0,
            0,
            isForce,
            0,0,0,0,0
        )
    def pauseMission(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            0,
            193,
            0,
            0,0,0,0,0,0,0
        )
    def continueMission(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            0,
            193,
            0,
            0,0,0,0,0,0,0
    )
    def getMissionWaypoints(self):
        msg = self.master.recv_match(type="MISSION_REQUEST_LIST",blocking=False);
        if msg is not None:
            return msg
        return None
    def clearAllWaypoints(self):
        msg = self.master.recv_match(type="MISSION_CLEAR_ALL",blocking=False)
        if msg is not None:
            return msg
        return None
    def reorderWaypointsCloserToFurther(self):
        curr_wp = self.getMissionWaypoints()
        if curr_wp is None:
            return None
        