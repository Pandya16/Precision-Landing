# Script to check whether connection with the pixhawk has been established or not

from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
import time

master = mavutil.mavlink_connection(
    device = '/dev/serial0',
    baud = 921600,
    source_system = 1,
    source_component = 191
)

print("Waiting for heartbeat")
master.wait_heartbeat()

print("Heartbeat recieved")