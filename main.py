import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import serial
import time
import argparse
parser = argparse.ArgumentParser()
baud_rate=57600
#parser.add_argument('--connect', default='127.0.0.1:14550')
parser.add_argument('--connect', default='/dev/ttyUSB0')
args = parser.parse_args()
# Connect to the Vehicle
if ':' in args.connect:
  baud_rate=921600
print("Connecting with baud rate ",baud_rate)
print('Connecting to vehicle on: %s' % args.connect)
#vehicle = connect(args.connect, baud=921600, wait_ready=False)
#921600 is the baudrate that you have set in the mission plannar or qgc
#use baudrate 57600 for sik radio, with wait_ready = True
vehicle = connect(args.connect, baud=57600)

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  #while not vehicle.is_armable:
  #  print(" Waiting for vehicle to pass takeoff prearm checks")
  #  time.sleep(1)
        
  print("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

  print("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt )
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print("Reached target altitude")
      break
    time.sleep(1)




#def goto(LocationGlobalRelative)
def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified location.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle
    vehicle.send_mavlink(msg)


def goto(targetpos):
  arm_and_takeoff(targetpos.alt)
  print("Take off complete")
  vehicle.simple_goto(targetpos)
  print("flying to ",targetpos.lat,",",targetpos.lon)
  currentpos=vehicle.location.global_relative_frame
  while(not (math.isclose(currentpos.lat,targetpos.lat,rel_tol=1e-07) and math.isclose(currentpos.lon,targetpos.lon,rel_tol=1e-07))):
    currentpos=vehicle.location.global_relative_frame
    time.sleep(1)
    print("Current position=",currentpos.lat,currentpos.lon)
    print("Target position= ",targetpos.lat,targetpos.lon)
  print("Target reached")


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def fly_square():
  send_ned_velocity(1,0,0,5)
  send_ned_velocity(0,1,0,5)
  send_ned_velocity(-1,0,0,5)
  send_ned_velocity(0,-1,0,5)

'''
-------Execution-------
 Breach signal is detected by the arduino connected to the computer. 
 In case of breach, arduino sends either 1 or 0 indicating one of two possible breach locations.
 Messages are recieved as bytes, and decoded to string. By default, arduino sends null charecters until there is a breach.

'''
'''
while not vehicle.is_armable:
    print(" Waiting for vehicle to pass intial Pre Arm checks")
    time.sleep(1)
'''
testloc1=LocationGlobalRelative(-35.36330701,149.16318586,15)                   #test coordinate for sim
testloc2=LocationGlobalRelative(-35.36148798,149.16507219,15)                   #test coordinate for sim
#loc1=LocationGlobalRelative(13.38205790,74.73544306, 15)
#loc2=LocationGlobalRelative(13.37967732,74.73636646, 15)
loc1=LocationGlobalRelative(13.17824206, 74.93502419, 7)                       #coordinate for left side of BC alva
loc2=LocationGlobalRelative(13.17855651, 74.93568980,7 )
print("Vehicle location:")
print(vehicle.location.global_frame)
vehicle.home_location=vehicle.location.global_frame
serial_flag=True                                                            #flag indicating if breach input is taken from serial or not
try:                                                                        #try block to check if arduino is connected
  ser = serial.Serial('/dev/ttyACM0', 9800, timeout=1)                      #Set up arduiono's serial port
except serial.serialutil.SerialException:                                   #if arduino is not connected take user input
  print("Arduino not connected. Expecting user input for breach location")
  serial_flag=False

while 1:                                                                    #loop to repeatedly check for breaches, so script can be running all the time
  if serial_flag:                                                       
    sig=ser.read()                                                          #Reads serial messages and stores it into a variable in bytes
    sig=sig.decode('utf-8')                                                 #datatype 'bytes' is converted to string

  else:   
    sig=input("Enter breach loc 1/0:")                                      #takes user input for breach signal as string 1 or 0                                
  if(sig=='1' or sig=='0'):
    print(sig)
    if sig=="1":
      print("Breach at location 1. Arming and taking off to",loc1.alt,"m")
      goto(loc1)
      print("Location reached, flying in square")
      fly_square()

    elif sig=="0":
      print("Breach at location 2. Arming and taking off to",loc2.alt,"m")
      goto(loc2)
      print("Location reached, flying in square")
      fly_square()
#vehicle.close()
