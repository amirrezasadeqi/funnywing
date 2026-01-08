#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time, math, argparse
from pymavlink import mavutil
from dronekit import Vehicle, Command, connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command

# Using this function for just Determining of the Lat/Lon and for Alt we would use the Relative Alt as you will see in the following
def get_location_meteres(original_location, dNorth, dEast):
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

def demo_launch_mission(aLocation, aSize = 500, land_incline = 0.06):
  """
  Creates a simple mission for demo purpose!
  """
  cmds = vehicle.commands

  print("Clearing Any Existing Commands")
  cmds.clear()

  print("Adding New Commands for Our Demo Mission!")

  # the first point (home point) would define in Absolute Global Frame! The rest will be global with relative altitude!
  cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1, 0, 0, 0, 0, 0, aLocation.lat, aLocation.lon, aLocation.alt))

  # Add take off command. Actually as I checked the lat and lon of this command has no effect in the result at least for the
  # Fix-Wings. In another words I set them to zero, values based on heading and first waypoint location. In all cases it just
  # follow the altitude and nothing more. But Any way, here I would use a waypoint in the way of the first real waypoint.
  wp_tkoff = get_location_meteres(aLocation, 5, -5)
  cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, wp_tkoff.lat, wp_tkoff.lon, 10))
 
  # Adding other waypoints
  wp1 = get_location_meteres(aLocation, aSize, -aSize)
  wp2 = get_location_meteres(aLocation, aSize, aSize)
  wp3 = get_location_meteres(aLocation, -aSize, aSize)
  wp4 = get_location_meteres(aLocation, -aSize, -aSize)
  # Determine Operating altitude based on the landing path lenght:
  land_alt = 2 * math.sqrt(2) * aSize * land_incline
  # take care about the altitude of the landing starting point. I think you should determine it based on the proper inclination of the landing rootself.
  # as I tested in SITL too steep landings after a while cause the plane go over the landing point and the landing would take place with a small pitch angle which takes too mauch time.
  # on the other hand as I tested for 0.18 inclination the plane tries to land exactly on the landing point and there were some oscillations in pitch angle. Also test for 6% inclination
  # and it has accurate and oscillations free land maneuver. I don't know exactly that these oscillations are for bad tune or something else but in future ... .
  cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp1.lat, wp1.lon, land_alt))
  cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp2.lat, wp2.lon, land_alt))
  cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp3.lat, wp3.lon, land_alt))
  cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp4.lat, wp4.lon, land_alt))

  # Adding Landing waypoint at home location
  # cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, aLocation.lat, aLocation.lon, 0))
  cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, wp2.lat, wp2.lon, 0))
  
  print("Upload New Commands to Vehicle!")
  cmds.upload()


def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist

def upload_mission(aFileName):
  """
  Upload a mission from a file.
  """
  #Read mission from file
  missionlist = readmission(aFileName)

  #Clear existing mission from vehicle
  print(' Clear Already Uploaded Mission')
  cmds = vehicle.commands
  cmds.clear()
 
  print("\nUpload mission from a file: %s" % aFileName)
  #Add new mission to vehicle
  for command in missionlist:
    cmds.add(command)
  print(' Upload mission')
  vehicle.commands.upload()


def vehicle_arm():
  """
    Safely Arm the vehicle!
  """

  print("Change Mode to AUTO")
  vehicle.mode = VehicleMode("AUTO")

  print("Wait for Pre-flight Checks!")
  while not vehicle.is_armable:
    print("Wait for Consistent Sensor Datas and Pre-flight checks!")
    time.sleep(1)

  print("Arming Motors!")
  vehicle.arm()


if __name__ == "__main__":

  parser = argparse.ArgumentParser(description='this is a test program')
  parser.add_argument('--connect', help='Address of vehicle connection.')
  parser.add_argument('--mission_file', help="File input for importing missions.")
  parser.add_argument('--land_incline', help="Inclination of the Line of Landing. Defualt is 0.06.")
  parser.add_argument('--squaure_side_len', help="Length of the square side which we use to define the mission around the vehicle. Default is 500.")
  args = parser.parse_args()

  connection_string = args.connect
  mission_file_address = args.mission_file
  land_incline = args.land_incline
  aSize = args.squaure_side_len


  print("Connecting to vehicle on: %s" % connection_string)
  vehicle = connect(connection_string, wait_ready=True)

  if not mission_file_address:
    print('Create a new mission (for current location)')
    home_loc = vehicle.location.global_frame
    if (not land_incline) and (not aSize):
      demo_launch_mission(home_loc)
    elif (not aSize):
      demo_launch_mission(home_loc, 500, land_incline)
    elif (not land_incline):
      demo_launch_mission(home_loc, aSize)
    else:
      demo_launch_mission(home_loc, aSize, land_incline)

  else:
    upload_mission(mission_file_address)


  # Arming Automatically cause the Plane to take off!
  vehicle_arm()

  print("Close Vehicle connection object!")
  vehicle.close()


# Remember bellow Commands. These will be used in the goto_location Scenarios!
# vehicle.mode = VehicleMode("GUIDED")
# gotolocation = LocationGlobalRelative(  -35.36019042, 149.15741926, 600)
# vehicle.simple_goto(gotolocation)


# Since there is not AUTO Landing Mode Like TAKEOFF mode (in which we can take off without a Mission file.) So in future I think we should create a Service
# For Adding this feature with trick like creating a temporary Mission. Also I think we will need another service for go to location feature(may be asynchronous
# Service. In future ...)

