#!/usr/bin/python
########################
## lk_pantilt.py : A library to manage servo systems
##
## Copyright (C) 2015 LAYAKK - www.layakk.com
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Prerrequisites
#	install python-serial
#
# Refs.:
#	http://www.pololu.com/docs/0J40
#	http://dmt195.wordpress.com/2009/01/19/python-to-interface-with-the-pololu-8-channel-servo-controller/
#	http://martinsant.net/?page_id=479
# 	http://forum.pololu.com/viewtopic.php?f=16&t=5591
########################

###
### IMPORTS SECTION
###
import re
import argparse
import lk_servo_system
from lk_screen_log import lkScreenMsg

###
### FUNCTION SECTION
###

# create_argumentparser : 	Creates the argument parser for this program
def create_argumentparser():
	desc =  'lk_pantilt.py (v1.0) - servo system control utility\n'
	desc += 'Copyright (c) 2015 Layakk (www.layakk.com) (@layakk)'

	parser=argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=desc)
	parser.add_argument("-c", "--config_file", help="File containing the configuration of the servo system managed by one microcontroller. If you don't specify this file, the lk_servo_system.py library will choose the one in the default location (/etc/lk_servo_system.conf)", type=str, required=False)
	parser.add_argument("-s", "--station", help='Pan-Tilt Station to be used. This value can be the station label or the station number', type=str, required=True)
	parser.add_argument("-l", "--speed_limit", help='Pulse variation speed limit for servo motors, in 0.25 usec / 10 msec. (0=nolimit, min=1, max=64). If not specified, the system will set the safe value from the config file.', type=int, required=False, default=-1)
	parser_mode_group = parser.add_mutually_exclusive_group(required=True)
	parser_mode_group.add_argument("-p", "--position", help='Pan,Tilt target position (degrees,degrees). Range [0..360] (be aware that the servo library will reject any value out of the servo range of operation).', type=str, required=False)
	parser_mode_group.add_argument("-k", "--park", help='Go to parking position.', required=False, action='store_true')
	parser_mode_group.add_argument("-g", "--get_station_position", help='Get station position from servo controller.', required=False, action='store_true')
	parser_mode_group.add_argument("-d", "--disable_station", help='Disable servos without changing position.', required=False, action='store_true')
	return parser

###
### GLOBAL VARIABLE SECTION
###
SSystem = None
lastTiltAngle = 90
lastPanAngle = 90
lastposfilename = "/var/opt/lk_servo_system/lastpos_"
station = 0
retvalue = 0

###
### M A I N
###

### Argument Parsing
arg_parser = create_argumentparser()
arguments = arg_parser.parse_args()

lkScreenMsg("lk_pantilt.py (v1.0) - servo system control utility")
lkScreenMsg("Copyright (c) 2015 Layakk (www.layakk.com) (@layakk)")
lkScreenMsg()

# Initialize servo configuration
lkScreenMsg("___ INITIALIZATION ___", IndentLevel=0)
lkScreenMsg("Initializing servo system...",IndentLevel=1)
lkScreenMsg("(config file is %s)\t" % arguments.config_file, ToBeContinued=True, IndentLevel=2)
if arguments.config_file:
	SSystem=lk_servo_system.lkServoSystem(arguments.config_file)
else:
	SSystem=lk_servo_system.lkServoSystem()
	
if SSystem.errCondition:
	lkScreenMsg(" FAILED!")
	lkScreenMsg("(%s)" % SSystem.lastError, IndentLevel=2)
	lkScreenMsg("Exiting.", IndentLevel=1)
	exit(1)
else:
	lkScreenMsg(" OK!")	

lkScreenMsg("Verifying command line...", ToBeContinued=True, IndentLevel=1)
# Check correction of antenna number value
_auxmatch = re.match('\d+$',arguments.station)
if _auxmatch:
	station = int(_auxmatch.group())
else:
	station = SSystem.GetStationNumber(arguments.station)

if station == -1:
	lkScreenMsg(" FAILED!")
	lkScreenMsg("(%s)" % SSystem.lastError, IndentLevel=2)
	lkScreenMsg("Exiting.", IndentLevel=1)
	SSystem.Close()
	exit(1)

if arguments.position:
	position = arguments.position.split(",")
	if len(position) != 2:
		lkScreenMsg(" FAILED!")
		lkScreenMsg("'position' must be a list of exactly 2 integer values, separated by comma.", IndentLevel=2)
		lkScreenMessage("Exiting.", IndentLevel=1)
		SSystem.Close()
		exit(1)
	try:
		newpan = int(position[0])
	except ValueError:
		lkScreenMsg(" FAILED!")
		lkScreenMsg("'pan value' must be an integer.", IndentLevel=2)
		lkScreenMessage("Exiting.", IndentLevel=1)
		SSystem.Close()
		exit(1)
	try:
		newtilt = int(position[1])
	except ValueError:
		lkScreenMsg(" FAILED!")
		lkScreenMsg("'tilt value' must be an integer.", IndentLevel=2)
		lkScreenMessage("Exiting.", IndentLevel=1)
		SSystem.Close()
		exit(1)
lkScreenMsg(" OK!")

lkScreenMsg(IndentLevel=0)
lkScreenMsg("___ EXECUTION ___")
if arguments.get_station_position:
	lkScreenMsg("Getting station position from servo controller...", IndentLevel=1)
	(pan, tilt) = SSystem.GetStationPanTilt(station)
	if SSystem.errCondition:
		lkScreenMsg(" FAILED!")
		lkScreenMsg("(%s)"%SSystem.lastError, IndentLevel=2)
		retvalue = 1
	else:
		lkScreenMsg(" OK!")
		
	lkScreenMsg("[ pan , tilt ] = [ %s, %s ] " % 
		(["disabled",str(int(pan))][int(pan)!=0], 
		["disabled",str(int(tilt))][int(tilt)!=0],),
		IndentLevel=2
	)
elif arguments.disable_station:
	lkScreenMsg("Disabling servos...", ToBeContinued=True, IndentLevel=1)
	SSystem.DisableStation(station)
	if SSystem.errCondition:
		lkScreenMsg(" FAILED!")
		lkScreenMsg("(%s)"%SSystem.lastError, IndentLevel=2)
		retvalue = 1
	else:
		lkScreenMsg(" OK!")
else:
	# Set station speed and acceleration
	lkScreenMsg("Setting speed and acceleration...", ToBeContinued=True, IndentLevel=1)
	SSystem.SetStationSpeedAccel(station, arguments.speed_limit)
	if SSystem.errCondition:
		lkScreenMsg(" FAILED!")
		lkScreenMsg(SSystem.lastError, IndentLevel=2)
		retvalue=1
	else:
		lkScreenMsg(" OK!")
		( (_panspeed,_panaccel,) , (_tiltspeed, _tiltaccel,), ) = SSystem.GetStationSpeedAccel(station)
		lkScreenMsg("Pan Servo  --> \tSpeed: %s\t Acceleration: %s" % (
			[str(_panspeed),"unlimited"][_panspeed==0],
			[str(_panaccel),"unlimited"][_panaccel==0],
			),
			IndentLevel=2
		)
		lkScreenMsg("Tilt Servo --> \tSpeed: %s\t Acceleration: %s" % (
			[str(_tiltspeed),"unlimited"][_tiltspeed==0],
			[str(_tiltaccel),"unlimited"][_tiltaccel==0],
			)
		)

	# Put antena in initial position
	if retvalue == 0:
		(lastPanAngle,  lastTiltAngle) = SSystem.GetStationLastpos(station)
		lkScreenMsg("Setting station target to its previous position ({pan},{tilt})... ".format(pan=lastPanAngle,tilt=lastTiltAngle), IndentLevel=1, ToBeContinued=True)
		SSystem.SetStationPanTilt(station, lastPanAngle, lastTiltAngle, wait=True)
		if SSystem.errCondition:
			lkScreenMsg(" FAILED!")
			retvalue=1
		else:
			lkScreenMsg(" OK!")
	
	# Continue with movement
	if retvalue == 0:
	
		if arguments.park:
			lkScreenMsg("Parking Station %d..." % station, ToBeContinued=True, IndentLevel=1)
			SSystem.ParkStation(station)
			if SSystem.errCondition:
				lkScreenMsg(" FAILED!")
				lkScreenMsg(SSystem.lastError, IndentLevel=2)
				retvalue=1
			else:
				lkScreenMsg(" OK!")
				(pan, tilt) = SSystem.GetStationPanTilt(station)
				lkScreenMsg("[ pan , tilt ] = [ %s, %s ] " % 
					(["disabled",str(int(pan))][int(pan)!=0], 
					["disabled",str(int(tilt))][int(tilt)!=0],),
					IndentLevel=2
				)
		elif arguments.position:
			pan=int(position[0])
			tilt=int(position[1])
			lkScreenMsg("Moving station to (%d,%d)..." % (pan,tilt), IndentLevel=1, ToBeContinued=True)
			SSystem.SetStationPanTilt(station, pan, tilt, wait=True)
			if SSystem.errCondition:
				lkScreenMsg(" FAILED!")
				lkScreenMsg(SSystem.lastError, IndentLevel=2)
				retvalue=1
			else:
				lkScreenMsg(" OK!")
				

lkScreenMsg(IndentLevel=0)
lkScreenMsg("___ TERMINATION ___")
		
	
lkScreenMsg("Closing connection to servo system... ", ToBeContinued=True, IndentLevel=1)
SSystem.Close()
if SSystem.errCondition:
	lkScreenMsg(" FAILED!")
	lkScreenMsg(SSystem.lastError, IndentLevel=2)
	retvalue = 1
else:
	lkScreenMsg(" OK!")

lkScreenMsg("Exiting program.")
exit(retvalue)
