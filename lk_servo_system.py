#!/usr/bin/python
########################
## lk_servo_system.py : A library to manage servo systems
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
##
## PRERREQUISITES:
##	python-serial python-lockfile 
##
## REFERENCES.:
##	http://www.pololu.com/docs/0J40
##	http://dmt195.wordpress.com/2009/01/19/python-to-interface-with-the-pololu-8-channel-servo-controller/
##	http://martinsant.net/?page_id=479
## 	http://forum.pololu.com/viewtopic.php?f=16&t=5591
##
## COMMENTS:
##	(*) We define a servo system as a single servo controller with some 
##	servos connected and some pan / tilt station definition.  Such a
##	system is then managed by this library software and completely
##	described in the configuration file.
##	At the moment, the system supports only one (1) servo controller at
##	a time, i.e., a servo system is described as a single servo controller 
##	Of course you can run the same utility in different moments on 
##	different servo controllers by specifying different configuration files
##	in the command line of the invoking program.
##	(*) Angles as always used as integer numbers, because usually servo 
##	motors don't have better resolution and this simplifies the 
##	angle <-> pulse conversion
##	(*) The code is prepared to work with any servo controller, although it
##	only supports Pololu MicroMaestro Servo Controller (implemented in 
##	the lkMM6_ServoController class. You can add code to support your 
##	servo controller by creating a similar class and adding the appropriate
##	instantiation code in lkServoSystem __init__ method.
##	(*) The lkServoSystem class has two variables to allow the user to 
##	control errors:
##		- errCondition : is True when the last method that has been
##		exectued has passed through an error, and False otherwise
##		- lastError : a string describing the error, or "" if no
##		errors happened during the execution of last invoked method
##
## USAGE:
##	To use this code you have to install it following the installation 
##	instructions.
##	After that, fill your configuration file (located by default at 
##	/etc/lk_servo_system.conf). The configuration file 
##	is a JSON file quite self-explained. 
##	Then, include the final path of this library in your PYTHONPATH 
##	environment variable and import it in your code.
##
##	You have to instantiate an object of class lkServoSystem and use
##	its methods to control your servo system. Methods and properties are
##	describe inline in the code.
########################
###
### IMPORTS SECTION
###
import serial
import time
import lockfile
import json
import sys

###
### CLASS DEFINITION SECTION
###

# CLASS: lkServo
#	This class defines all the functional parameters of a servo motor.
class lkServo:
	minAngle=0
	maxAngle=0
	pulseRange=0
	minPulse=0
	maxPulse=0
	centralPulse=0
	weakPulseSpeed=0
	safePulseSpeed=0
	safePulseAcceleration=0

	currentAngle=0
	currentPulse=0
	currentPulseSpeed=0
	currentPulseAcceleration=0
	
	def __init__(	self,
			minAngle, angleRange, centralPulse, pulseHalfRange, 
			weakPulseSpeed, safePulseSpeed , safeAccelerationSpeed
	):
			self.angleRange = angleRange
			self.minAngle = minAngle
			self.centralPulse = centralPulse
			self.pulseRange = 2 * pulseHalfRange
			self.weakPulseSpeed = weakPulseSpeed
			self.safePulseSpeed = safePulseSpeed
			self.safeAccelerationSpeed = safeAccelerationSpeed
			self.maxAngle = self.minAngle + self.angleRange
			self.minPulse = self.centralPulse - pulseHalfRange
			self.maxPulse = self.centralPulse + pulseHalfRange


# CLASS: lkServoController
#	This class defines all the functional parameters of a servo controller,
#	the interface with the device and all the used commands of the servo
#	controller.
class lkMM6_ServoController:
	_serial_ = None
	_serialLock_ = None
	_usbPort_ = None
	
	errCondition = False
	lastError = ""
	numServos = 8

	# __init__ : 		Initilization function
	#	usbPort: the special file name that references the usb-serial 
	#	interface (typically /dev/ttyACM0). This file name must be 
	#	specified in the configuration file.
	def __init__(self, usbPort):
		self._usbPort_ = usbPort
		try:
			self._serialLock_ = lockfile.FileLock(self._usbPort_)
			self._serial_ = serial.Serial(usbPort, timeout=1)
				
		except:
			self.errCondition = True
			self.lastError = "Error initializing serial usb port %s (%s)" % (self._usbPort_ , sys.exc_info()[0],)
			self.lastError += " (Check that the associated .lock file '%s' does not exists)" % (usbPort+".lock")
			return
		
		self.errCondition = False
		self.lastError = ""

	# Close : 		Closing function (closes the serial interface)
	def Close(self):
		try:
			self._serial_.close()
		except:
			self.errCondition = True
			self.lastError = "Error while closing serial connection to micro controller: %s" % sys.exc_info()[0]
			return
		self.errCondition=False
		self.lastError=""

	# GetServoPulse : 	Retrieves from the servo controller the 
	#			current instant value of the pulse being sent 
	#			to a servo.
	#	servo: the servo number. The function does not check if the
	#	number is correct (0-7).
	#	return: the value of the pulse in microseconds (usecs) or -1
	#	if errors.
	def GetServoPulse(self, servo):
		_pulse = -1
		try:
			self._serialLock_.acquire()
		except lockfile.LockTimeout as err:
			self.errCondition = True
			self.lastError = "Timeout while trying to lock servo %d." % servo
			return -1
		else:
			self._serial_.write(chr(0x90) + chr(servo))
			_read_value = self._serial_.read(2)
			_pulse = ord(_read_value[0])+(ord(_read_value[1])<<8)
			_pulse = _pulse / 4.0
			self._serialLock_.release()
			self.errCondition = False
			self.lastError = ""
			return _pulse

	# SetServoPulse : 	Sets the target of the pulse to be sent to a
	#			servo. Although this has an immediate effect,
	#			the pulse value will be reached after a time
	#			that depends on configured servo speed and 
	#			acceleration.
	#	servo: the servo number. The function does not check if the
	#	number is correct (0-7).
	#	pulse: the target pulse in usecs for the servo. The function
	#	does not verify whether the pulse values is withing the valid
	#	range for this servo.
	def setServoPulse (self, servo, pulse):
		_pulse_low = ((4*int(pulse)) & 0x7f)
		_pulse_high = ((4*int(pulse)) >> 7) & 0x7f
		_chan  = servo &0x7f
		_data =  chr(0x84) + chr(_chan) + chr(_pulse_low) + chr(_pulse_high)
		try:
			self._serialLock_.acquire(timeout=5)
		except lockfile.LockTimeout as err:
			self.errCondition = True
			self.lastError = "Timeout while trying to lock servo %d." % servo
			return -1
		else:
			self._serial_.write(_data)
			self._serialLock_.release()	
			self.errCondition = False
			self.lastError = ""
			return 0

	# SetServoPulseSpeed : 	Sets the servo pulse duration variation speed
	#			limit.
	#	servo: the servo number. The function does not check if the
	#	number is correct (0-7).
	#	speed: duration variation speed. Units: 0,25 usecs / 10 ms	
	#	0 = unlimited
	def SetServoPulseSpeed(self, servo, speed):
		_speed_low = speed & 0x7F
		_speed_high = (speed >> 7) & 0x7F
		try:
			self._serialLock_.acquire(timeout=5)
		except lockfile.LockTimeout as err:
			self.errCondition = True
			self.lastError = "Servo {servo_num}: failed to acquire lock on {dev}.".format(servo_num=servo, dev=self._usbPort_)
			return -1
		else:
			self._serial_.write(chr(0x87) + chr(servo) + chr(_speed_low) + chr(_speed_high))
			self._serial_.flush()
			self._serialLock_.release()	
			self.errCondition = False
			self.lastError = ""
			return 0

	# SetServoPulseAcceleration :
	#			Sets the servo pulse duration variation 
	#			acceleration limit
	#	servo: the servo number. The function does not check if the
	#	number is correct (0-7).
	#	acceleration: duration variation acceleration. 
	#	Units: (0,25 usecs / 10 ms) / 80 ms
	#	0 = unlimited
	def SetServoPulseAcceleration(self, servo, acceleration):
		_a_low = acceleration & 0x7F
		_a_high = (acceleration >> 7) & 0x7F
		try:
			self._serialLock_.acquire(timeout=5)
		except lockfile.LockTimeout as err:
			self.errCondition = True
			self.lastError = "Servo {servo_num}: failed to acquiere lock on {dev}.".format(servo_num=servo, dev=self._usbPort_)
			return -1
		else:
			self._serial_.write(chr(0x89) + chr(servo) + chr(_a_low) + chr(_a_high))
			self._serial_.flush()
			self._serialLock_.release()	
			self.errCondition = False
			self.lastError = ""
			return 0

# CLASS: lkPanTiltStation
#	Definition of an abstract entity that comprises two servo motors assembled in a 
#	pan / tilt configuration
class lkPanTiltStation:
	_lastposFileName_ = ""
	_number_ = -1

	panServo = -1
	tiltServo = -1
	panParkAnlge = -1
	tiltParkAnlge = -1
	label = ""
	lastposFile = None
	lastPan = -1
	lastTilt = -1

	errCondition = False
	lastError = ""

	# __init__ : 		Initilization function
	#	panServo : the number of the servo that controls PAN movement
	#	tiltServo : the number of the servo that controls TILT movement
	#	panParkAngle : the angle of the pan servo to be set for the servo
	#	to be parked. This is used when you have a "safe" position for 
	#	your servo
	#	tiltParkAngle : the angle of the tilt servo to be set for the servo
	#	to be parked.
	def __init__(self, number, label, panServo, tiltServo, panParkAngle, tiltParkAngle):
		self._number_ = number
		self.label = label
		self.panServo = panServo
		self.tiltServo = tiltServo
		self.panParkAngle = panParkAngle
		self.tiltParkAngle = tiltParkAngle

		self.lastposFileName = "/var/lib/lk_servo_system/lastpos_station_{:02d}".format(self._number_)
		try:
			self.lastposFile=open(self.lastposFileName, 'r+')
			if self.lastposFile is None:
				self.errCondition = True
				self.lastError = "Unable to open last position file while intializating pan/tilt station %d" % number
		except:
			self.errCondition = True
			self.lastError = "Error while opening file %s (%s)" % (
				self.lastposFileName, sys.exc_info()[0]
				)
			return None
		self.LoadLastpos()

	# Close : 	Termination Function
	#		Closes the lastpos file and exits
	def Close(self):
		self.errCondition = False
		self.lastError = ""
		self.WriteLastpos()
		try:
			self.lastposFile.close()
		except:
			self.errCondition = True
			self.lastError = "Error while closing file %s (%s)" % (
				self.lastposFileName, sys.exc_info()[0]
				)
	
	# LoadLastpos : 	Reads the stored station pan/tilt last position
	#			from file
	def LoadLastpos(self):
		_file = self.lastposFile
		_lastPanAngle=90
                _lastTiltAngle=90
		try:
			_file.seek(0)
			_lastPanAngle=int(_file.readline())
			_lastTiltAngle=int(_file.readline())
			self.errCondition = False
			self.lastError = ""
		except:
			self.errCondition = True
			self.lastError = "Unable to read last position of station %d (%s) from file '%s'" % (self._number_, self.label, self.lastposFileName)
		self.lastPan = _lastPanAngle
		self.lastTilt = _lastTiltAngle


	# SaveLastpos : 	Saves the current pan/tilt values to file
	def WriteLastpos(self):
		_file = self.lastposFile
		try:
			_file.seek(0)
			_file.write("{:03d}\n".format(self.lastPan))
			_file.write("{:03d}\n".format(self.lastTilt))
			_file.flush()
			self.errCondition = False
			self.lastError = ""
		except:
			self.errCondition = True
			self.lastError = "Unable to write last position of station %d (%s)" % (self._number_, self.label)
		

# CLASS: lkServoSystem
#	Definition of the class that compounds a servo system. This class is the interface
#	of the library with external code.
class lkServoSystem:
	_configFile_ = None
	_controllerModel_ = ""
	_SController_ = None
	_servo_ = None
	_PTStation_ = None

	errCondition = False
	lastError = ""
	numPTStations = 0

	# __init__ : 		Initilization function
	#	configfile : The name of the JSON configuration file containing
	#	the servo system complete description.
	def __init__(self, configfile="/etc/lk_servo_system.conf"):
		self.errCondition = False
		self.lastError = ""

		try:
			self._configFile_ = open(configfile, 'r')
		except:
			self.errCondition = True
			self.lastError = "Unable to open " + configfile
			return
		try:
			json_spec = json.load(self._configFile_)
			self._controllerModel_ = json_spec["Model"]
			_tmp = json_spec["UsbPort"]
		except:
			self.errCondition = True
			self.lastError = "Error parsing json file " + configfile
			return

		if self._controllerModel_ == "Micro Maestro 6":
			self._SController_ = lkMM6_ServoController(json_spec["UsbPort"])
		else:
			self.errCondition = True
			self.lastError = "Servo controller model '%s' not supported" % self._controllerModel_
			return
		if self._SController_.errCondition:
			self.errCondition = True
			self.lastError = "Error initializing %s servo controller: (%s)" % (self._controllerModel_, self._SController_.lastError,)
			return

		self._servo_ = [None] * self._SController_.numServos
		try:
			for servo in json_spec["Servo"]:
				_numservo = servo["Number"]
				self._servo_[_numservo] = lkServo(
					minAngle=servo["MinAngle"],
					angleRange=servo["AngleRange"], 
					centralPulse=servo["CentralPulse"],
					pulseHalfRange=servo["PulseHalfRange"],
					weakPulseSpeed=servo["WeakPulseSpeed"],
					safePulseSpeed=servo["SafePulseSpeed"], 
					safeAccelerationSpeed=servo["SafeAccelerationSpeed"]
					) 
		except:
			self.errCondition = True
			self.lastError = "Error parsing servo configuration (%s)." % sys.exc_info()[0]
			return
		
		self._PTStation_ = [None] * (1+max([PT["Number"] for PT in json_spec["PanTiltStation"]]))
		for PTStation in json_spec["PanTiltStation"]:
			try:
				if PTStation["PanServoNumber"] < 0 or PTStation["TiltServoNumber"] < 0 or PTStation["PanServoNumber"] > self._SController_.numServos-1 or PTStation["TiltServoNumber"] > self._SController_.numServos-1:
					self.lastError = 'PanTiltStation "%s" has a servo number out of servo controller range.' % PTStation["Label"]
					return
				_numStation = PTStation["Number"]
				self._PTStation_[_numStation] = lkPanTiltStation(
					number = _numStation ,
					label = PTStation["Label"] , 
					panServo = PTStation["PanServoNumber"] ,
					tiltServo = PTStation["TiltServoNumber"] ,
					panParkAngle = PTStation["PanParkAngle"] ,
					tiltParkAngle = PTStation["TiltParkAngle"] ,
					)
				if self._PTStation_[_numStation].errCondition:
					self.errCondition = True
					self.lastError = "Error creating pan/tilt station %d ( %s ): %s" % (_numStation, self._PTStation_[_numStation].label, self._PTStation_[_numStation].lastError)
			except:
				self.errCondition = True
				self.lastError = "Error parsing Pan/Tilt station configuration (%s)." % sys.exc_info()[0]
				return
		self.numPTStations = len(self._PTStation_)

	# Close: 		Closes the servo controller an 
	#			all the PTStations and exits.
	def Close(self):
		self._SController_.Close()
		_errCondition = self._SController_.errCondition
		_lastError = self._SController_.lastError
		for _station in self._PTStation_:
			if _station is not None:
				_station.Close()
				_errCondition = _errCondition or _station.errCondition
				if _station.errCondition:
					_lastError += " # " + _station.lastError

	# GetStationNumber: 	Gets the number of a pan/tilt station provided
	#			its label. This is provided to allow a user to
	#			reference a station by its label instead of its
	#			number.
	#	label : The label of the pan/tilt station as written in the
	#	configuration file.
	#	returns : The station number corresponding to specified label or -1
	#	if the label is not found
	def GetStationNumber(self, label):
		number=-1
		try:
			number = [i for i,station in enumerate(self._PTStation_) if station.label == label][0]
			self.errCondition = False
			self.lastError = ""
		except:
			self.errCondition = True
			self.lastError = "The station label '%s' is not defined." % label
			number=-1
		return number

	# ServoPulse2Deg : 	Returns the corresponding angle value (deg) of
	#			a pulse duration (usecs) for a particular servo.
	#	servonum : the servo number. The functions does not check whether this
	#	number is correct.
	#	pulsevalue : the pulse duration in usecs.
	#	returns : the corresponding angle in degrees (always integer).
	def ServoPulse2Deg(self, servonum, pulsevalue):
		if pulsevalue == 0:
			return 0
		else:
			return round(self._servo_[servonum].minAngle+((pulsevalue-self._servo_[servonum].minPulse)*self._servo_[servonum].angleRange/self._servo_[servonum].pulseRange))

	# ServoDeg2Pulse : 	Returns the corresponding pulse value (usecs) 
	#			of an angle (deg) for a particular servo.
	#	servonum : the servo number. The functions does not check whether this
	#	number is correct.
	#	anglevalue : the angle in degrees.
	#	returns : the corresponding pulse duration in usecs (always a 
	#	non-negative integer).
	def ServoDeg2Pulse(self, servonum, anglevalue):
		return round(self._servo_[servonum].minPulse+((anglevalue-self._servo_[servonum].minAngle)*self._servo_[servonum].pulseRange/self._servo_[servonum].angleRange))

	# GetStationPanTilt : 	Returns current position (pan, tilt) set in 
	#			the servo controller (pan,tilt) for a station.
	#	stnumber : the station number
	#	returns : (pan, tilt) tuple. (-1, -1) if any error.
	def GetStationPanTilt(self, stnumber):
		panPulse = self._SController_.GetServoPulse(self._PTStation_[stnumber].panServo)
		if self._SController_.errCondition:
			self.errCondition = True
			self.lastError = "Error while retrieving Station %d pan value (%s)" % [stnumber, self._SController_.lastError,]
			return (-1,-1)
		tiltPulse = self._SController_.GetServoPulse(self._PTStation_[stnumber].tiltServo)
		if self._SController_.errCondition:
			self.errCondition = True
			self.lastError = "Error while retrieving Station %d tilt value (%s)" % [stnumber, self._SController_.lastError,]
			return (-1,-1)
		
		self.errCondition = False
		self.lastError = ""
		return (
			self.ServoPulse2Deg(self._PTStation_[stnumber].panServo,panPulse),
			self.ServoPulse2Deg(self._PTStation_[stnumber].tiltServo,tiltPulse)
		)

	# GetStationLastpos : 		Returns the last position of this station
	#	stnumber : the station number
	#	returns : the stored station position in the format:
	#		(pan, tilt)
	def GetStationLastpos(self, stnumber):
		return (self._PTStation_[stnumber].lastPan, self._PTStation_[stnumber].lastTilt)

	# DisableStation : 	Disables a station in the servo controller.
	#	stnumber : the station number. Correctness of station number is not
	#	checked.
	def DisableStation(self, stnumber):
		self.errCondition = False
		self.lastError = ""
		self._SController_.setServoPulse(self._PTStation_[stnumber].panServo,0)
		self.errCondition = self._SController_.errCondition
		self.lastError = self._SController_.lastError
		self._SController_.setServoPulse(self._PTStation_[stnumber].tiltServo,0)
		self.errCondition = self.errCondition or self._SController_.errCondition
		self.lastError = self.lastError + self._SController_.lastError
		
	# SetServoSpeedAccel : 	Sets the speed and acceleration for a servo.
	#	servonum : the servo number. The functions does not check
	#	whether this number is correct.
	#	speed : the new speed to be set on the controller for the 
	#	servo. Units: (0,25 usecs / 10 ms)
	#	acceleration : the new acceleration to be set on the controller
	#	for the servo. If this is not specified, then the functions
	#	chooses the right value attending to the value of speed: if 
	#	speed is lower than a threshold defined by "weakPulseSpeed"
	#	in the configuration file, then acceleration limit is set to 
	#	"unlimited" to avoid the servo to hang; otherwise, a safe 
	#	acceleration is set on the controller.
	#	Units: (0,25 usecs / 10 ms) / 80 ms
	def SetServoSpeedAccel(self, servonum, speed, acceleration=-1):
		_acceleration = 0
		_speed = 0
		if acceleration == - 1:
			if speed <= self._servo_[servonum].weakPulseSpeed:
				_acceleration = 0
			else:
				_acceleration = self._servo_[servonum].safeAccelerationSpeed
		else:
			_acceleration = acceleration
		
		if speed == - 1:
			_speed = self._servo_[servonum].safePulseSpeed
		else:
			_speed = speed

		self._SController_.SetServoPulseSpeed(servonum, _speed)
		if not self._SController_.errCondition:
			self._servo_[servonum].currentPulseSpeed = _speed
		else:
			self.errCondition = self._SController_.errCondition
			self.lastError = self._SController_.lastError

		if not self._SController_.errCondition:
			self._servo_[servonum].currentPulseAcceleration = _acceleration
		else:
			self.errCondition = self.errCondition or self._SController_.errCondition
			self.lastError = self.lastError + self._SController_.lastError
		
	# SetStationSpeedAccel : 	Sets the speed and acceleration limits of 
	#				the pan/tilt station servos
	#	stnumber : the station number. Correctness of station number is not
	#	checked.
	#	speed : the speed limit to be set, in (0,25 usecs / 10 ms).
	#	If this is a number, then it is applied to both pan and tilt,
	#	whereas if it is a list, then the list should contain
	#	(panspeed, tiltspeed).
	#	acceleration : the acceleration limit to be set. See the
	#	explanation of this parameter in SetServoSpeedAccel function.
	#	Units: (0,25 usecs / 10 ms) / 80 ms
	def SetStationSpeedAccel(self, stnumber, speed, acceleration=-1):
		if isinstance(speed, tuple) or isinstance(speed, list):
			_panspeed = speed[0]
			_tiltspeed = speed[1]
		else:
			_panspeed = speed
			_tiltspeed = speed
		if isinstance(acceleration, tuple) or isinstance(acceleration, list):
			_panaccel = acceleration[0]
			_tiltaccel = acceleration[1]
		else:
			_panaccel = acceleration
			_tiltaccel = acceleration
		self.SetServoSpeedAccel(self._PTStation_[stnumber].panServo, _panspeed, _panaccel)
		_errCondition = self.errCondition
		_lastError = self.lastError
		self.SetServoSpeedAccel(self._PTStation_[stnumber].tiltServo, _tiltspeed, _tiltaccel)
		self.errCondition = _errCondition or self.errCondition
		self.lastError = _lastError + self.lastError
		
	# GetStationSpeedAccel : 	Gets the station speed and acceleration limits for
	#				pan and tilt servo.
	#	stnumber : the station number. Correctness of station number is not
	#	checked.
	#	returns: ( (panspeed, panacceleration) , (tiltspeed, tiltacceleration) )
	def GetStationSpeedAccel(self, stnumber):
		return (
				(
				self._servo_[self._PTStation_[stnumber].panServo].currentPulseSpeed ,
				self._servo_[self._PTStation_[stnumber].panServo].currentPulseAcceleration ,
				) ,
				(
				self._servo_[self._PTStation_[stnumber].tiltServo].currentPulseSpeed ,
				self._servo_[self._PTStation_[stnumber].tiltServo].currentPulseAcceleration ,
				) ,
			)

	# SetStationPanTilt : 		Sets the pan and tilt targets for the
	#				servos of a pan/tilt station on 
	#				the controller.
	#				This function also writes position to file
	#	stnumber : the station number. Correctness of station number is not
	#	checked.
	#	pan : the pan value (degrees) to be set.
	#	tilt : the tilt value (degrees) to be set.
	#		NOTE: If the function gets a pan angle out of servo 
	#		operational range, the fucntion will calculate an 
	#		equivalent pan/tilt position whenever possible, and
	#		move the servos accordingly.
	#	wait : if this parameter is True, then the fuction does not
	#	return until the station reaches its target position
	def SetStationPanTilt(self, stnumber, pan, tilt, wait=False):
		_panservo = self._PTStation_[stnumber].panServo
		_tiltservo = self._PTStation_[stnumber].tiltServo

		_panpos = pan if pan >= 0 else (pan + 360 * (int(abs(pan)/360)+1))
		_tiltpos = tilt if tilt >= 0 else (tilt + 360 * (int(abs(tilt)/360)+1))
		_panalt = _panpos
		_tiltalt = _tiltpos

		if _tiltpos < self._servo_[_tiltservo].minAngle or _tiltpos > self._servo_[_tiltservo].maxAngle:
			self.errCondition = True
			self.lastError = "TILT angle (%d) is out of limits [%d..%d]" % (tilt, self._servo_[_tiltservo].minAngle, self._servo_[_tiltservo].maxAngle)
			return

		if _panpos < self._servo_[_panservo].minAngle or _panpos > self._servo_[_panservo].maxAngle:
			_panalt = (_panpos + 180 + 360) % 360
			if _panalt < self._servo_[_panservo].minAngle or _panalt > self._servo_[_panservo].maxAngle:
				self.errCondition = True
				self.lastError = "PAN angle (%d) is out of limits [%d..%d]" % (pan, self._servo_[_panservo].minAngle, self._servo_[_panservo].maxAngle)
				return
			else:
				_tiltalt = self._servo_[_tiltservo].maxAngle - (_tiltpos-self._servo_[_tiltservo].minAngle)


		self._SController_.setServoPulse(_panservo, self.ServoDeg2Pulse(_panservo,_panalt))
		if self._SController_.errCondition:
			self.errCondition = True
			self.lastError = "Failed to set pan angle %d on Station %d (servo %d): %s" % (pan, stnumber, _panservo, self._SController_.lastError)
			return
		self._SController_.setServoPulse(_tiltservo, self.ServoDeg2Pulse(_tiltservo,_tiltalt))
		if self._SController_.errCondition:
			self.errCondition = True
			self.lastError = "Failed to set tilt angle %d on Station %d (servo %d): %s" % (tilt, stnumber, _tiltservo, self._SController_.lastError)
			return

		# Wait for the servo to finish
		if wait:
			_has_arrived = False
			while _has_arrived == False:
				_has_arrived = True
				for (_s,_angle) in ((_panservo,_panalt), (_tiltservo,_tiltalt)):
					_current_pulse = self._SController_.GetServoPulse(_s)
					_target_pulse = self.ServoDeg2Pulse(_s, _angle)
					if _current_pulse == -1:
						return -1
					if _current_pulse == 0:
						_sleep_time=1
						time.sleep(_sleep_time)
						_has_arrived = False
						break
					if _current_pulse != _target_pulse:
						_sleep_time = (4.0*abs(self._SController_.GetServoPulse(_s) - _target_pulse))/(100.0*self._servo_[_s].currentPulseSpeed)
						time.sleep(_sleep_time)
						_has_arrived = False
						break
		self._PTStation_[stnumber].lastPan = pan
		self._PTStation_[stnumber].lastTilt = tilt
		self._PTStation_[stnumber].WriteLastpos()
		
	
	# ParkStation : 	Moves the pan and tilt servos to the parking
	#			position defined in the configuration file.
	#	stnumber : the station number. Correctness of station number is not
	#	checked.
	def ParkStation(self, stnumber):
		self.SetStationPanTilt( stnumber, self._PTStation_[stnumber].panParkAngle,
 			self._PTStation_[stnumber].tiltParkAngle, wait=True)
		
	# GetStationLimits : 	Returns the stations pan/tilt limits
	#	stnumber : the station number. Correctness of station number is not
	#	checked.
	#	returns : the station limits (degrees) in the format:
	#		( (min_pan, max_pan) , (min_tilt, max_tilt) )
	def GetStationLimits(self, stnumber):
		_panServo = self._PTStation_[stnumber].panServo
		_tiltServo = self._PTStation_[stnumber].tiltServo
		return (
			(
				self._servo_[_panServo].minAngle ,
				self._servo_[_panServo].maxAngle ,
			) ,
			(
				self._servo_[_tiltServo].minAngle ,
				self._servo_[_tiltServo].maxAngle ,
			)
		)
