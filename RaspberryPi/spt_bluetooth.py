#------------------------------------------------------------------------------
# Filename:     spt_bluetooth.py
# Author(s):    David Taylor, Dhruv Singhal
# ShortDesc:    Enables bluetooth for the SPT robot (for EECS 159)
#
# Usage:
# - Manual control is accomplished
#  - Auto: CV run on the NVIDIA Jetson Nano will steer the car
#  - Manual: DualShock4 controller to steer the car
#
# Changelog
# Version   Date        Delta
# v1.0      2020.03.05  Ported from "team_herbie" repo, cleaned up some
#
# Resources:
# - resource    |   url
#
# Feature Requests: (based on priority)
# - Add a startup verification system (probably return a confirmation after all of the object constructors are run?) that gives LED feedback (progress bar for boot?).
# - Run on boot on Raspberry Pi
# - Wrap everything in classes
#  - Add controller pairing failsafe
# - Add different controlling modes (rn we only have "intuitive" mode)
#------------------------------------------------------------------------------

#-----<LIBRARIES>-----
import RPi.GPIO as GPIO

from time import sleep
import smbus
import struct

# Used for game controller interfacing
import pygame

# LED control
#import board    # Adafruit library per https://circuitpython.readthedocs.io/projects/neopixel/en/latest/
#import neopixel # library for controlling the LED ring

#-----<GLOBAL VARIABLES>-----

# GPIO PIN MAPPING
LED_PIN = 18

MOTOR_R = 32   # Used for the right side of the vehicle
MOTOR_L = 33   # Used for the left side of the vehicle

# CONSTANTS
DRIVE_MODES = 2

#-----<CLASSES>-----

class DC_Motor_Controller:
    """Object for controlling the DC motors with software PWM, utilizing the RPi.GPIO library"""

    # Default data members
    bus = smbus.SMBus(1)
    address = 0x04


    idleSpeed = 0       # Center speed at zero
    speedScaler = 1    # Max speed is 100, min is -100.

    maxDeltaY = 2.0     # Greatest change between cycles for forward/reverse axis in percent
    maxDeltaX = 1.0     # Greatest change between cycles for left/right axis in percent

    rSpeed = 0          # State variable that will be adjusted towards setpoint defined by user input
    lSpeed = 0          # "



    # Pass the GPIO numbers for motor connections A and B
    def __init__(self, pinR, pinL):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)    # See RPi.GPIO docs for what this is

        GPIO.setup(int(pinR), GPIO.OUT)    # Change output mode of pinR
        GPIO.setup(int(pinL), GPIO.OUT)    # Change output mode of PinL

        self.R_PWM = GPIO.PWM(pinR, 210)    # Original setpoint was 200Hz: dhruv's gut told us we needed 192Hz i guess
        self.L_PWM = GPIO.PWM(pinL, 210)    # "

        self.R_PWM.start(self.idleSpeed)    # Activate PWM for pin R
        self.L_PWM.start(self.idleSpeed)    # Activate PWM for pin L

    # Output of changeSpeed depends on the current drive mode.
    def changeSpeed(self, rightStick, leftStick):
        """Input values of rightStick and leftStick between -100 and 100"""
        # Check to see if values have changed, abort if not
        if (rightStick == self.rSpeed) and (leftStick == self.lSpeed):
            return False

        # Implement basic mixing algorithm
        rTemp = leftStick + (self.maxDeltaX * (rightStick))
        lTemp = leftStick - (self.maxDeltaX * (rightStick))

        # Limit rate of change for forward/backward motion
        if ((lTemp - self.lSpeed) > self.maxDeltaY):
            self.lSpeed += self.maxDeltaY
        elif ((lTemp - self.lSpeed) < - self.maxDeltaY):
            self.lSpeed -= self.maxDeltaY
        else:
            self.lSpeed = lTemp

        if ((rTemp - self.rSpeed) > self.maxDeltaY):
            self.rSpeed += self.maxDeltaY
        elif ((rTemp - self.rSpeed) < - self.maxDeltaY):
            self.rSpeed -= self.maxDeltaY
        else:
            self.rSpeed = rTemp

        if self.rSpeed > 100:
            self.rSpeed = 100
        if self.rSpeed < -100:
            self.rSpeed = -100

        if self.lSpeed > 100:
            self.lSpeed = 100
        if self.lSpeed < -100:
            self.lSpeed = -100

        #send to arduino via i2c

        rMotorValue = self.idleSpeed + (self.rSpeed/self.speedScaler)
        lMotorValue = self.lSpeed + (self.lSpeed/self.speedScaler)
        package = struct.pack('ff', rMotorValue, lMotorValue)
        try: self.bus.write_block_data(self.address, 1, list(package))
        except OSError as err:
            print("OSError detected")

class Remote_Control:
    """Use a DualShock4 controller to manually control the operation of the robot"""

    # Controller Button States
    Ex = False          # PyGame Button 0
    Circle = False      # PyGame Button 1
    Triangle = False    # PyGame Button 2
    Square = False      # PyGame Button 3
    L_Bumper = False    # PyGame Button 4
    R_Bumper = False    # PyGame Button 5
    L_Trigger = False   # PyGame Button 6
    R_Trigger = False   # PyGame Button 7
    Share = False       # PyGame Button 8
    Options = False     # PyGame Button 9
    PS = False          # PyGame Button 10
    L_Stick = False     # PyGame Button 11
    R_Stick = False     # PyGame Button 12

    # Controller Axis States
    L_X_Axis = 0        # PyGame Axis 0: [0, 1.0]
    R_X_Axis = 0        # PyGame Axis 3: [0, 1.0]
    L_Y_Axis = 0        # PyGame Axis 1: [0, 1.0]
    R_Y_Axis = 0        # PyGame Axis 4: [0, 1.0]
    L_Trigger = -1.0    # PyGame Axis 2: [-1.0, 1.0]
    R_Trigger = -1.0    # PyGame Axis 5: [-1.0, 1.0]

    def __init__(self):
        pygame.init()    # Initialize pygame library
        self.controller = pygame.joystick.Joystick(0) # Connect to the controller (and hope that it is paired with the Pi)
        self.controller.init()   # Prepare to read data from controller

    def update(self):   # Only update the relavent buttons/axies
        pygame.event.get()
        self.L_Y_Axis = self.controller.get_axis(1)
        self.R_X_Axis = self.controller.get_axis(3)
        self.R_Y_Axis = self.controller.get_axis(4)
        self.Ex = self.controller.get_button(0)
        self.PS = self.controller.get_button(10)

def __main__():

    GPIO.cleanup()  # Clear any previously used GPIO modes

    # Initialize motor controller objects
    motors = DC_Motor_Controller(MOTOR_R, MOTOR_L)

    # Initialize DualShock4 Controller Connection
    DS4 = Remote_Control()
    print("Remote Control initiated\n")
    R_X_AXIS_SCALE_VAL = 100    # Scale right stick X-axis by 100 to match the changeSpeed method input range
    L_X_AXIS_SCALE_VAL = 100    # Scale left stick X-axis by 100 to match the changeSpeed method input range
    R_Y_AXIS_SCALE_VAL = 100    # Scale right stick Y-axis by 100 to match the changeSpeed method input range
    L_Y_AXIS_SCALE_VAL = 100    # Scale left stick Y-axis by 100 to match the changeSpeed method input range

    try:
        while True:
            DS4.update()
            motors.changeSpeed((DS4.R_X_Axis * R_X_AXIS_SCALE_VAL), (DS4.L_Y_Axis * L_Y_AXIS_SCALE_VAL))

    except KeyboardInterrupt:
        print("\nEXITING NOW\n")
        DS4.controller.quit()
        GPIO.cleanup()  # to be used when GPIO is active

#-----</FUNCTIONS>-----

# Code Execution
__main__()
