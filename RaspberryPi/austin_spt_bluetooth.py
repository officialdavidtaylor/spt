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
# v0.1      2019-12-01  List of libraries
# v1.0      2019-12-05  Funtional driving code
# v2.0      2020-02-15  Updated for VictorSPX speed controllers
# v2.1      2020-02-26  Added basic control system
#
# Resources:
# - resource    |   url
#
# Feature Requests: (based on priority)
# - Run on boot on Raspberry Pi 4B
# - Fix inability to re-change modes
# - Optomize changeSpeed method
# - Integrate NVIDIA Jetson Nano (with either ML or OpenCV image processing)
# - Add a startup verification system (probably return a confirmation after all of the object constructors are run?) that gives LED feedback (progress bar for boot?).
#  - Add controller pairing failsafe
# - Add different controlling modes (rn we only have tank mode)
# - Replace pygame with homebrewed device file interpretation
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


    idleSpeed = 30.0    # Function of the speed controllers - PWM neutral has period of 1.5ms
    speedScaler = 10    # Max speed is 40%, min speed is 20% due to PWM config

    maxDeltaY = .50     # Greatest change between cycles for forward/reverse axis in percent
    maxDeltaX = 0.2    # Greatest change between cycles for left/right axis in percent

    rSpeed = 0          # State variable that will be adjusted towards setpoint defined by user input
    lSpeed = 0          # "



    # Pass the GPIO numbers for motor connections A and B
    def __init__(self, pinR, pinL, mode):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)    # See RPi.GPIO docs for what this is

        GPIO.setup(int(pinR), GPIO.OUT)    # Change output mode of pinR
        GPIO.setup(int(pinL), GPIO.OUT)    # Change output mode of PinL

        self.R_PWM = GPIO.PWM(pinR, 210)    # Original setpoint was 200Hz: dhruv's gut told us we needed 192Hz i guess
        self.L_PWM = GPIO.PWM(pinL, 210)    # "

        self.R_PWM.start(self.idleSpeed)    # Activate PWM for pin R
        self.L_PWM.start(self.idleSpeed)    # Activate PWM for pin L

        self.driveMode = mode

    def cycleMode(self):
        if self.driveMode == (DRIVE_MODES - 1):
            self.driveMode = 0
        else:
            self.driveMode += 1

    # Output of changeSpeed depends on the current drive mode.
    def changeSpeed(self, rightStick, leftStick):
        """Input values of rightStick and leftStick between -100 and 100"""
        # Check to see if values have changed, abort if not
        if (rightStick == self.rSpeed) and (leftStick == self.lSpeed):
            return False

        # <--Experimental mixing algorithm-->

        # Implement basic mixing algorithm
        rTemp = leftStick + (self.maxDeltaX * (rightStick)) # be careful not to divide by zero
        lTemp = leftStick - (self.maxDeltaX * (rightStick)) # "

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
        #print("rightMotorValue = " + str(rMotorValue))
        lMotorValue = self.lSpeed + (self.lSpeed/self.speedScaler)

        package = struct.pack('ff', rMotorValue, lMotorValue)

        try: self.bus.write_block_data(self.address, 1, list(package))
        except OSError as err:
            print("not today satan (OSError)")



class Axis:
    def __init__(self, axis_id, pygame_controller, position=0):
        # Axis id should be obtained from pygame
        self.axis_id = axis_id
        self.axis_position = axis_position
        self.pygame_controller = pygame_controller
    def updateState(self):
        self.axis_position = self.pygame_controller.update(self.axis_id)

class Button:
    def __init__(self, button_id, pygame_controller, button_state=False):
        # Button id should be obtained from pygame
        self.button_id = button_id
        self.button_state = button_state
    def updateState(self):
        self.button_state = self.pygame_controller.get_button(self.button_id)

class Buttons:
    self.buttons = {
        X: None,
        Circle: None,
        Triangle: None,
        Square: None,
        L_Bumper: None,
        R_Bumper: None,
        L_Trigger: None,
        R_Trigger: None,
        Share: None,
        Options: None,
        PS: None,
        L_Stick: None,
        R_Stick: None,
    }
    def __init__(self, pygame_controller):
        controller_id_incrementor = 0
        for(key in self.buttons.keys()):
            self.buttons[key] = Button(controller_id_incrementor, pygame_controller)
    def updateAllButtonsState(self):
        for(key in self.buttons.key()):
            self.buttons[key].updateState()




class Remote_Control:
    """Use a DualShock4 controller to manually control the operation of the robot"""
    def __init__(self):
        pygame.init()    # Initialize pygame library
        self.pygame_controller = pygame.joystick.Joystick(0) # Connect to the controller (and hope that it is paired with the Pi)
        self.pygame_controller.init()   # Prepare to read data from controller

        # Controller Buttons
        # TODO: this may break not sure if keys need to be strings in python
        self.BUTTONS = BUTTONS(self.pygame_controller)

        # Controller Axis'
        self.AXISES = {
            left_x_axis: Axis(0, self.pygame_controller),
            left_y_axis: Axis(1, self.pygame_controller),
            right_x_axis: Axis(2, self.pygame_controller),
            right_y_axis: Axis(3, self.pygame_controller),
        }


    def update(self):
        # Only update the relavent buttons/axies
        # TODO: why? ^
        pygame.event.get()
        self.BUTTONS.updateAllButtonsState()
        for(key in self.AXISES.keys()): self.BUTTONS[key].updateState()

def __main__():

    GPIO.cleanup()  # Clear any previously used GPIO modes

    driveMode = 0   # Start in Intuitive Mode (mode 0)

    # Initialize Neopixel ring
    #

    # Initialize motor controller objects
    motors = DC_Motor_Controller(MOTOR_R, MOTOR_L, driveMode)

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