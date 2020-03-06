import serial
import RPi.GPIO as GPIO
import os, time

GPIO.setmode(GPIO.BOARD)
port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=2.5)

while True:
    rcv = port.read(20)
    print(rcv)
