import serial
import RPi.GPIO as GPIO
import os, time

GPIO.setmode(GPIO.BOARD)
port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.5)

while True:
    port.write('A')
    rcv = port.read(10)
    print rcv
