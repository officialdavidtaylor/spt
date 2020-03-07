from serial import Serial
import RPi.GPIO as GPIO
import os, time

GPIO.setmode(GPIO.BOARD)
port = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=0.5)

while True:
    rcv = port.read(100)
    print(rcv)
