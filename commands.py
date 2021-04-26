#!/usr/bin/cv RPi.GPIO
#import cv2
import numpy as np
from time import sleep
from signal import pause
import serial
import os
import math
import time


#This line solves the issue of being unable to run the code in ssh given the
#error "Cannot connect to X server"
#os.environ['DISPLAY'] = ':0'

# Frame size in pixels
frameWidth = 800
frameHeight = 600
ARDUINO = True
VS = "red"

#Establishes serial connection with the Arduino before the code can be run
if ARDUINO:
    if __name__ == '__main__':
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        ser.flush()

# This commande activates the video capture
#cap = cv2.VideoCapture(0)
#cap.set(3,frameWidth)
#cap.set(4,frameHeight)


def empty(a):
    pass

#cv2.namedWindow("Parameters")
#cv2.resizeWindow("Parameters",800,600)
#cv2.createTrackbar("Thresh 1","Parameters",70,255,empty)
#cv2.createTrackbar("Thresh 2","Parameters",20,255,empty)
#cv2.createTrackbar("Threshold1","Parameters",11,20,empty)
#cv2.createTrackbar("Threshold2","Parameters",2,10,empty)

#Generates the slider bar window with values pre-set for the Red balloon
#as of now
#if VS == "red":
#    cv2.createTrackbar("H_HIGH ","Parameters", 193,255,empty)
#    cv2.createTrackbar("S_HIGH ","Parameters", 214,255,empty)
#    cv2.createTrackbar("V_HIGH ","Parameters", 187,255,empty)
#    cv2.createTrackbar("H_LOW ", "Parameters", 0, 255, empty)
#    cv2.createTrackbar("S_LOW ", "Parameters", 116,255,empty)
#    cv2.createTrackbar("V_LOW ","Parameters", 140, 255,empty)
#if VS == "blue":
#    cv2.createTrackbar("H_HIGH ","Parameters", 123,255,empty)
#    cv2.createTrackbar("S_HIGH ","Parameters", 255,255,empty)
#    cv2.createTrackbar("V_HIGH ","Parameters", 255,255,empty)
#    cv2.createTrackbar("H_LOW ", "Parameters", 84, 255, empty)
#    cv2.createTrackbar("S_LOW ", "Parameters", 148,255,empty)
#    cv2.createTrackbar("V_LOW ","Parameters", 96, 255,empty) 

#if VS == "yellow":
#    cv2.createTrackbar("H_HIGH ","Parameters", 255,255,empty)
#    cv2.createTrackbar("S_HIGH ","Parameters", 218,255,empty)
#    cv2.createTrackbar("V_HIGH ","Parameters", 255,255,empty)
#    cv2.createTrackbar("H_LOW ", "Parameters", 0, 255, empty)
#    cv2.createTrackbar("S_LOW ", "Parameters", 117,255,empty)
#    cv2.createTrackbar("V_LOW ","Parameters", 151, 255,empty)

#cv2.createTrackbar("Frame Ratio","Parameters",8,10,empty)
#cv2.createTrackbar("Sensetivity","Parameters", 4,10,empty)
#cv2.createTrackbar("Area Min","Parameters",15000,30000,empty)
#cv2.createTrackbar("Area Max","Parameters",400000,500000,empty)
#ROI_FRAME_RATIO = cv2.getTrackbarPos("Frame Ratio", "Parameters")/10


def comnd(ipt):
    while ipt == []:
        ipt = input("Please Enter a Command: ")
        if ipt == "list":
            print("List of Commands:   ")
            print("")
            print("Command:     What it Do:")
            print("")
            print("F            Forward")
            print("B            Back")
            print("S            Stop")
            print("R            90deg CW")
            print("r            90deg CCW")
            print("L            FWD Line")
            print("*            Charge Balloon")
            print("M            Maze Solver")
            print("I            IR Sensor Reading")
            print("D            Distance Check [cm]")
            print("-            Color Check")
            print("c            CW Large Radius")
            print("C            CCW Large Radius")
            print("T            Balloon Tracking")
            print("W            FOR THE WIN!") 

            print("Q            180deg CW")

            ipt = []
        else:
            #ipt = list(ipt)
            #ipt = ipt[1]
            print("Sending:   ",ipt)
            if ARDUINO:
                ser.write(str(ipt).encode('utf-8'))
                sleep(.5)
                while ser.in_waiting > 0 and ARDUINO == True:
                    line = ser.readline().decode('utf-8').rstrip()
                    print(line)
    return ipt


# Main loop of the script

while True:
    ipt = []
    ipt = comnd(ipt)
    if ARDUINO:
        ser.write(str(ipt).encode('utf-8'))
        #sleep(.5)
        while ser.in_waiting > 0 and ARDUINO == True:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)

#cap.release()
#cv2.destroyAllWindows()
