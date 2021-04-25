#!/usr/bin/cv RPi.GPIO
import cv2
import numpy as np
from time import sleep
from signal import pause
import serial
import os
import math

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
cap = cv2.VideoCapture(0)
cap.set(3,frameWidth)
cap.set(4,frameHeight)


def empty(a):
    pass

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters",800,600)
cv2.createTrackbar("Thresh 1","Parameters",70,255,empty)
cv2.createTrackbar("Thresh 2","Parameters",20,255,empty)
cv2.createTrackbar("Threshold1","Parameters",11,20,empty)
cv2.createTrackbar("Threshold2","Parameters",2,10,empty)

#Generates the slider bar window with values pre-set for the Red balloon
#as of now
if VS == "red":
    cv2.createTrackbar("H_HIGH ","Parameters", 193,255,empty)
    cv2.createTrackbar("S_HIGH ","Parameters", 214,255,empty)
    cv2.createTrackbar("V_HIGH ","Parameters", 187,255,empty)
    cv2.createTrackbar("H_LOW ", "Parameters", 0, 255, empty)
    cv2.createTrackbar("S_LOW ", "Parameters", 116,255,empty)
    cv2.createTrackbar("V_LOW ","Parameters", 140, 255,empty)
if VS == "blue":
    cv2.createTrackbar("H_HIGH ","Parameters", 123,255,empty)
    cv2.createTrackbar("S_HIGH ","Parameters", 255,255,empty)
    cv2.createTrackbar("V_HIGH ","Parameters", 255,255,empty)
    cv2.createTrackbar("H_LOW ", "Parameters", 84, 255, empty)
    cv2.createTrackbar("S_LOW ", "Parameters", 148,255,empty)
    cv2.createTrackbar("V_LOW ","Parameters", 96, 255,empty) 

if VS == "yellow":
    cv2.createTrackbar("H_HIGH ","Parameters", 255,255,empty)
    cv2.createTrackbar("S_HIGH ","Parameters", 218,255,empty)
    cv2.createTrackbar("V_HIGH ","Parameters", 255,255,empty)
    cv2.createTrackbar("H_LOW ", "Parameters", 0, 255, empty)
    cv2.createTrackbar("S_LOW ", "Parameters", 117,255,empty)
    cv2.createTrackbar("V_LOW ","Parameters", 151, 255,empty)

cv2.createTrackbar("Frame Ratio","Parameters",8,10,empty)
cv2.createTrackbar("Sensetivity","Parameters", 4,10,empty)
cv2.createTrackbar("Area Min","Parameters",15000,30000,empty)
cv2.createTrackbar("Area Max","Parameters",400000,500000,empty)
ROI_FRAME_RATIO = cv2.getTrackbarPos("Frame Ratio", "Parameters")/10

ROWS = int(frameHeight * ROI_FRAME_RATIO)
COLS = int(frameWidth)

#Function to draw contours based on the image and parameters provided
def getContours(img,imgContour):
    global cent, area, b
    contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > cv2.getTrackbarPos("Area Min","Parameters"):
            cv2.drawContours(imgContour,cnt,-1,(0,255,0),5)
            pari = cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,0.02 * pari, True)
            #print(len(approx))
            x,y,w,h = cv2.boundingRect(approx)
            cent = x+(w/2)
            cv2.rectangle(imgContour, (x,y),(x + w, y + h), (255,0,0),5)
            cv2.putText(imgContour, "Points: " + str(len(approx)),(x + w + 20, y + 20),cv2.FONT_HERSHEY_COMPLEX, .7, (0,0,255),2)
            cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX,0.7,(0,0,255),2)
            b = True

def crop(img):
    IMAGE_Y_INDEX = 1
    IMAGE_X_INDEX = 0

    roi_vector = {'x' : 0, 'y' : 0, 'w' : 0, 'h': 0}
    roi_vector['w'] = 800
    roi_vector['h'] = int(600 * (cv2.getTrackbarPos("Frame Ratio","Parameters")/10))
    roi_vector['x'] = 0
    roi_vector['y'] = img.shape[IMAGE_Y_INDEX] - roi_vector['h']
    y_start = roi_vector['y']
    y_end = roi_vector['y'] + roi_vector['h']
    x_start = roi_vector['x']
    x_end = roi_vector['x'] + roi_vector['w']
    cropped = img[y_start:y_end, x_start:x_end]
    h = abs(y_start - y_end)
    w = abs(x_start + x_end)
    print(h,w)
    return cropped, h, w

def polygon(ROWS,COLS,SENS):
    poly_points = np.array([[int(COLS * 0.25), ROWS],
        [int (COLS * 0.75), ROWS],
        [COLS, int(ROWS * SENS)],
        [COLS, int(ROWS * 0.25)],
        [0,int(ROWS * 0.25)],
        [0,int(ROWS * SENS)]], np.int32)
    return poly_points

def superimpose(img,points):
    cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    cv2.polylines(img,[points],True,(0,0,0),5)
    return img

def poly_cont(img):

    bin_image = cv2.adaptiveThreshold(img, 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,cv2.getTrackbarPos("Threshold1","Parameters"),cv2.getTrackbarPos("Threshold2","Parameters"))



     #ret,bin_image = cv2.threshold(img,cv2.getTrackbarPos("BW_THRESH","Parameters"), 255,cv2.THRESH_BINARY)
    kernel = np.ones((3,3),np.uint8)
    #bin_image = cv2.dilate(bin_image, kernel, iterations = 1)
    bin_image = cv2.erode(bin_image, kernel, iterations = 1)
    #bin_image = ~bin_image
    cv2.imshow("Binary",bin_image)
    contours, hierarchy = cv2.findContours(bin_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    #Might want to put the ConvexHull here
    return contours, hierarchy

def findpath(img,imgContour,points,contours,Vis):
    angle_deviation = 0
    distance_unobstructed = 0
    area_unobstructed = 0
    MIN_CONTOUR_AREA = cv2.getTrackbarPos("Area Min","Parameters")
    MAX_CONTOUR_AREA = cv2.getTrackbarPos("Area Max","Parameters")
    rows,cols = 480, 640

    if contours is None:
        return -1, -1, -1

    y_list = []
    cnt_list = []
    ROWS = int(frameHeight * cv2.getTrackbarPos("Frame Ratio","Parameters")/10)
    COLS = int(frameWidth) 
    for cnt in contours:
        pari = cv2.arcLength(cnt,True)
        approx = cv2.approxPolyDP(cnt,0.02 * pari, True)
        #print(len(approx))
        if cv2.contourArea(cnt) > MIN_CONTOUR_AREA and cv2.contourArea(cnt) < MAX_CONTOUR_AREA:
            cnt = cv2.convexHull(cnt)
            moments = cv2.moments(cnt)
            #approx = cv2.approxPolyDP(cnt,0.02 * pari, True) 
            com_x = int(moments['m10']/moments['m00'])
            com_y = int(moments['m01']/moments['m00'])
            if cv2.pointPolygonTest(points,(com_x,com_y),True) > 0:
                cnt_list.append([cnt, com_x,com_y])
                y_list.append(com_y)
    if y_list is not None:
        y_list.sort()
    else:
        return -1, -1, -1
    for item in cnt_list:
        if item[2] == y_list[-1]:
            angle_deviation = int(math.atan((int(COLS/2) - item[1])/(ROWS - item[2])) * 180 / math.pi)
            distance_unobstructed = int(math.sqrt(math.pow((int(COLS/2) - item[1]),2) + math.pow((ROWS - item[2]),2)))
            area_unobstructed = int(cv2.contourArea(item[0]))
            cv2.drawContours(imgContour,cnt,-1, (0,255,0),5)
            if Vis:
                cv2.circle(img,(item[1],item[2]), 3,3)
                cv2.drawContours(img, [item[0]], 0, (0,255,0),2)
                cv2.line(img,(int(COLS/2),ROWS), (item[1], item[2]), (0,255,0),1)
                cv2.putText(img, str(angle_deviation), (item[1] + 10, item[2] + 10),cv2.FONT_HERSHEY_COMPLEX , 1, (255,0,0),2)
                cv2.putText(img, str(distance_unobstructed), (item[1] - 20, item[2] - 20),cv2.FONT_HERSHEY_COMPLEX , 1, (255,0,0),2)

    return angle_deviation, distance_unobstructed, area_unobstructed

def comnd(ipt):
    while ipt == []:
        ipt = input("Please Enter a Command: ")
        if ipt == "list":
            print("List of Commands:   ")
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
    while ipt == "T":
        _,frame = cap.read()
        #print(len(frame))
        imgContours = frame.copy()
        #print("Inside the Balloon while")
        # Thresholds for contour stuff (unused as of now I think)
        threshold1 = cv2.getTrackbarPos("Thresh 1","Parameters")
        threshold2 = cv2.getTrackbarPos("Thresh 2","Parameters")

        # Higher and lower thresholds of the HSV values
        lg = np.array([cv2.getTrackbarPos("H_LOW ","Parameters"),cv2.getTrackbarPos("S_LOW ","Parameters"),cv2.getTrackbarPos("V_LOW ","Parameters")])
        ug = np.array([cv2.getTrackbarPos("H_HIGH ","Parameters"),cv2.getTrackbarPos("S_HIGH ","Parameters"),cv2.getTrackbarPos("V_HIGH ","Parameters")])
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        imgBlur = cv2.GaussianBlur(hsv,(7,7),1)
        #testBlur = cv2.GaussianBlur(frame,(7,7),1)
        # mask is the masked image of the blured hsv that seeks values of h,s,and v
        # within the specified bounds
        mask = cv2.inRange(hsv, lg, ug)

        #print(len(mask))

        #imgGray and Canny are unused currently, haven't figured out if this is
        #Something we would like to pursue in the future.
        imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
        imgCanny = cv2.Canny(imgGray,threshold1,threshold2)
        kernel = np.ones((5,5))
        imgDil = cv2.dilate(mask, kernel, iterations = 1)
        b = False
        getContours(imgDil, imgContours)
        #Image outputs for the pure image, the masked image, and the contours
        cv2.imshow("Frame: ",frame)
        cv2.imshow("HSV:   ",hsv)
        cv2.imshow("HSV Blur  ", imgBlur)
        cv2.imshow("Mask: ",mask)
        cv2.imshow("imgDil",imgDil)
        cv2.imshow("Contours: ", imgContours)
        print(b)
        #print(area)
        if (b == True and area < 100000 and ARDUINO == True):
            #print(cent)
            if (cent < 80):
                ser.write(str(2).encode('utf-8'))
                #sleep(.4)
                print("left")
            if (cent > 510):
                #ser.write(str(3).encode('utf-8'))
                print("right")
                #sleep(.4)
            if (cent >=  80 and cent <= 510):
                ser.write(str(1).encode('utf-8'))
                print("straight")
                #sleep(.4)
            ser.write(str(0).encode('utf-8'))
        # Command to read back what was sent to the Arduino
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    while ipt == "P":
        threshold1 = cv2.getTrackbarPos("Thresh 1","Parameters")
        threshold2 = cv2.getTrackbarPos("Thresh 2","Parameters")
        lg = np.array([cv2.getTrackbarPos("H_LOW ","Parameters"),cv2.getTrackbarPos("S_LOW ","Parameters"),cv2.getTrackbarPos("V_LOW ","Parameters")])
        ug = np.array([cv2.getTrackbarPos("H_HIGH ","Parameters"),cv2.getTrackbarPos("S_HIGH ","Parameters"),cv2.getTrackbarPos("V_HIGH ","Parameters")])
        success, img = cap.read()
        #print("Hello")
        cropped,h,w  = crop(img)
        #print("Goodbye")
        imgHSV = cv2.cvtColor(cropped,cv2.COLOR_BGR2HSV)
        imgMask = cv2.inRange(imgHSV,lg,ug)
        #cv2.imshow("Mask: ", imgMask)
        imgContours = img.copy()
        Sens = cv2.getTrackbarPos("Sensetivity","Parameters")/10
        points = polygon(int(h),w,Sens)
        imgImp = superimpose(cropped, points)

        imgGray = cv2.cvtColor(imgImp,cv2.COLOR_BGR2GRAY)
        imgBlur = cv2.medianBlur(imgGray,3)
        imgBlur = cv2.GaussianBlur(imgGray,(5,5),0)
        #cv2.imshow("Blur",imgBlur)

        imgCanny = cv2.Canny(imgBlur,threshold1,threshold2)

        cv2.imshow("Canny", imgCanny)
        kernel = np.ones((7,7))
        imgDil = cv2.dilate(imgCanny,kernel,iterations=1)
        imgDil = ~imgDil
        #cv2.imshow("Dil", imgDil)
        contours,hierarchy  = poly_cont(imgDil)
        contours_p = getContours(imgMask,cropped)
        angle_deviation, distance_unobstructed, area_unobstructed = findpath(cropped,imgContours,points,contours, True)

        #Currently the ser.write is sending integers one character at a time. Cant get it to take them as a whole.
        #Also might have to take the sleep out to get better performance.(4/23 7:18p)
        if ARDUINO and angle_deviation < 30:
            #ser.write(int(angle_deviation)) 
            #ser.write(int(angle_deviation).encode('utf-8'))
            if (angle_deviation < -10):
                ser.write(str(2).encode('utf-8'))
                sleep(.1)
                print("left")
            if (angle_deviation > 10):
                ser.write(str(3).encode('utf-8'))
                print("right")
                sleep(.1)
            if (angle_deviation >=  -10 and angle_deviation <= 10):
                ser.write(str(1).encode('utf-8'))
                print("straight")
                sleep(.1)
            #sleep(0.4)
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                print(line)
        #findpath(cropped,imgDil,points,contours, True)
        #imgStack = stackImages(0.8,([img,cropped],[img,img]))
        #cv2.imshow("Image:  ",imgDil)
        cv2.imshow("Final",cropped)
        #cv2.imshow("Result",imgStack)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
cap.release()
cv2.destroyAllWindows()
