import cv2
import numpy as np

frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(0)
cap.set(3,frameWidth)
cap.set(4,frameHeight)

width, height = 350,350
pts1 = np.float32([[0,219],[640,219],[0,480],[640,480]])
pts2 = np.float32([[0,0],[width,0],[0,height],[width,height]])
matrix = cv2.getPerspectiveTransform(pts1,pts2)

def empty(a):
    pass

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters",640,240)
cv2.createTrackbar("Threshold1","Parameters",150,255,empty)
cv2.createTrackbar("Threshold2","Parameters",255,255,empty)
cv2.createTrackbar("Area","Parameters",5000,30000,empty)

cv2.namedWindow("Thresholds")
cv2.resizeWindow("Thresholds",640,240)
cv2.createTrackbar("H_HIGH ","Parameters", 240,255,empty)
cv2.createTrackbar("S_HIGH ","Parameters", 255,255,empty)
cv2.createTrackbar("V_HIGH ","Parameters", 255,255,empty)
cv2.createTrackbar("H_LOW ", "Parameters", 0, 255, empty)
cv2.createTrackbar("S_LOW ", "Parameters", 100,255,empty)
cv2.createTrackbar("V_LOW ","Parameters", 100, 255,empty)



def getContours(img,imgContour):
    global cent, x, y, angle

    contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #(255,0,255) color ||| 7 = width
    #cv2.drawContours(imgContour, contours, -1, (255,0,255),7)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        (x,y),(MA,ma),angle = cv2.fitEllipse(cnt)
        areaMin = cv2.getTrackbarPos("Area","Parameters")
        if area > areaMin:
            cv2.drawContours(imgContour,cnt,-1, (0,255,0),5)
            pari = cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt, 0.02 * pari, True)
            print(len(approx))
            x,y,w,h = cv2.boundingRect(approx)
            cent = x+(w/2)
            cv2.rectangle(imgContour,(x,y),(x + w, y + h), (255,0,0),5)
            cv2.putText(imgContour,"Points: " + str(len(approx)),(x + w + 20, y + 20),cv2.FONT_HERSHEY_COMPLEX, .7, (0,0,255),2)
            cv2.putText(imgContour,"Area: " + str(int(area)), (x + w + 20, y + 45),cv2.FONT_HERSHEY_COMPLEX, 0.7,(0,0,255),2)
while True:
    success, img = cap.read()
    
    imgContoursY = img.copy()
    imgContoursP = img.copy()

    #uy = np.array([40,255,255])
    #ly = np.array([0,100,100])
    
    ly = np.array([cv2.getTrackbarPos("H_LOW ","Parameters"),cv2.getTrackbarPos("S_LOW ","Parameters"),cv2.getTrackbarPos("V_LOW ","Parameters")])
    uy = np.array([cv2.getTrackbarPos("H_HIGH ","Parameters"),cv2.getTrackbarPos("S_HIGH ","Parameters"),cv2.getTrackbarPos("V_HIGH ","Parameters")])
    
    up = np.array([200, 255, 255])
    lp = np.array([100, 14, 71])
    
    for x in range (0,4):
        cv2.circle(img,(pts1[x][0],pts1[x][1]),15,(0,255,0),cv2.FILLED)
    imgOutput = cv2.warpPerspective(img,matrix,(width,height))
    
    imgHSV = cv2.cvtColor(imgOutput,cv2.COLOR_BGR2HSV)

    imgYMask = cv2.inRange(imgHSV,ly,uy)
    imgPMask = cv2.inRange(imgHSV,lp,up)

    imgBlur = cv2.GaussianBlur(imgOutput, (7,7),1)
    imgGray = cv2.cvtColor(imgBlur,cv2.COLOR_BGR2GRAY)
    
    threshold1 = cv2.getTrackbarPos("Threshold1","Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2","Parameters")
    imgCanny = cv2.Canny(imgGray,threshold1,threshold2)
    kernel = np.ones((5,5))
    
    
    imgDilY = cv2.dilate(imgYMask,kernel,iterations=1)
    imgDilP = cv2.dilate(imgPMask,kernel,iterations=1)
    
    getContours(imgDilY,imgContoursY)
    #getContours(imgDilP, imgContoursP)

    #(x,y),(MA,ma),angle = cv2.fitEllipse(imgContoursY)
    print("X:   ", x)
    print("Y:   ", y)
    print("Angle:  ",angle)
    #imgStack = stackImages(0.8,([img,imgCanny],[imgDil,imgContours]))
    cv2.imshow("Y Mask",imgYMask)
    cv2.imshow("P Mask",imgPMask)
    #cv2.imshow("Dilate",imgDil)
    cv2.imshow("Yellow Contours",imgContoursY)
    cv2.imshow("Purple Contours",imgContoursP)
    #cv2.imshow("Result",imgStack)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
