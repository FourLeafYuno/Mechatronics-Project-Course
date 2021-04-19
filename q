import cv2
import numpy as np

frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(0)
cap.set(3,frameWidth)
cap.set(4,frameHeight)

def empty(a):
    pass

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters",640,240)
cv2.createTrackbar("Threshold1","Parameters",150,255,empty)
cv2.createTrackbar("Threshold2","Parameters",255,255,empty)
cv2.createTrackbar("Area","Parameters",5000,30000,empty)



def getContours(img,imgContour):


    contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #(255,0,255) color ||| 7 = width
    #cv2.drawContours(imgContour, contours, -1, (255,0,255),7)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = cv2.getTrackbarPos("Area","Parameters")
        if area > areaMin:
            cv2.drawContours(imgContour,cnt,-1, (0,255,0),5)
            pari = cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt, 0.02 * pari, True)
            print(len(approx))
            x,y,w,h = cv2.boundingRect(approx)
            cv2.rectangle(imgContour,(x,y),(x + w, y + h), (255,0,0),5)
            cv2.putText(imgContour,"Points: " + str(len(approx)),(x + w + 20, y + 20),cv2.FONT_HERSHEY_COMPLEX, .7, (0,0,255),2)
            cv2.putText(imgContour,"Area: " + str(int(area)), (x + w + 20, y + 45),cv2.FONT_HERSHEY_COMPLEX, 0.7,(0,0,255),2)
while True:
    success, img = cap.read()
    
    imgContoursY = img.copy()
    imgContoursP = img.copy()

    uy = np.array([52,255,255])
    ly = np.array([0,81,97])
    
    up = np.array([200, 255, 255])
    lp = np.array([100, 14, 71])
    
    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    imgYMask = cv2.inRange(imgHSV,ly,uy)
    imgPMask = cv2.inRange(imgHSV,lp,up)

    imgBlur = cv2.GaussianBlur(img, (7,7),1)
    imgGray = cv2.cvtColor(imgBlur,cv2.COLOR_BGR2GRAY)
    
    threshold1 = cv2.getTrackbarPos("Threshold1","Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2","Parameters")
    imgCanny = cv2.Canny(imgGray,threshold1,threshold2)
    kernel = np.ones((5,5))
    
    
    imgDilY = cv2.dilate(imgYMask,kernel,iterations=1)
    imgDilP = cv2.dilate(imgPMask,kernel,iterations=1)
    
    getContours(imgDilY,imgContoursY)
    getContours(imgDilP, imgContoursP)

    #imgStack = stackImages(0.8,([img,imgCanny],[imgDil,imgContours]))
    cv2.imshow("Y Mask",imgYMask)
    cv2.imshow("P Mask",imgPMask)
    #cv2.imshow("Dilate",imgDil)
    cv2.imshow("Yellow Contours",imgContoursY)
    cv2.imshow("Purple Contours",imgContoursP)
    #cv2.imshow("Result",imgStack)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
