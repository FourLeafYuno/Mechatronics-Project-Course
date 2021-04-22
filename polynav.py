import cv2
import numpy as np
import math

frameWidth = 800
frameHeight = 600
cap = cv2.VideoCapture(0)
cap.set(3,frameWidth)
cap.set(4,frameHeight)

#SENS = 0.4
IMAGE_X_INDEX = 1
IMAGE_Y_INDEX = 0



def empty(a):
    pass

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters",800,600)
cv2.createTrackbar("Thresh 1","Parameters",150,255,empty)
cv2.createTrackbar("Thresh 2","Parameters",255,255,empty)
cv2.createTrackbar("Threshold1","Parameters",11,20,empty)
cv2.createTrackbar("Threshold2","Parameters",2,10,empty)

cv2.createTrackbar("Frame Ratio","Parameters",7,10,empty)
cv2.createTrackbar("Sensetivity","Parameters", 4,10,empty)
cv2.createTrackbar("Area","Parameters",5000,30000,empty)
ROI_FRAME_RATIO = cv2.getTrackbarPos("Frame Ratio", "Parameters")/10

ROWS = int(frameHeight * ROI_FRAME_RATIO)
COLS = int(frameWidth)

def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0,rows):
            for y in range(0,cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0,0),None,scale,scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),None,scale,scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height,width,3),np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0,rows): 
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0,rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x],(0,0),None,scale,scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1],imgArray[0].shape[0]),None,scale,scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x],cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = np.hstack(imgArray)
    return ver

def crop(img):
    IMAGE_Y_INDEX = 1
    IMAGE_X_INDEX = 0
    
    roi_vector = {'x' : 0, 'y' : 0, 'w' : 0, 'h': 0}
    roi_vector['w'] = img.shape[IMAGE_X_INDEX]
    roi_vector['h'] = int(img.shape[IMAGE_Y_INDEX] * cv2.getTrackbarPos("Frame Ratio","Parameters")/10)
    roi_vector['x'] = 0
    roi_vector['y'] = img.shape[IMAGE_Y_INDEX] - roi_vector['h']
    y_start = roi_vector['y']
    y_end = roi_vector['y'] + roi_vector['h']
    x_start = roi_vector['x']
    x_end = roi_vector['x'] + roi_vector['w']
    cropped = img[y_start:y_end, x_start:x_end]
    h = abs(y_start - y_end)
    w = abs(x_start - x_end)
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
    #Not sure what the adaptive threholding gets you but it looked interesting so I put it in here for later
    #If it works and is useful
    
    bin_image = cv2.adaptiveThreshold(img, 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,cv2.getTrackbarPos("Threshold1","Parameters"),cv2.getTrackbarPos("Threshold2","Parameters"))
    
    
    
    #ret,bin_image = cv2.threshold(img,cv2.getTrackbarPos("BW_THRESH","Parameters"), 255,cv2.THRESH_BINARY)
    kernel = np.ones((3,3),np.uint8)
    #bin_image = cv2.dilate(bin_image, kernel, iterations = 1)
    bin_image = cv2.erode(bin_image, kernel, iterations = 1)
    #bin_image = ~bin_image
    cv2.imshow("Binary",bin_image)
    contours, hierarchy = cv2.findContours(bin_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    return contours, hierarchy

def findpath(img,imgContour,points,contours,Vis):
    angle_deviation = 0
    distance_unobstructed = 0
    area_unobstructed = 0
    MIN_CONTOUR_AREA = cv2.getTrackbarPos("Area","Parameters") 
    rows,cols = 600,800

    if contours is None:
        return -1, -1, -1

    y_list = []
    cnt_list = []
    ROWS = int(frameHeight * cv2.getTrackbarPos("Frame Ratio","Parameters")/10)
    COLS = int(frameWidth)
    for cnt in contours:
        if cv2.contourArea(cnt) > MIN_CONTOUR_AREA:
            moments = cv2.moments(cnt)
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
            angle_deviation = int(math.atan((int(cols/2) - item[1])/(rows - item[2])) * 180 / math.pi)
            distance_unobstructed = int(math.sqrt(math.pow((int(cols/2) - item[1]),2) + math.pow((rows - item[2]),2)))
            area_unobstructed = int(cv2.contourArea(item[0]))
            cv2.drawContours(imgContour,cnt,-1, (0,255,0),5) 
            if Vis:
                cv2.circle(img,(item[1],item[2]), 3,3)
                cv2.drawContours(img, [item[0]], 0, (0,255,0),2)
                cv2.line(img,( int(800/2),600), (item[1], item[2]), (0,255,0),1)
                cv2.putText(img, str(angle_deviation), (item[1] + 10, item[2] + 10),cv2.FONT_HERSHEY_COMPLEX , 1, (255,0,0),2)
                cv2.putText(img, str(distance_unobstructed), (item[1] - 20, item[2] - 20),cv2.FONT_HERSHEY_COMPLEX , 1, (255,0,0),2)

    return angle_deviation, distance_unobstructed, area_unobstructed





def getContours(img,imgContour):


    contours,hierarchy = cv2.findContours(img,cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
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
            #cv2.rectangle(imgContour,(x,y),(x + w, y + h), (255,0,0),5)
            cv2.putText(imgContour,"Points: " + str(len(approx)),(x + w + 20, y + 20),cv2.FONT_HERSHEY_COMPLEX, .7, (0,0,255),2)
            cv2.putText(imgContour,"Area: " + str(int(area)), (x + w + 20, y + 45),cv2.FONT_HERSHEY_COMPLEX, 0.7,(0,0,255),2)
    return contours

while True:
    success, img = cap.read()
    print(img)
    cropped,h,w  = crop(img)
    #cv2.imshow("Cropped: ", cropped)
    imgContours = img.copy()
    Sens = cv2.getTrackbarPos("Sensetivity","Parameters")/10
    points = polygon(int(h),w,Sens) #polygon(int(frameHeight * Sens),frameWidth,Sens)
    #print(points)
    imgImp = superimpose(cropped, points) 
    #imgBlur = cv2.GaussianBlur(img, (5,5),1)
    imgGray = cv2.cvtColor(imgImp,cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.medianBlur(imgGray,3)
    imgBlur = cv2.GaussianBlur(imgGray,(5,5),0)
    #cv2.imshow("Blur",imgBlur)
    threshold1 = cv2.getTrackbarPos("Thresh 1","Parameters")
    threshold2 = cv2.getTrackbarPos("Thresh 2","Parameters")
    #imgCanny = cv2.Canny(imgGray,threshold1,threshold2)
    #imgCanny = cv2.Canny(imgImp,threshold1,threshold2)
    imgCanny = cv2.Canny(imgBlur,threshold1,threshold2)
    
    cv2.imshow("Canny", imgCanny)
    kernel = np.ones((7,7))
    imgDil = cv2.dilate(imgCanny,kernel,iterations=1)
    imgDil = ~imgDil
    cv2.imshow("Dil", imgDil)
    contours,hierarchy  = poly_cont(imgDil)
    #contours = getContours(imgDil,imgContours)
    findpath(cropped,imgContours,points,contours, True)
    #findpath(cropped,imgDil,points,contours, True)
    #imgStack = stackImages(0.8,([img,cropped],[img,img]))
    #cv2.imshow("Image:  ",imgDil)
    cv2.imshow("Final",cropped)
    #cv2.imshow("Result",imgStack)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

