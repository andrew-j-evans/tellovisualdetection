from djitellopy import Tello
import time
import cv2
import numpy as np
from math import *

global lower, upper, lower2, upper2

#HSV Values for color 1 (pink)
lower = np.array([140, 50, 20])
upper = np.array([179, 255, 255])

#HSV Values for color 2 (blue)
lower2 = np.array([80, 40, 20])
upper2 = np.array([130, 255, 255])

#The required contour area to count as detected object
minDetectionArea = 100

#Defines the tello object and connects to the drone
tello = Tello()
tello.connect()
print("Battery Life Percentage: " + str(tello.get_battery()) + "%")
tello.streamon()

#Makes sure all tello accelerations are at 0
time.sleep(0.25)
tello.send_rc_control(0,0,0,0)

#Gets tello screen width and height in pixels
img = tello.get_frame_read().frame
width = img.shape[1]
frameHeight = img.shape[0]
TelloDisplayPixels = width
print("Screen size (pixels): " + str(TelloDisplayPixels) + " by " + str(frameHeight))

tracebackDirection = []
tracebackDistance = []


def getMask():
#Function to mask a certain predefined color and determine the location of the object relative to the screen.

    #Captures image from Tello camera and reformats it
    img = tello.get_frame_read().frame
    time.sleep(0.1)
    image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #Creates two color masks, one of each color, and sligtly slurs them
    mask_color1 = cv2.inRange(image, lower, upper)
    mask_color2 = cv2.inRange(image, lower2, upper2)

    mask_color1 = cv2.medianBlur(mask_color1, 5)
    mask_color2 = cv2.medianBlur(mask_color2, 5)

    colorShow = cv2.bitwise_and(img, img, mask = mask_color1 + mask_color2)

    #Finds the contours of each mask area
    contours_color1, h = cv2.findContours(mask_color1,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_color2, h = cv2.findContours(mask_color2,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    x = y = w = h = 0
    x2 = y2 = w2 = h2 = 0
    xcenter = ycenter = 0

    centers_color1 = []
    centers_color2 = []

    #Iterates through each color of contour, if there exists a color of each contour where
    #the center is the same, then that is the target point.
    if len(contours_color1) != 0:
        for contour in contours_color1:
            if cv2.contourArea(contour) > minDetectionArea:
                x, y, w, h = cv2.boundingRect(contour)
                xcenter = x + w/2;
                ycenter = y + h/2
                centers_color1.append([xcenter,ycenter,w,h])

                cv2.rectangle(img, (x,y), (x+w, y+h), (0, 0, 255), 3)
                cv2.circle(img, (int(xcenter),int(ycenter)), 10, (0, 0, 255), -1)

    if len(contours_color2) != 0:
        for contour in contours_color2:
            if cv2.contourArea(contour) > minDetectionArea:
                x2, y2, w2, h2 = cv2.boundingRect(contour)
                xcenter2 = x2 + w2/2;
                ycenter2 = y2 + h2/2
                centers_color2.append([xcenter2,ycenter2,w2,h2])

                cv2.rectangle(img, (x2,y2), (x2+w2, y2+h2), (255, 0, 0), 3)
                cv2.circle(img, (int(xcenter2),int(ycenter2)), 10, (255, 0, 0), -1)

    targetX = targetY = targetW = targetH = 0


    for i in range(len(centers_color1)):
        for j in range(len(centers_color2)):           
            try:
                distance = dist([centers_color1[i][0],centers_color1[i][1]], [centers_color2[j][0],centers_color2[j][1]])
                if(int(distance) <= 5 and int(distance) >= 0):
                    targetX = centers_color1[i][0]
                    targetY = centers_color1[i][1]
                    targetW = centers_color1[i][2]
                    targetH = centers_color1[i][3]
            except:
                trash = 0

    cv2.putText(img, "Target", (int(targetX),int(targetY)), 3, 1, (0, 255, 0), 2, cv2.LINE_4)
    cv2.imshow("Color 1", colorShow)
    cv2.imshow("webcam", img)

    cv2.waitKey(1)
    
    return [targetX, targetY, targetW, targetH]

def findAngle():
#Finds the ideal angle the drone must turn in one go by using trigonometry.
#This may not be compltely accurate, as it is assuming the lens is completely linear,
#while it has curvature to it

    foundTarget = False
    CyclesWithTarget = 0
    MinValidCycles = 1

    while((not foundTarget and CyclesWithTarget < MinValidCycles)):
        CyclesWithTarget = 0
        [x ,y, w, h] = getMask()
        if(x != 0 or y != 0):
            CyclesWithTarget += 1
            foundTarget = True
        else:
            CyclesWithTarget = 0
    #41.3
    #33.04
    theta = 33.04 #Tello has a field of fiew of 82.6 degrees, which divided by 2 is 41.3
    xmax  = TelloDisplayPixels #Replace with the number of pixels on the drone camera screen
    L = (xmax) / (2*tan(radians(theta)))

    degreeModifier = 1;

    if(x > xmax / 2):
        calcx = x - (xmax / 2)

        #Multiply by -1 to signal clockwise rotation
        degreeModifier = -1
    else:
        #Change for formula for degrees depending on horizontal positioning
        calcx = (xmax / 2) - x
        
    degree = round(degrees(atan(calcx / L))) * degreeModifier

    return degree
    
def changeAngle(angle):
#Changes the angle using the calculated angle in findAngle()
    RotateDegrees = angle
    if(RotateDegrees < 0):
        print("Angle : " + str(-1 * RotateDegrees))
        tello.rotate_clockwise(-1 * RotateDegrees)
    else:
        print("Angle : " + str(RotateDegrees))
        tello.rotate_counter_clockwise(RotateDegrees)

def angleSpeed(distanceFromCenter):
#Calculates the speed the drone should turn by making it slower the closer it is to the center
    if(distanceFromCenter > 200):
        return 20
    else:
        speed = distanceFromCenter / 10
        return round(speed)

def changeAngle2():
#Changes the angle with more precision by moving a small amount multiple times to center the target

    foundTarget = False
    CyclesWithTarget = 0
    MinValidCycles = 1
    LimitAttempts = 1
    LimitAttemptsBool = False
    if(LimitAttempts > 0):
        LimitAttemptsBool = True

    while((not foundTarget and CyclesWithTarget < MinValidCycles) and (not LimitAttemptsBool or LimitAttempts >= 0)):
        CyclesWithTarget = 0
        LimitAttempts = LimitAttempts - 1
        [x ,y, w, h] = getMask()
        
        if(x != 0 or y != 0):
            #Found target, incrememnt target cycles
            CyclesWithTarget += 1
            foundTarget = True
        else:
            CyclesWithTarget = 0
    xmax  = TelloDisplayPixels

    MOE = 30

    speed = 0
    count = 0
    while(count < 30):

        [x ,y, w, h] = getMask()
        while(x == 0):
                [x ,y, w, h] = getMask()

        while(x > xmax / 2 + MOE):
            #clockwise
            dist = (x - xmax / 2)
            tello.send_rc_control(0,0,0,angleSpeed(dist))


            [x ,y, w, h] = getMask()
            while(x == 0):
                [x ,y, w, h] = getMask()

        tello.send_rc_control(0,0,0,0)
        

        while(x < xmax / 2 - MOE):
            dist = (xmax / 2 - x)
            tello.send_rc_control(0,0,0,-angleSpeed(dist))

            [x ,y, w, h] = getMask()
            while(x == 0):
                [x ,y, w, h] = getMask()
        tello.send_rc_control(0,0,0,0)

        if(x < xmax / 2 + MOE and x > xmax / 2 - MOE):
            count = count + 1
            print(count)
        else:
            count = 0

    print("On target homie")

def distanceToTarget():
#Uses a ratio of pixels per square inch to determine the distance of the target from the drone.
    maxLength = 0
    distances = []
    i = 0
    while(i < 10):
        [x, y, w, h] = getMask()
        if(w != 0):
            i = i + 1
        if(w) > maxLength:
            maxLength = w



    print("Max Length: " + str(maxLength))
    distanceAway = 0
    cDIn = 8.125

    cDiameter = 470
    distanceAway = 1 * cDiameter / maxLength

    #Changes the distance from feet to centimeters
    distCenti = distanceAway * 30.48

    #70.485 height of the table we are using
    Height = tello.get_distance_tof() - 70.485
    

    LinearDistance = sqrt(distCenti*distCenti-Height*Height)

    print("Distance = " + str(LinearDistance))
    print("Height = " + str(Height))

    return LinearDistance

def telloFlyInSegments():
#Performs the final flight in two iterations, as the calibrations were made at 200 centimeters away.
#Flies to 200 centimeters away from the target, then performs every check again at 200 centmeters.

    changeAngle(findAngle())
    time.sleep(0.25)
    changeAngle2()

    time.sleep(0.5)

    distance = distanceToTarget()
    

    if(distance < 220):
        tello.move_forward(int(distance))
    else:
        tello.move_forward(int(distance - 200))
        time.sleep(1)
        
        telloSetHeight(160)

        found = False
        for i in range(10):
            x, y, z, h = getMask()
            if(x != 0):
                found = True

        time.sleep(1)
        if(found):
            changeAngle2()
            time.sleep(0.25)
            distance = distanceToTarget()
            tello.move_forward(int(distance))
        else:
            tello.move_forward(int(200))
  
def telloSetHeight(height):
#Uses current known height to decrease or increase to another known height
    t_height = tello.get_distance_tof()
    print(t_height)
    if(t_height > height + 20):
        tello.move_down(t_height - height)

def seesTarget(checks):
#Performs a check if the target is in field of view or not
    for i in range(checks):
        [x, y, z, h] = getMask()
        if(x != 0):
            return True
    return False

def telloLeavePad(distance):
#Leaves the target after it has landed on it
    time.sleep(1)
    tello.takeoff()
    time.sleep(1)
    tello.rotate_clockwise(180)
    time.sleep(1)
    tello.move_forward(distance)
    time.sleep(1)
    tello.land()
    tello.end()

def telloFinalFlight():
#Performs the final Tello test flight for target identification

    sleeptime = 2

    flightTime = 0

    tello.takeoff()
    time.sleep(sleeptime + 4)
    tello.move_up(120) #120
    time.sleep(sleeptime + 2)
    tello.move_forward(175)
    time.sleep(sleeptime)
    tello.rotate_clockwise(180)
    time.sleep(sleeptime)
    

    sawTarget = False
    heightRuns = [120, 140]
    i = 0

    while(not sawTarget and i < len(heightRuns)):

        print("Adjusting Height")
        telloSetHeight(heightRuns[i])
        time.sleep(sleeptime)

        print("Looking for target")
        if(seesTarget(30)):
            telloFlyInSegments()
            sawTarget = True
            break;
        else:
            tello.rotate_clockwise(30)
        
            time.sleep(sleeptime)

            if(seesTarget(30)):
                telloFlyInSegments()
                sawTarget = True
                break;
            else:
                tello.rotate_counter_clockwise(60)
                
                time.sleep(sleeptime)
                
                if(seesTarget(30)):
                    telloFlyInSegments()
                    sawTarget = True
                    break;
        
        tello.rotate_clockwise(30)
        i = i + 1

    flightTime = tello.get_flight_time()
    tello.land()
    time.sleep(sleeptime)


    if(sawTarget):
        print("Exiting the premesis")
        telloLeavePad(100)
        print("Flight successful.")
        print("Total flight time of " + str(flightTime) + "seconds")
    else:
        tello.land()
        time.sleep(sleeptime)
        tello.stop()
        print("Error: no target detected")

telloFinalFlight()
