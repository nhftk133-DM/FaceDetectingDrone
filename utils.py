from djitellopy import Tello  # drone control library
import cv2  # openCV library
import numpy as np
import time

Framecount = 0

def initializeTello():  # Function from utils
    myDrone = Tello()  # Creating drone object
    myDrone.connect()  # Connect to the drone
    myDrone.for_back_velocity = 0  # Starting X velocity
    myDrone.left_right_velocity = 0  # Starting Y velocity
    myDrone.up_down_velocity = 0  # Starting Z velocity
    myDrone.yaw_velocity = 0  # Starting yaw velocity
    myDrone.speed = 0  # Starting general velocity
    print(myDrone.get_battery())  # print tello battery percentage
    myDrone.streamoff()  # turn off stream in case of turning on last run
    myDrone.streamon()  # turn on stream
    return myDrone  # initializeTello returns our object


def telloGetFrame(myDrone, w=360, h=240):
    myFrame = myDrone.get_frame_read()  # new object "myFrame" is getting the drones frames
    myFrame = myFrame.frame
    img = cv2.resize(myFrame, (w, h))  # resizing the frame to another resolution
    return img  # telloGetFrame returns our frame for the drone camera




# Viola–Jones object detection framework
def findFace(img, faceCascade):
      # calling haarcascade file
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Converting the color of the image to Grayscale
    faces = faceCascade.detectMultiScale(imgGray, 1.1, 5)
    # arg1 - image
    # arg2 - scaleFactor: Parameter specifying how much the image size is reduced at each image scale.
    # arg3 - minNeighbors:  Parameter specifying how many neighbors each candidate rectangle should have retain it pixel minNeighbors.

    myFaceListC = []  # list for storing our central point of our all faces "x and y" as "CX and CY"
    myFaceListArea = []  # list for storing our area to find witch one is the closer (the largest)

    #  Find all faces and drew a rectangle
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3)  # drawing the rectangle
        cx = x + w // 2  # Center X
        cy = y + h // 2  # Center Y
        area = w * h  # Face area
        myFaceListArea.append(area)  # adding a single item of area parameter to myFaceListArea
        myFaceListC.append([cx, cy])  # adding a single item of (CX,CY) parameters to myFaceListArea myFaceListC

    if len(myFaceListArea) != 0:  # Chaking if a face detected in the frame
        i = myFaceListArea.index(max(myFaceListArea))  # Finding the index to the largest area
        return img, [myFaceListC[i], myFaceListArea[i]]  # If face is found then return the image and the two lists
    else:
        return img, [[0, 0], 0]  # else return the image with default parameters as indicators


    #  Face tracking
def trackFace(myDrone, info, w, h, pid, pErrorYaw, pErrorForBack, pErrorUpDown):
    ## PID
    errorYaw = info[0][0] - w // 2  # [CX] - [image_width/2], determining witch error on the yaw axis (right or left)
    speedYaw = pid[0] * errorYaw + pid[1] * (errorYaw - pErrorYaw)  # kp*error + kd*(error - pErrorYaw) were pErrorYaw is previous Yaw error
    speedYaw = int(np.clip(speedYaw, -100, 100))  # constrain it so it will not exceed the limit ***{must be an integer}

    errorForBack = 50 - info[1]/100  # Area, determining witch error on the X axis (Front or Back)
    speedForBack = pid[0] * errorForBack + pid[1] * (errorForBack - pErrorForBack)  # kp*errorForBack + kd*(errorForBack - pErrorForBack) were pErrorForBack is previous ForBack error
    speedForBack = int(np.clip(speedForBack, -100, 100))  # constrain it so it will not exceed the limit ***{must be an integer}

    errorUpDown = h // 2.3 - info[0][1]  # [CY] - [image_height/2], determining witch error on the yaw axis (up or down)
    speedUpDown = pid[0] * errorUpDown + pid[1] * (errorUpDown - pErrorUpDown)  # kp*errorForBack + kd*(errorForBack - pErrorForBack) were pErrorForBack is previous ForBack error
    speedUpDown = int(np.clip(speedUpDown, -100, 100))  # constrain it so it will not exceed the limit ***{must be an integer}

    print(speedUpDown)

    if info[0][0] != 0:
        myDrone.yaw_velocity = speedYaw  # if we detect a face then we sending to the drone to trace of the face in yaw axis
        myDrone.for_back_velocity = speedForBack  # if we detect a face then we sending to the drone to trace of the face in ForBack axis
        myDrone.up_down_velocity = speedUpDown
    else:  # setting the values
        myDrone.for_back_velocity = 0
        myDrone.left_right_velocity = 0
        myDrone.up_down_velocity = 0
        myDrone.yaw_velocity = 0
        errorYaw = 0
        errorForBack = 0

    if myDrone.send_rc_control:  # sanding the values "send_rc_control is bool"
        myDrone.send_rc_control(myDrone.left_right_velocity,
                                myDrone.for_back_velocity,
                                myDrone.up_down_velocity,
                                myDrone.yaw_velocity)
    return errorYaw, errorForBack, errorUpDown
