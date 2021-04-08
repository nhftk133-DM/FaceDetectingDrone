from utils import *
import cv2

w, h = 360, 240
pid = [0.5, 0.5, 0]
pErrorYaw = 0
pErrorForBack = 0
pErrorUpDown = 0
startCounter = 0  # for no Flight 1   - for flight 0


myDrone = initializeTello()  # Function from utils
faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')  # calling haarcascade file
count = 0

while True:
    count + 1

    # Start flight
    if startCounter == 0:
        myDrone.takeoff()
        startCounter = 1

    # Step 1 - get tello image
    img = telloGetFrame(myDrone, w, h)  # Function from utils getting image for tello camera
    # Step 2 - detect face from tello image
    if count % 5 == 0:

        img, info = findFace(img, faceCascade)  # The img parameter gating all of the findFace function from util
        # Step 3 - face tracking YAW and ForBack
        pErrorYaw = trackFace(myDrone, info, w, h, pid, pErrorYaw, pErrorForBack,pErrorUpDown)[0]  # putting the previous error to pErrorYaw parameter
        pErrorForBack = trackFace(myDrone, info, w, h, pid, pErrorYaw, pErrorForBack,pErrorUpDown)[1]  # putting the previous error to pErrorForBack parameter
        pErrorUpDown = trackFace(myDrone, info, w, h, pid, pErrorYaw, pErrorForBack, pErrorUpDown)[2]  # putting the previous error to pErrorForBack parameter
        # print(info[1],[0]) # Print the coordination CX
    else:
        print("waiting")

    cv2.imshow('Image', img)  # displaying drone image
    if cv2.waitKey(1) & 0xFF == ord('q'):  # safety feature press 'q' in order to land the drone
        myDrone.land()
        break
