#!/usr/bin/env python

from mavsdk import System
from mavsdk.follow_me import (Config, FollowMeError, TargetLocation)
from mavsdk.offboard import (Attitude, OffboardError)
import cv2
import numpy as np
from sys import base_prefix

import cv2.aruco as aruco
import sys, time, math
import serial
# from video import Video

import time
import yaml
# from dt_apriltags import Detector

import asyncio

import random

import subprocess
import time
from collections import defaultdict
from enum import Enum
# from pynput.keyboard import Listener, Key, KeyCode
from mavsdk.gimbal import GimbalMode

# from pynput.keyboard import Key, Controller
from util import ImageUtil

default_height = 8.0
follow_distance = 1.0
direction = Config.FollowDirection.BEHIND
responsiveness = 0.02

distance = 400


cap_props = {'focus_auto': 0, 'focus_absolute': 20, 'white_balance_temperature_auto': 0, 'white_balance_temperature': 6500
             }

for key in cap_props:
    subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format(key, str(cap_props[key]))],
                    shell=True)


def drawCorner(frame, corners, center, colors):
  # extract the bounding box (x, y)-coordinates for the AprilTag
  # # and convert each of the (x, y)-coordinate pairs to integers
    (ptA, ptB, ptC, ptD) = corners
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))
    # draw the bounding box of the AprilTag detection
    cv2.line(frame, ptA, ptB, colors, 2)
    cv2.line(frame, ptB, ptC, colors, 2)
    cv2.line(frame, ptC, ptD, colors, 2)
    cv2.line(frame, ptD, ptA, colors, 2)
    # draw the center (x, y)-coordinates of the AprilTag
    (cX, cY) = (int(center[0]), int(center[1]))
    cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


async def lidar_read():
    ser2 = serial.Serial("/dev/ttyUSB0",115200,8,'N',1,timeout=0.5)
    distance = 400
    freq = 0
    #await asyncio.sleep(0.05)
    while True:
        data2 = ser2.readline()  # read LiDAR stream
        data2 = data2.decode("utf-8") # turn from byte to string needed for USB
        #print(data2)
        freq = freq +1
        dist = (data2.split(" "))
        jarak = str(dist)
        jarak=(jarak[14:18])
        if(len(jarak)) == 4:
            try:
                jarak = (float(jarak))
                distance = jarak*100
                distance = int(distance)
                #print("Distance(cm): ", distance)
            except ValueError:
                print("\n!!!!!FLOAT ERROR!!!!!!\n")
                break

        if freq == 1:
            break

    return distance

async def manual_controls():
    """Main function to connect to the drone and input manual controls"""
    print("Application Started. Live view is enable? {}".format(isEnableDroneStream))
    drone = System()
    blink = 0

    if isEnableDroneControl:
        await drone.connect(system_address="serial:///dev/ttyACM0")
        
        # This waits till a mavlink based drone is connected
        print("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone with UUID: {state.uuid}")
                break

        info = await drone.info.get_version()
        print(info)


        conf = Config(default_height, follow_distance, direction, responsiveness)
        await drone.follow_me.set_config(conf)

    while True:
        blink = blink + 1
        ret, frame = cap.read()
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
        smallestTag = None
        smallestTagCenter = None
        print("blink: ", blink)


        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                                cameraMatrix=camera_matrix, distCoeff=camera_distortion)

        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                
                if isEnableDroneStream:
                    # draw the bounding box of the ArUCo detection
                    cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                if isEnableDroneStream:
                    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                    # draw the ArUco marker ID on the frame
                    cv2.putText(frame, str(markerID),
                        (topLeft[0], topLeft[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
                
                if smallestTag is None:
                    smallestTag = corners
                    smallestTagCenter = (cX, cY)
                else:
                    lengthSmallest = np.linalg.norm(smallestTag[1]-smallestTag[0])
                    lengthCurrent = np.linalg.norm(corners[1]-corners[0])
                    if lengthCurrent < lengthSmallest:
                        smallestTag = corners
                        smallestTagCenter =(cX, cY)
        
        if smallestTag is not None:
            (topLeft, topRight, bottomRight, bottomLeft) = smallestTag
            topLeft = (int(topLeft[0]), int(topLeft[1]))
 
            if isEnableDroneStream:
                cv2.putText(frame, str('smallest'),
                    (topLeft[0], topLeft[1]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 255), 2)

        roll = 0
        pitch = 0
        throttle = 0.5
        yaw = 0

        if isEnableDroneStream:
            cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  

        if smallestTag is not None:
            await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
            print("--------------**Target Detected!!**------------------")
 
            height, width, channels = frame.shape
            pitch, roll = imageUtil.getMatrix(frame,width, height, int(smallestTagCenter[0]), int(smallestTagCenter[1]))

            (topLeft, topRight, bottomRight, bottomLeft) = smallestTag
            tagWidth = np.linalg.norm(topLeft[0]-topRight[0])

            cv2.putText(frame, "PITCH {}".format(pitch), (300, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, "ROLL {}".format(roll), (300, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, "THROTLE {}".format(throttle), (300, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, "YAW {}".format(yaw), (300, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.putText(frame, "WIDTH {}".format(tagWidth), (300, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            #print("-- Starting offboard")
            try:
                await drone.offboard.start()
            except OffboardError as error:
                print(f"Starting offboard mode failed with error code: \
                        {error._result.result}")
                print("-- Program Terminate and land --")
                await drone.action.land()
                return
                
            if isEnableDroneControl:
                jarak = await lidar_read()
                print("Distance(cm): ", jarak)
                if jarak >1:       #original 99
                    guling = roll*30  #ori 30
                    junam = pitch*-30 
                    print("MARKER ID: ", markerID)
                    await drone.offboard.set_attitude(Attitude(guling, junam, yaw, 0.5)) #ori 0.5
                    await asyncio.sleep(0.08)    #ori 0.4 - ori 0.1
                    #await drone.offboard.set_attitude(Attitude(-guling, -junam, yaw, 0.51))  #ori 0.52
                    #asyncio.sleep(0.5)    #ori 0.1
                    #time.sleep(0.1)
                    print("Height: ", jarak, "ROLL: ", guling, "PITCH: ", junam)
                    blink = 0
                    #continue
                else:
                    print("INSTRUCTION LAND")
                    await drone.action.land()
                    await asyncio.sleep(7)
            
        #print("-- Stopping offboard")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed with error code: \
                        {error._result.result}")

        if blink > 3:
            if smallestTag is None:
                print("GPS MODE")
                conf = Config(default_height, follow_distance, direction, responsiveness)
                await drone.follow_me.set_config(conf)
                await drone.follow_me.start()
                await asyncio.sleep(0.5)
                blink = 0
        
        elif blink != 0:
            print("~~Hovering~~")
            await drone.offboard.set_attitude(Attitude(0.0, 0.0, yaw, 0.5)) #ori 0.5
            await asyncio.sleep(0.08)
                
            
        '''if isEnableDroneStream:
            cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break'''    



imageUtil = ImageUtil()
#control = KeyboardCtrl()

#--- Define Tag
id_to_find  = 72
marker_size  = 10 #- [cm]

#--- Get the camera calibration path
calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters  = aruco.DetectorParameters_create()

#--- Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

# keyboard = Controller()

isEnableDroneControl = True
isEnableDroneStream = True

if __name__ == '__main__':

    loop = asyncio.get_event_loop()
    loop.run_until_complete(manual_controls())
