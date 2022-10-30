# !/usr/bin/env python
# coding: utf-8
import cv2 as cv
import numpy as np
from cv2 import aruco
import PID
import Arm_Lib

class ArUcoTracker:
    def __init__(self):
        self.Arm = Arm_Lib.Arm_Device()
        self.target_servox=90
        self.target_servoy=45
        # profile 0x: (0.5, 0.2, 0.31)  Original PID x values
        # profile 0y: (0.5, 0.2, 0.35)  Original PID y values
	# profile 1xy: (0.75, 0.1, 0.3) smooth but I need more fast
	# profile 2xy: (0.7, 0.1, 0.5)  better if it can go a bit fast but this ok gor now
        self.xservo_pid = PID.PositionalPID(0.7, 0.1, 0.5)
        self.yservo_pid = PID.PositionalPID(0.7, 0.1, 0.5)
        self.r = ()

    def detectMarker(self, img, markerSize=4, totalMarkers=50,draw=False):
        imGray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        customDict = aruco.Dictionary_create(nMarkers=7,markerSize=3) # nMarkers=100 works well with the PSvita ArUco cards
        arucoParam = aruco.DetectorParameters_create()
        b_boxes, ids, _ = aruco.detectMarkers(imGray, customDict, parameters=arucoParam)
        if draw: aruco.drawDetectedMarkers(img,b_boxes)
        arr = np.array(b_boxes) # convert the tuple into an numpy array
        return [arr, ids]

    def follow(self, img):
        x=320
        y=240
        corners, _ = self.detectMarker(img,draw=False)  # Detect ArUco marker corner=(1,1,4,2) for one marker
        if corners.any():
            # cv.polylines(img, np.int0(corners), True, (0,255,0),2)
            p0 = np.squeeze(corners) # remove single dimensional entries (1,1,4,2) -> (4,2)
            if p0.shape == (4,2): 
                center,_= cv.minEnclosingCircle(p0) # disregard multiple instances 
                # r = cv.boundingRect(p0) # output=(x0,y0,w,h)
                x = int(center[0])
                y = int(center[1])
                print('Tracking: {}, {}'.format(x,y), flush=True)
                cv.putText(img, "Tracking", (150,20), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv.circle(img,(x,y),20,(0,255,0),-1)
            # offset = 15
            # self.r = (r[0]-offset, r[1]-offset, r[2]+offset*2, r[3]+offset*2) # offsetting the boundary
            # cv.rectangle(img,(self.r[0],self.r[1]),(self.r[0]+self.r[2],self.r[1]+self.r[3]),(0,255,0),2)

        point_x = x
        point_y = y
        if not (self.target_servox>=180 and point_x<=320 or self.target_servox<=0 and point_x>=320):
            self.xservo_pid.SystemOutput = point_x
            self.xservo_pid.SetStepSignal(320)
            self.xservo_pid.SetInertiaTime(0.01, 0.1)
            target_valuex = int(1500 + self.xservo_pid.SystemOutput)
            self.target_servox = int((target_valuex - 500) / 10)

            if self.target_servox > 180:self.target_servox = 180
            if self.target_servox < 0: self.target_servox = 0
        if not (self.target_servoy>=180 and point_y<=240 or self.target_servoy<=0 and point_y>=240):

            self.yservo_pid.SystemOutput = point_y
            self.yservo_pid.SetStepSignal(240)
            self.yservo_pid.SetInertiaTime(0.01, 0.1)
            target_valuey = int(1500 + self.yservo_pid.SystemOutput)
            self.target_servoy = int((target_valuey - 500) / 10) - 45

            if self.target_servoy > 360: self.target_servoy = 360
            if self.target_servoy < 0: self.target_servoy = 0
        joints_0 = [self.target_servox, 135, self.target_servoy / 2, self.target_servoy / 2, 90, 30]
        self.Arm.Arm_serial_servo_write6_array(joints_0, 300)
        return img
