# Project: ArUco Tracker
# Description: Robot arm movement system with ArUco marker with IR or Optical
# Code By: Anuradha Gunawardhana
# Version: 0.2
# Date: 17.09.2022

import cv2 as cv
from tracker import ArUcoTracker

tracker = ArUcoTracker()
#--------------------------------------- Select Cameras -----------------------------------------------#
#cam_ID = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0'  # optical camera
# cam_ID = '/dev/v4l/by-id/usb-Generic_USB2.0_PC_CAMERA-video-index0'  # IR Camera
cam_ID = '/dev/v4l/by-id/usb-Microsoft_Microsoft®_LifeCam_HD-5000-video-index0' # Microsoft camera
#------------------------------------------------------------------------------------------------------#

def main():
    tracker.Arm.Arm_serial_set_torque(1) # torque setting
    #joints_0 = [90, 150, 20, 20, 90, 30]
    joints_0 = [90, 90, 0, 0, 0, 0] # for 2 axis
    tracker.Arm.Arm_serial_servo_write6_array(joints_0, 1500) # move arm to the initial resting position

    cam = cv.VideoCapture(cam_ID)
    cam.set(cv.CAP_PROP_AUTO_EXPOSURE, 3) # Need to set auto exposure level to 3 and ten back to 1 to set an absolute value
    cam.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)
    cam.set(cv.CAP_PROP_EXPOSURE, 5)   # minimize exposure to avoid detection failing due to bright overhead light
    cam.set(cv.CAP_PROP_AUTOFOCUS, 0) # Turn off autofocus on Microsoft camera
    while True:
        timer = cv.getTickCount()
        _, frame = cam.read()
        frame = cv.resize(frame, (640,480))
        # frame = cv.rotate(frame, cv.ROTATE_180)  # rotate the camera
        frame = tracker.follow(frame)
        fps = cv.getTickFrequency() / (cv.getTickCount() - timer)    
        # print('fps={}'.format(int(fps),end='\r'))
        cv.putText(frame, "FPS: {:02n}".format(int(fps)), (10,20), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        #cv.imshow('Tracking', frame)  # Uncomment when using with a monitor
        if cv.waitKey(1) == ord('q'):
            cam.release()
            break
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
