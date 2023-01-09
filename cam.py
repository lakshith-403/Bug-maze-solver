from controller import Camera, Robot
from typing import Optional
import cv2
import numpy as np

cam: Camera


def init_cam(robot: Robot, timestep):
    global cam
    cam = robot.getDevice("camera")
    cam.enable(timestep)


def can_see_ball():
    global cam
    img = cam.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    img = cv2.flip(img, 1)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define range wanted color in HSV
    # lower_val = np.array([17, 19, 27])
    # upper_val = np.array([57, 239, 250])
    lower_val = np.array([20, 0, 0])
    upper_val = np.array([25, 250, 250])


    # Threshold the HSV image - any green color will show up as white
    mask = cv2.inRange(hsv, lower_val, upper_val)

    # if there are any white pixels on mask, sum will be > 0
    has_yellow = np.sum(mask)
    print(has_yellow)
    if has_yellow > 50000:
        return True
    else:
        return False
