from controller import Camera, Robot
import cv2
import numpy as np

cam: Camera


def init_cam(robot: Robot, timestep):
    """
    Initialize the camera

    :param robot:
    :param timestep:
    """
    global cam
    cam = robot.getDevice("camera")
    cam.enable(timestep)


def can_see_ball():
    """
    filter the image from camera to find yellow

    :return: True If there is enough yellow on image else False
    """
    global cam
    img = cam.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    img = cv2.flip(img, 1)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define range wanted color in HSV
    lower_val = np.array([20, 0, 0])
    upper_val = np.array([25, 250, 250])

    # Threshold the HSV image
    mask = cv2.inRange(hsv, lower_val, upper_val)

    has_yellow = np.sum(mask) > 50000

    if has_yellow:
        return True
    else:
        return False
