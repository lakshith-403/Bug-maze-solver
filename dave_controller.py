import random

from proximity_sensor import *
from motors import *
from cam import *
import time

robot = Robot()
timestep = 2
receiver = robot.getDevice("receiver")
receiver.enable(10)

robot.step(timestep)
init_motors(robot, timestep)
init_sensors(robot, timestep)
init_cam(robot, timestep)

MAX_SPEED = 6.2

state = "heading_target"

current_x = 0
current_y = 0

target_x = 100
target_y = 100

target_angle = 0

current_angle = 0
best_index = 0

position_record = (time.time(), [0, 0])

wall_follow_direction = "left"

got_money = False
turning_to_goal = False

dollars = 0


def reset_data():
    """
    reset global variables and make the robot chase balls after reaching the target
    """
    global state, target_x, target_y, target_angle, best_index, got_money, turning_to_goal
    state = "heading_target"
    target_x = 100
    target_y = 100
    target_angle = 0
    best_index = 0
    got_money = False
    turning_to_goal = False


def is_stuck(x, y):
    """
    :param x: robot's x coordinate
    :param y: robot's y coordinate
    :return: If the robot is stuck in the same place for two seconds
    """
    global position_record

    time_now = time.time()
    if get_distance(x, y, position_record[1][0], position_record[1][1]) > 0.01:
        position_record = (time_now, [x, y])
    else:
        if time_now - position_record[0] > 2:
            return True
    return False


def get_distance(x1, y1, x2, y2):
    """
    Find the euclidean distance between two points
    :param x1: coordinate 1 x
    :param y1: coordinate 1 y
    :param x2: coordinate 2 x
    :param y2: coordinate 2 y
    :return: distance
    """
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))


def angle_dif(angle1, angle2):
    """
    Difference between two angles
    :param angle1: a1 in degrees
    :param angle2: a2 in degrees

    :return: difference in degrees
    """
    diff = (angle2 - angle1 + 180) % 360 - 180
    return diff + 360 if diff < -180 else diff


def get_angle_delta():
    """
    Difference of angle between the two lines y=0 and current (pos to target pos)

    :return: angle in degrees (+-)
    """
    global target_y, target_x, current_y, current_x, current_angle, target_angle
    target_angle = math.degrees(math.atan2(target_y - current_y, target_x - current_x))

    if target_angle < 0:
        target_angle = 360 + target_angle

    return angle_dif(target_angle, current_angle)


while robot.step(timestep) != -1:
    while receiver.getQueueLength() > 0:
        data = json.loads(receiver.getData().decode('utf-8'))

        if data["dollars"] != dollars:  # reached a target with rupees, reset data to chase the next ball
            reset_data()
            dollars = data["dollars"]

        current_x = data["robot"][0]
        current_y = data["robot"][1]

        current_angle = data["robotAngleDegrees"]

        if data["rupees"] != 0:  # if got rupees then target chasing mode
            goals = data["goals"]
            target_x = 1000
            target_y = 1000
            for goal in goals:  # find the nearest target
                if get_distance(current_x, current_y, goal[0], goal[1]) < get_distance(current_x, current_y, target_x,
                                                                                       target_y):
                    target_x = goal[0]
                    target_y = goal[1]

            if not got_money:  # when get rupees for the first time, turn to target irrespective of walls
                state = "heading_target"
                got_money = True
                turning_to_goal = True
        else:  # ball chasing mode
            balls = data["collectibles"]
            for ball in balls:  # find the nearest ball
                if get_distance(current_x, current_y, ball[0], ball[1]) < get_distance(current_x, current_y, target_x,
                                                                                       target_y):
                    target_x = ball[0]
                    target_y = ball[1]
        receiver.nextPacket()

    stuck = is_stuck(current_x, current_y)

    if stuck:
        # print("stuck")
        go_back()
        state = "heading_target"
        continue

    if can_see_ball():
        print("can_see_ball")
        set_velocity(MAX_SPEED, MAX_SPEED)
        continue

    update_sensor_readings(should_print=False)
    readings = get_sensor_readings(direction=wall_follow_direction)

    angle_delta = get_angle_delta()

    if state == "heading_target":
        if abs(angle_delta) < 10:
            print("heading_target")
            set_velocity(MAX_SPEED, MAX_SPEED)
            turning_to_goal = False
        else:  # fix the angle
            if angle_delta > 0:
                set_velocity(MAX_SPEED, -MAX_SPEED)
            elif angle_delta < 0:
                set_velocity(-MAX_SPEED, MAX_SPEED)
        if (readings['front'] or readings['close_left_corner'] or readings['close_right_corner']) and not turning_to_goal :  # found wall -> follow that
            state = "wall_following"
            if angle_delta > 0:  # randomly decide the turning direction
                wall_follow_direction = "right"
            else:
                wall_follow_direction = "left"
            set_velocity(0, 0)

    elif state == "wall_following":
        print("wall_following")
        if wall_follow_direction == "left":
            if readings['front']:
                set_velocity(MAX_SPEED, -MAX_SPEED)
            else:
                if readings['left']:
                    set_velocity(MAX_SPEED, MAX_SPEED)
                else:
                    set_velocity(MAX_SPEED / 8, MAX_SPEED)

            if readings['left_corner']:
                set_velocity(MAX_SPEED, MAX_SPEED / 8)
        elif wall_follow_direction == "right":
            if readings['front']:
                set_velocity(-MAX_SPEED, MAX_SPEED)
            else:
                if readings['right']:
                    set_velocity(MAX_SPEED, MAX_SPEED)
                else:
                    set_velocity(MAX_SPEED, MAX_SPEED / 8)

            if readings['right_corner']:
                set_velocity(MAX_SPEED/8, MAX_SPEED)

        if abs(angle_delta) < 10 and not readings['front']:  # heading the target, so switch to that mode
            state = "heading_target"
