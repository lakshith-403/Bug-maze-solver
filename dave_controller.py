'''
todo check if the bot is stuck in the same place -> go backward and rotate till it faces the target
todo follow wall on left or right depending on the target position
'''

from proximity_sensor import *
from motors import *
from cam import *
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
receiver.enable(10)

robot.step(timestep)
init_motors(robot, timestep)
init_sensors(robot, timestep)
init_cam(robot, timestep)

MAX_SPEED = 6.2

state = "heading_target"  # "heading_target"

current_x = 0
current_y = 0

target_x = 100
target_y = 100

current_angle = 0
best_index = 0

position_record = (time.time(), [0, 0])


def is_stuck(x, y):
    global position_record

    time_now = time.time()
    if get_distance(x, y, position_record[1][0], position_record[1][1]) > 0.08:
        position_record = (time_now, [x, y])
    else:
        if time_now - position_record[0] > 5:
            return True
    return False


def get_distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))


def angle_abs(angle1, angle2):
    diff = (angle2 - angle1 + 180) % 360 - 180
    return diff + 360 if diff < -180 else diff


def get_angle_delta():
    global target_y, target_x, current_y, current_x, current_angle
    target = math.degrees(math.atan2(target_y - current_y, target_x - current_x))

    if target < 0:
        target = 360 - target
    # target -= 90

    print(f"{target} , {current_angle}")
    return angle_abs(target, current_angle)


while robot.step(timestep) != -1:
    while receiver.getQueueLength() > 0:
        data = json.loads(receiver.getData().decode('utf-8'))

        current_x = data["robot"][0]
        current_y = data["robot"][1]

        current_angle = data["robotAngleDegrees"]

        if data["rupees"] != 0:
            goals = data["goals"]
            target_x = 1000
            target_y = 1000
            for goal in goals:
                if get_distance(current_x, current_y, goal[0], goal[1]) < get_distance(current_x, current_y, target_x,
                                                                                       target_y):
                    target_x = goal[0]
                    target_y = goal[1]
        else:
            balls = data["collectibles"]
            for ball in balls:
                if get_distance(current_x, current_y, ball[0], ball[1]) < get_distance(current_x, current_y, target_x,
                                                                                       target_y):
                    target_x = ball[0]
                    target_y = ball[1]
        receiver.nextPacket()

    stuck = is_stuck(current_x, current_y)

    if stuck:
        set_position(-5, -5)
        state = "heading_target"
        continue

    if can_see_ball():
        set_velocity(MAX_SPEED, MAX_SPEED)
        continue

    update_sensor_readings(should_print=False)
    readings = get_sensor_readings()

    angle_delta = get_angle_delta()
    print(f"{angle_delta} target: {target_x} {target_y}")

    if state == "heading_target":
        if abs(angle_delta) < 10:
            print("heading")
            set_velocity(MAX_SPEED, MAX_SPEED)
        else:  # fix the angle
            if angle_delta > 0:
                set_velocity(MAX_SPEED, -MAX_SPEED)
            elif angle_delta < 0:
                set_velocity(-MAX_SPEED, MAX_SPEED)
        if readings['front']:
            state = "wall_following"
            set_velocity(0, 0)

    elif state == "wall_following":
        if readings['front']:
            set_velocity(MAX_SPEED, -MAX_SPEED)
        else:
            if readings['left']:
                set_velocity(MAX_SPEED, MAX_SPEED)
            else:
                set_velocity(MAX_SPEED / 8, MAX_SPEED)

        if readings['left_corner']:
            set_velocity(MAX_SPEED, MAX_SPEED / 8)

        if abs(angle_delta) < 10:
            state = "heading_target"
