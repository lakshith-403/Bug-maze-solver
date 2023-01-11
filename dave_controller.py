

from proximity_sensor import *
from motors import *
from cam import *
import time

robot = Robot()
timestep = 2  #int(robot.getBasicTimeStep())
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

target_angle = 0

current_angle = 0
best_index = 0

position_record = (time.time(), [0, 0])

wall_follow_direction = "left"

got_money = False
turning_to_goal = False

dollars = 0


def reset_data():
    global state, target_x, target_y, target_angle, best_index, got_money, turning_to_goal
    state = "heading_target"  # "heading_target"
    target_x = 100
    target_y = 100
    target_angle = 0
    best_index = 0
    got_money = False
    turning_to_goal = False


def is_stuck(x, y):
    global position_record

    time_now = time.time()
    if get_distance(x, y, position_record[1][0], position_record[1][1]) > 0.07:
        position_record = (time_now, [x, y])
    else:
        if time_now - position_record[0] > 2:
            return True
    return False


def get_distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))


def angle_abs(angle1, angle2):
    diff = (angle2 - angle1 + 180) % 360 - 180
    return diff + 360 if diff < -180 else diff


def get_angle_delta():
    global target_y, target_x, current_y, current_x, current_angle, target_angle
    target_angle = math.degrees(math.atan2(target_y - current_y, target_x - current_x))

    if target_angle < 0:
        target_angle = 360 + target_angle

    return angle_abs(target_angle, current_angle)


while robot.step(timestep) != -1:
    while receiver.getQueueLength() > 0:
        data = json.loads(receiver.getData().decode('utf-8'))

        if data["dollars"] != dollars:
            reset_data()
            dollars = data["dollars"]

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

            if not got_money:
                state = "heading_target"
                got_money = True
                turning_to_goal = True
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
        go_back()
        state = "heading_target"
        continue

    if can_see_ball():
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
        if readings['front'] and not turning_to_goal:
            state = "wall_following"
            if angle_delta > 0:
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

        if abs(angle_delta) < 10 and not readings['front']:
            state = "heading_target"
