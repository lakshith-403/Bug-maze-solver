from controller import Robot
import json
from proximity_sensor import *
from motors import *
import random

robot = Robot()
timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
receiver.enable(10)

robot.step(timestep)
init_motors(robot, timestep)
init_sensors(robot, timestep)

MAX_SPEED = 6.2


while robot.step(timestep) != -1:
    # while receiver.getQueueLength() > 0:
    #     data = json.loads(receiver.getData().decode('utf-8'))
    #     print(data)
    #     receiver.nextPacket()
    update_sensor_readings(should_print=False)
    readings = get_sensor_readings()

    if readings['front']:
        set_velocity(MAX_SPEED, -MAX_SPEED)
    else:
        if readings['left']:
            set_velocity(MAX_SPEED, MAX_SPEED)
        else:
            set_velocity(MAX_SPEED/8, MAX_SPEED)

    if readings['left_corner']:
        set_velocity(MAX_SPEED, MAX_SPEED/8)
