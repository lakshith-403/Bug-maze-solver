from controller import Robot
import json
from proximity_sensor import init_sensors, update_sensor_readings, get_wall_array
from motors import *
import random

robot = Robot()
timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
receiver.enable(10)

robot.step(timestep)
init_motors(robot, timestep)
init_sensors(robot, timestep)

bot_step = 1  # in cm

while robot.step(timestep) != -1:
    # while receiver.getQueueLength() > 0:
    #     data = json.loads(receiver.getData().decode('utf-8'))
    #     print(data)
    #     receiver.nextPacket()
    update_sensor_readings(should_print=False)
    walls = get_wall_array()
    print(walls)
    if not walls[1]:
        set_velocity(5, 5)
    else:
        set_velocity(-2, 5)
    pass
