from controller import Robot
import json
from proximity_sensor import init_sensors, update_sensor_readings, get_wall_array
from motors import *
import random

robot = Robot()
timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
receiver.enable(10)

# left.setVelocity(-6.28)
# right.setVelocity(-6.28)


init_sensors(robot, timestep)
init_motors(robot, timestep)

robot.step(timestep)

didFindWall = False
move_forward()
turn(direction='r')

while robot.step(timestep) != -1:
    while receiver.getQueueLength() > 0:
        data = json.loads(receiver.getData().decode('utf-8'))
        # print(data['robotAngleDegrees'])
        receiver.nextPacket()
    set_motors_to_forward()
    update_sensor_readings(should_print=False)
    walls = get_wall_array()
    if didFindWall:
        if not walls[0]:
            forward_bit()
            turn(direction='l')
            forward_bit()
        elif not walls[1]:
            pass
        elif not walls[2]:
            forward_bit()
            turn(direction='r')
            forward_bit()
        else:
            turn(direction='l')
    else:
        if walls[1]:
            didFindWall = True
            if not walls[0]:
                turn(direction='r')
            else:
                turn(direction='l')

    # # print(walls)
    # print(sensor.getValue())

    pass

'''
time: float
collectibles: [tuple<float, float>]
rupees: Int
dollars: float
goals: [tuple<float, float>]
robot: [float, float]
robotAngleDegrees: float
'''

'''
Do the bug zero for most middle ball

'''

'''
import os
import math
import random
from controller import Robot, DistanceSensor, Motor

# Set the target destination coordinates
TARGET_X = 5.0
TARGET_Y = 5.0

# Get the robot and its components
robot = Robot()
left_sensor = robot.getDistanceSensor('left_sensor')
right_sensor = robot.getDistanceSensor('right_sensor')
left_motor = robot.getMotor('left_motor')
right_motor = robot.getMotor('right_motor')

# Enable the distance sensors
left_sensor.enable(32)
right_sensor.enable(32)

# Set the initial speeds of the motors
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.5)
right_motor.setVelocity(0.5)

# Run the robot's main loop
while robot.step(32) != -1:
    # Read the distance sensor values
    left_distance = left_sensor.getValue()
    right_distance = right_sensor.getValue()

    # Get the current position of the robot
    position = robot.getPosition()
    current_x = position[0]
    current_y = position[1]
    
    # Calculate the angle to the target destination
    target_angle = math.atan2(TARGET_Y - current_y, TARGET_X - current_x)
    
    # Calculate the difference between the current heading and the target angle
    heading_difference = target_angle - position[2]
    
    # If there is a wall on the left, turn right
    if left_distance < 0.5:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0.5)
    # If there is a wall on the right, turn left
    elif right_distance < 0.5:
        left_motor.setVelocity(0.5)
        right_motor.setVelocity(0)
    # If there are no walls, adjust the heading to move towards the target
    else:
        # If the heading difference is small, continue straight
        if abs(heading_difference) < 0.1:
            left_motor.setVelocity(0.5)
            right_motor.setVelocity(0.5)
        # If the heading difference is large, turn towards the target
        else:
            # If the heading difference is positive, turn right
            if heading_difference > 0:
                left_motor.setVelocity(0.5)
                right_motor.setVelocity(0)
            # If the heading difference is negative, turn left
            else:
                left_motor.setVelocity(0)
                right_motor.setVelocity(0.5)

'''