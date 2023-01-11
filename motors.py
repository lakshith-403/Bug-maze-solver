import math
import time
from typing import Optional
from controller import Motor, PositionSensor, Robot, Receiver
import json

robot: Optional[Robot] = None

left: Optional[Motor] = None
right: Optional[Motor] = None

left_sensor: Optional[PositionSensor] = None
right_sensor: Optional[PositionSensor] = None

MAX_SPEED = 6.28
ERROR_DELTA = 0.01

left_pos = 0
right_pos = 0

timestep = 0


def init_motors(bot, _timestep):
    """
    initialize robot variables locally and setup motors for position control.

    :param _timestep: time step of the simulation
    :param bot: E Puck robot instance
    """
    global left, right
    global left_sensor, right_sensor
    global robot
    global timestep

    timestep = _timestep
    robot = bot

    left = robot.getDevice('left wheel motor')
    right = robot.getDevice('right wheel motor')

    left_sensor = robot.getDevice('left wheel sensor')
    right_sensor = robot.getDevice('right wheel sensor')
    left_sensor.enable(timestep)
    right_sensor.enable(timestep)

    # for Velocity control
    # left.setPosition(float("inf"))
    # right.setPosition(float("inf"))
    # left.setVelocity(0)
    # right.setVelocity(0)

    # for Position control
    left.setPosition(0)
    right.setPosition(0)
    left.setVelocity(MAX_SPEED)
    right.setVelocity(MAX_SPEED)


def set_position_control():
    """
        set motors for position control
    """
    global left, right, MAX_SPEED
    left.setVelocity(MAX_SPEED)
    right.setVelocity(MAX_SPEED)


def set_velocity(left_velocity, right_velocity):
    """
        set velocities for motors (unit - rads-1 )
        negative values means rotating backwards

        :param left_velocity: Velocity for the left motor [-6.28, +6.28]
        :param right_velocity: Velocity for the right motor [-6.28, +6.28]
    """
    global left, right
    left.setPosition(float("inf"))
    right.setPosition(float("inf"))
    if left_velocity is not None:
        left.setVelocity(left_velocity)
    if right_velocity is not None:
        right.setVelocity(right_velocity)


def get_position_delta(step):
    """
        find how much should the motor position change for the robot to move a step

        :param step: step distance in centimeters
        :return: motor position delta required for a step
    """
    return step / math.pi * 2.05


def turn(direction):
    """
        Turn the robot by 90% left or right

        :param direction: right = 1, left = -1
    """
    global left, right, ERROR_DELTA
    delta = get_position_delta(math.pi * 1.2)
    set_position(delta * direction, -1 * delta * direction)


def move_forward(step):
    set_position(get_position_delta(step), get_position_delta(step))
    return


def go_back():
    start_time = time.time()
    global robot
    while robot.step(timestep) != -1:
        set_velocity(-6, -6)
        if time.time() - start_time > 1:
            break


def set_position(left_delta, right_delta):
    global left, right, ERROR_DELTA
    set_position_control()

    current_left_pos = left_sensor.getValue()
    current_right_pos = right_sensor.getValue()

    target_left_pos = current_left_pos + left_delta
    target_right_pos = current_right_pos + right_delta

    left.setPosition(target_left_pos)
    right.setPosition(target_right_pos)

    while robot.step(timestep) != -1:
        current_left_pos = left_sensor.getValue()
        current_right_pos = right_sensor.getValue()
        if abs(current_right_pos - target_right_pos) <= ERROR_DELTA and \
                abs(current_left_pos - target_left_pos) <= ERROR_DELTA:
            break

    return
