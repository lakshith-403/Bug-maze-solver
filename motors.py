import time
from typing import Optional
from controller import Motor, PositionSensor, Robot

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

    # Velocity control
    left.setPosition(float("inf"))
    right.setPosition(float("inf"))
    left.setVelocity(0)
    right.setVelocity(0)


def set_velocity(left_velocity, right_velocity):
    """
        set velocities for motors (unit - rads-1 )
        negative values means rotating backwards

        :param left_velocity: Velocity for the left motor [-6.28, +6.28]
        :param right_velocity: Velocity for the right motor [-6.28, +6.28]
    """
    global left, right
    if left_velocity is not None:
        left.setVelocity(left_velocity)
    if right_velocity is not None:
        right.setVelocity(right_velocity)


def go_back():
    """
    Go back for 1 second with max speed
    """
    start_time = time.time()
    global robot
    while robot.step(timestep) != -1:
        set_velocity(-6, -6)
        if time.time() - start_time > 1:
            break
