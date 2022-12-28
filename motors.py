import math
from typing import Optional
from controller import Motor, PositionSensor, Robot, Receiver
import json

robot: Optional[Robot] = None

left: Optional[Motor] = None
right: Optional[Motor] = None

left_sensor: Optional[PositionSensor] = None
right_sensor: Optional[PositionSensor] = None

MAX_SPEED = 6.28
ERROR_DELTA = 0.05

left_pos = 0
right_pos = 0

timestep = 0


def init_motors(bot, _timestep):
    """
    initialize left and right motors for velocity control (sets position to inf)

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


def move_forward():
    left.setVelocity(MAX_SPEED)
    right.setVelocity(MAX_SPEED)

    current_left_pos = left_sensor.getValue()
    current_right_pos = right_sensor.getValue()

    print(current_right_pos)

    target_left_pos = current_left_pos + 15
    target_right_pos = current_right_pos + 15

    left.setPosition(target_left_pos)
    right.setPosition(target_right_pos)

    while robot.step(timestep) != -1:
        current_left_pos = left_sensor.getValue()
        current_right_pos = right_sensor.getValue()
        if abs(current_right_pos - target_right_pos) <= ERROR_DELTA and \
                abs(current_left_pos - target_left_pos) <= ERROR_DELTA:
            break

    return


def set_motors_to_forward():
    left.setPosition(float("inf"))
    right.setPosition(float("inf"))
    left.setVelocity(MAX_SPEED)
    right.setVelocity(MAX_SPEED)


def get_angle(receiver: Receiver):
    while receiver.getQueueLength() > 0:
        data = json.loads(receiver.getData().decode('utf-8'))['robotAngleDegrees']
        print(data)
        receiver.nextPacket()
        return data


def stop_bot(milliseconds):
    left.setVelocity(0)
    right.setVelocity(0)
    left.setPosition(float("inf"))
    right.setPosition(float("inf"))
    for _ in range(0, milliseconds, timestep):
        robot.step(timestep)


def turn(direction):
    stop_bot(500)

    left.setVelocity(MAX_SPEED / 4.0)
    right.setVelocity(MAX_SPEED / 4.0)

    current_left_pos = left_sensor.getValue()
    current_right_pos = right_sensor.getValue()

    # print(current_right_pos)

    target_left_pos = current_left_pos - 2.25 * (-1 if direction == "r" else 1)
    target_right_pos = current_right_pos + 2.25 * (-1 if direction == "r" else 1)

    left.setPosition(target_left_pos)
    right.setPosition(target_right_pos)

    while robot.step(timestep) != -1:
        current_left_pos = left_sensor.getValue()
        current_right_pos = right_sensor.getValue()
        # print(current_left_pos, current_right_pos)
        if abs(current_right_pos - target_right_pos) <= ERROR_DELTA and \
                abs(current_left_pos - target_left_pos) <= ERROR_DELTA:
            break

    stop_bot(500)

    return


def forward_bit():
    stop_bot(300)

    left.setVelocity(MAX_SPEED)
    right.setVelocity(MAX_SPEED)

    current_left_pos = left_sensor.getValue()
    current_right_pos = right_sensor.getValue()

    # print(current_right_pos)

    target_left_pos = current_left_pos + 3
    target_right_pos = current_right_pos + 3

    left.setPosition(target_left_pos)
    right.setPosition(target_right_pos)

    while robot.step(timestep) != -1:
        current_left_pos = left_sensor.getValue()
        current_right_pos = right_sensor.getValue()
        # print(current_left_pos, current_right_pos)
        if abs(current_right_pos - target_right_pos) <= ERROR_DELTA and \
                abs(current_left_pos - target_left_pos) <= ERROR_DELTA:
            break

    stop_bot(300)

    return
