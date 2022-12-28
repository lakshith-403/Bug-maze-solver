ps = []
sensor_values = []

SENSOR_THRESHOLD = 300


def init_sensors(robot, timestep):
    """
        initialize proximity sensors to use by the robot

        :param robot: E Puck robot instance
        :param timestep: Time step of the simulation
    """

    global ps
    global sensor_values
    ps = []
    ps_names = [
        'ps0', 'ps1', 'ps2', 'ps3',
        'ps4', 'ps5', 'ps6', 'ps7'
    ]
    sensor_values = []
    for i in range(8):
        ps.append(robot.getDevice(ps_names[i]))
        ps[i].enable(timestep)


def update_sensor_readings(should_print: bool):
    """
    :param should_print: should print the sensor values

    updates the global variable 'sensor_values' with sensor readings
    300 > is wall exists
    """
    global sensor_values
    sensor_values = []
    for i in range(8):
        sensor_values.append(ps[i].getValue())
        if should_print:
            print("%.1f" % sensor_values[i], end=" ")
    if should_print:
        print()


def get_wall_array():
    """
    Check and return wall status in the current position of the robot.

    :return: Boolean array of length 3. Do walls exist on [left, front, right]
    """
    wall_array = [False, False, False]
    if sensor_values[5] >= SENSOR_THRESHOLD:
        wall_array[0] = True
    if sensor_values[0] >= SENSOR_THRESHOLD or sensor_values[7] >= SENSOR_THRESHOLD:
        wall_array[1] = True
    if sensor_values[2] >= SENSOR_THRESHOLD:
        wall_array[0] = True

    return wall_array
