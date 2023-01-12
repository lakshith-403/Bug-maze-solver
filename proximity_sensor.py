ps = []
sensor_values = []

SENSOR_THRESHOLD = 240


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
    """
    global sensor_values
    sensor_values = []
    for i in range(8):
        sensor_values.append(ps[i].getValue())
        if should_print:
            print("%.1f" % sensor_values[i], end=" ")
    if should_print:
        print()


def get_sensor_readings(direction):
    """
    Create a dict with keys ['left', 'right' , 'left_corner', 'right_corner', 'front'].
    True If wall exists else False.
    left and right distance thresholds are lower than front.
    from two front sensors, only one sensor is used depending on the wall follow direction.
    threshold for left, right will change depending on if there's a front wall

    :param direction: 'left' or 'right'
    :return: wall dictionary
    """
    dictionary = {
        'left': False,
        'right': False,
        'left_corner': False,
        'right_corner': False,
        'front': False
    }

    if direction == "left" and sensor_values[7] >= SENSOR_THRESHOLD / 3:
        dictionary['front'] = True
    if direction == "right" and sensor_values[0] >= SENSOR_THRESHOLD / 3:
        dictionary['front'] = True

    if sensor_values[5] >= (SENSOR_THRESHOLD if not dictionary['front'] else SENSOR_THRESHOLD / 3):
        dictionary['left'] = True
    if sensor_values[2] >= (SENSOR_THRESHOLD if not dictionary['front'] else SENSOR_THRESHOLD / 3):
        dictionary['right'] = True
    if sensor_values[6] >= SENSOR_THRESHOLD / 1.5:
        dictionary['left_corner'] = True
    if sensor_values[1] >= SENSOR_THRESHOLD / 1.5:
        dictionary['right_corner'] = True

    return dictionary
