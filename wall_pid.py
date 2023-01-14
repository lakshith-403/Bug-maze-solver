KP = 0.56977
KD = 0.3
KI = 0

last_error = 0
total_error = 0


def get_correction(angle_delta, base_speed):
    global KP, KD, KI, last_error, total_error
    total_error += angle_delta

    p = KP * angle_delta
    i = KI * total_error
    d = KD * (angle_delta - last_error)

    last_error = angle_delta

    correction = p + i + d
    left_speed = cap_values_to_range(base_speed + correction, -6.2, 6.2)
    right_speed = cap_values_to_range(base_speed - correction, -6.2, 6.2)
    return left_speed, right_speed


def cap_values_to_range(val, minimum, maximum):
    val = min(val, maximum)
    val = max(val, minimum)
    return val
