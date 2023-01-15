# coefficients = [8.8698, -0.0518, 0.0001]
#
#
# def get_distance(reading):
#     if reading > 1600:
#         reading = 1600
#     if reading < 67:
#         reading = 67
#
#     val = 0
#     for i in range(0, 3):
#         val += coefficients[i] * pow(reading, i)
#     return val
#
#
# KP = 1
# KD = 0
# KI = 0
#
# last_error = 0
# total_error = 0
#
#
# def get_correction(dis, base_speed, direction):
#     global KP, KD, KI, last_error, total_error
#
#     error = dis - 2
#
#     total_error += error
#
#     p = KP * error
#     i = KI * total_error
#     d = KD * (error - last_error)
#
#     last_error = error
#
#     correction = p + i + d
#     speed1 = cap_values_to_range(base_speed + correction, 0, 6.2)
#     speed2 = cap_values_to_range(base_speed - correction, 0, 6.2)
#
#     if direction == "left":
#         return speed1, speed2
#     else:
#         return speed2, speed1
#
#
# def cap_values_to_range(val, minimum, maximum):
#     val = min(val, maximum)
#     val = max(val, minimum)
#     return val
