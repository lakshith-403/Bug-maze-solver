from typing import Tuple, List
import math

choices_history: List[Tuple] = []  # (x, y, choice)


def get_distance(x1, y1, x2, y2):
    """
    Find the euclidean distance between two points
    :param x1: coordinate 1 x
    :param y1: coordinate 1 y
    :param x2: coordinate 2 x
    :param y2: coordinate 2 y
    :return: distance
    """
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))


def pick_direction(angle_delta: float, position: Tuple['float']) -> str:
    global choices_history
    current_choice: str
    if angle_delta > 0:  # decide the turning direction from orientation respect to target
        current_choice = "right"
    else:
        current_choice = "left"

    prev_choice = ""
    for choice in choices_history:
        if get_distance(choice[0], choice[1], position[0], position[1]) < 0.2:  # met this place earlier
            prev_choice = choice[2]
            break

    new_choice = current_choice
    if prev_choice == current_choice:  # about to go in a loop
        new_choice = "left" if current_choice == "right" else "right"

    choices_history.append((position[0], position[1], new_choice))

    return new_choice


def clear_history():
    global choices_history
    choices_history = []
