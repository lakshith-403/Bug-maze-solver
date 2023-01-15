from typing import Tuple, List, Optional
import math

choices_history: List[Tuple] = []  # (x, y, choice)
pending_choice: Optional[Tuple] = None


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


def notify_position(x, y):
    global pending_choice, choices_history
    if pending_choice is None:
        return
    if get_distance(x,y, pending_choice[0], pending_choice[1]) > 0.5:
        choices_history.append(pending_choice)
        pending_choice = None


def pick_direction(angle_delta: float, position: Tuple['float']) -> str:
    global choices_history, pending_choice
    current_choice: str
    if angle_delta > 0:  # decide the turning direction from orientation respect to target
        current_choice = "right"
    else:
        current_choice = "left"

    prev_choice = ""
    for choice in choices_history:
        if get_distance(choice[0], choice[1], position[0], position[1]) < 0.2:  # met this place earlier
            prev_choice = choice[2]
            print(f"found pos -> {prev_choice}")
            break

    new_choice = current_choice
    if prev_choice == current_choice:  # about to go in a loop
        new_choice = "left" if current_choice == "right" else "right"

    pending_choice = (position[0], position[1], new_choice)

    return new_choice


def clear_history():
    global choices_history
    choices_history = []
