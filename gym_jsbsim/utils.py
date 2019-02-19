import functools
import operator
from typing import Tuple
from gym_jsbsim.aircraft import cessna172P, a320, f15
from typing import Dict, Iterable


class AttributeFormatter(object):
    """
    Replaces characters that would be illegal in an attribute name

    Used through its static method, translate()
    """
    ILLEGAL_CHARS = '\-/.'
    TRANSLATE_TO = '_' * len(ILLEGAL_CHARS)
    TRANSLATION_TABLE = str.maketrans(ILLEGAL_CHARS, TRANSLATE_TO)

    @staticmethod
    def translate(string: str):
        return string.translate(AttributeFormatter.TRANSLATION_TABLE)


def get_env_id(task_type, aircraft, enable_flightgear) -> str:
    """
    Creates an env ID from the environment's components

    :param task_type: Task class, the environment's task
    :param aircraft: Aircraft namedtuple, the aircraft to be flown
    :param enable_flightgear: True if FlightGear simulator is enabled for visualisation else False
     """
    if enable_flightgear:
        fg_setting = 'FG'
    else:
        fg_setting = 'NoFG'
    return f'JSBSim-{task_type.__name__}-{aircraft.name}-{fg_setting}-v0'


def get_env_id_kwargs_map() -> Dict[str, Tuple]:
    """ Returns all environment IDs mapped to tuple of (task, aircraft, flightgear) """
    # lazy import to avoid circular dependencies
    from gym_jsbsim.heading_control_task import HeadingControlTask, TurnHeadingChangeLevelControlTask, HeadingControlTask_1Bis

    map = {}
    for task_type in (HeadingControlTask, TurnHeadingChangeLevelControlTask,HeadingControlTask_1Bis):
        for plane in (cessna172P, a320, f15):
            for enable_flightgear in (True, False):
                id = get_env_id(task_type, plane, enable_flightgear)
                assert id not in map
                map[id] = (task_type, plane, enable_flightgear)
    return map


def product(iterable: Iterable):
    """
    Multiplies all elements of iterable and returns result

    ATTRIBUTION: code provided by Raymond Hettinger on SO
    https://stackoverflow.com/questions/595374/whats-the-function-like-sum-but-for-multiplication-product
    """
    return functools.reduce(operator.mul, iterable, 1)


def reduce_reflex_angle_deg(angle: float) -> float:
    """ Given an angle in degrees, normalises in [-179, 180] """
    # ATTRIBUTION: solution from James Polk on SO,
    # https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees#
    new_angle = angle % 360
    if new_angle > 180:
        new_angle -= 360
    return new_angle
