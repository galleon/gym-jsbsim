from collections import namedtuple
from gym.spaces import Box, Discrete


Property = namedtuple('Property', ['name_jsbsim', 'description', 'min', 'max', 'spaces'], defaults=(None, None, float('-inf'), float('+inf'), Box))
