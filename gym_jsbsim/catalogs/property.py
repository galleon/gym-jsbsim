from collections import namedtuple
from gym.spaces import Box, Discrete


Property = namedtuple('Property', ['name_jsbsim', 'description', 'min', 'max', 'access', 'spaces', 'update'], defaults=(None, None, float('-inf'), float('+inf'),'RW', Box, None))
