from collections import namedtuple
from gym.spaces import Box, Discrete


Property = namedtuple('Property', ['name_jsbsim', 'description', 'min', 'max', 'access', 'spaces', 'update'])

Property.__new__.__defaults__ = ( None, float('-inf'), float('+inf'),'RW', Box, None)
