from collections import namedtuple
from gym.spaces import Box, Discrete

'''

A class to wrap and extend the Property object implemented in JSBSim

'''

Property = namedtuple('Property', ['name_jsbsim', 'description', 'min', 'max', 'access', 'spaces', 'update'])

Property.__new__.__defaults__ = ( None, float('-inf'), float('+inf'),'RW', Box, None)
