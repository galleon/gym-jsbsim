# -*- coding: utf-8 -*-

from gym.envs.registration import registry, register, make, spec
from gym_jsbsim.heading_control_task import HeadingControlTask

"""

This script registers JSBSimEnv

with OpenAI Gym so that they can be instantiated with a gym.make(id,task,aircraft_name)

 command.


 To use do:

       env = gym.make('GymJsbsim-v0',task,aircraft_name)

"""


register(

    id='GymJsbsim-HeadingControlTask-A320-v0',

    entry_point='gym_jsbsim.jsbsim_env:JSBSimEnv',

    kwargs = dict(task = HeadingControlTask, aircraft_name = "A320")

)