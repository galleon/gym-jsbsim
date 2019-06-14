# -*- coding: utf-8 -*-

from gym.envs.registration import registry, register, make, spec
from gym_jsbsim.heading_control_task import HeadingControlTask
from gym_jsbsim.simulation_parameters import ROOT_DIR, AircraftPath

"""

This script registers JSBSimEnv

with OpenAI Gym so that they can be instantiated with a gym.make(id,task,aircraft_name)

 command.


 To use do:

       env = gym.make('GymJsbsim-{task}-{aircraft_name}-v0')

"""

from os import listdir
from os.path import isdir, join

aircraft_path = join(ROOT_DIR,AircraftPath)

aircraft_names = [d for d in listdir(aircraft_path) if isdir(join(aircraft_path, d))]
tasks = dict(HeadingControlTask = HeadingControlTask)



for aircraft_name in aircraft_names :
    for task in tasks :
        register(

            id=f'GymJsbsim-{task}-{aircraft_name}-v0',

            entry_point='gym_jsbsim.jsbsim_env:JSBSimEnv',

            kwargs = dict(task = tasks[task], aircraft_name = aircraft_name)

        )