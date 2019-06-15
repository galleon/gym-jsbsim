from os import listdir
from os.path import isdir, join

from gym.envs.registration import registry, register, make, spec
from gym_jsbsim.heading_control_task import HeadingControlTask
from gym_jsbsim.simulation_parameters import ROOT_DIR, AIRCRAFT_PATH

"""

This script registers JSBSimEnv

with OpenAI Gym so that they can be instantiated with a gym.make(id,task,aircraft_name)

 command.


 To use do:

       env = gym.make('GymJsbsim-{task}-{aircraft_name}-v0')

"""

ABS_AIRCRAFT_PATH = join(ROOT_DIR, AIRCRAFT_PATH)

AIRCRAFT_NAMES = [d for d in listdir(ABS_AIRCRAFT_PATH) if isdir(join(ABS_AIRCRAFT_PATH, d))]
TASKS = dict(HeadingControlTask=HeadingControlTask)


for aircraft_name in AIRCRAFT_NAMES:
    for task in TASKS:
        register(
            id=f'GymJsbsim-{task}-{aircraft_name}-v0',
            entry_point='gym_jsbsim.jsbsim_env:JSBSimEnv',
            kwargs=dict(task=TASKS[task], aircraft_name=aircraft_name)

        )
