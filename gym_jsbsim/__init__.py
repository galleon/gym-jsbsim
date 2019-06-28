from os import listdir,environ
from os.path import isdir, join

from gym.envs.registration import registry, register, make, spec
from gym_jsbsim.envs import TASKS
from gym_jsbsim.catalogs import Catalog

"""

This script registers JSBSimEnv

with OpenAI Gym so that they can be instantiated with a gym.make(id,task,aircraft_name)

 command.


 To use do:

       env = gym.make('GymJsbsim-{task}-{aircraft_name}-v0')

"""

ROOT_DIR = environ['JSBSIM_ROOT_DIR']
ABS_AIRCRAFT_PATH = join(ROOT_DIR, "aircraft")

AIRCRAFT_NAMES = [d for d in listdir(ABS_AIRCRAFT_PATH) if isdir(join(ABS_AIRCRAFT_PATH, d))]


for aircraft_name in AIRCRAFT_NAMES:
    for task in TASKS:
        register(
            id=f'GymJsbsim-{task}-{aircraft_name}-v0',
            entry_point='gym_jsbsim.jsbsim_env:JSBSimEnv',
            kwargs=dict(task=TASKS[task], aircraft_name=aircraft_name)

        )