# -*- coding: utf-8 -*-

"""

    Parameter file to configure JSBSim simulation

"""
from os import path


ROOT_DIR = "H:/jsbsim-code" # the root directory where JSBSim starts looking for its system directories.
AircraftPath = "aircraft" # the relative path to the aircraft config file directories.
EnginePath = "engine"    # the path relative to the engine config file directories.
SystemPath = "systems"   # the path relative to the systems config file directories.

#ic file
IC_FILE = path.join(path.dirname(path.abspath(__file__)),'reset00.xml')
useStoredPath=False # true if the stored AircraftPath should be used to get the IC file


jsbsim_freq = 60  # JSBSim integration frequency
agent_interaction_steps = 5 # simulation steps before the agent interact


