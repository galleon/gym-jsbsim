# -*- coding: utf-8 -*-

from gym_jsbsim.properties import*
from gym_jsbsim.task import Task

"""

    A task in which the agent must perform steady, level flight maintaining its

    initial heading.
"""


state_var = [ delta_altitude,

             altitude_sl_ft,

             delta_heading,

             v_down_fps,

             v_air,

             p_dot,

             q_dot,

             r_dot,
             ]

action_var = [ aileron_cmd,

              elevator_cmd,

              rudder_cmd,

              throttle_cmd,

            ]

init_conditions = { #'ic/h-sl-ft', 'initial altitude MSL [ft]')

                    initial_altitude_ft : 5000,

                    #'ic/terrain-elevation-ft', 'initial terrain alt [ft]')

                    initial_terrain_altitude_ft : 0,

                    #'ic/long-gc-deg', 'initial geocentric longitude [deg]')

                    initial_longitude_geoc_deg : 1.442031,

                    #'ic/lat-geod-deg', 'initial geodesic latitude [deg]')

                    initial_latitude_geod_deg : 43.607181,

                    #'ic/u-fps', 'body frame x-axis velocity; positive forward [ft/s]')

                    initial_u_fps : 0,

                    #'ic/v-fps', 'body frame y-axis velocity; positive right [ft/s]')

                    initial_v_fps : 0,

                    #'ic/w-fps', 'body frame z-axis velocity; positive down [ft/s]')

                    initial_w_fps :  0,

                    #'ic/p-rad_sec', 'roll rate [rad/s]')

                    initial_p_radps : 0,

                    #'ic/q-rad_sec', 'pitch rate [rad/s]')

                    initial_q_radps : 0,

                    #'ic/r-rad_sec', 'yaw rate [rad/s]')

                    initial_r_radps : 0,

                    #'ic/roc-fpm', 'initial rate of climb [ft/min]')

                    initial_roc_fpm : 0,

                    #'ic/psi-true-deg', 'initial (true) heading [deg]')

                    initial_heading_deg : 100,

                    # target heading deg

                    target_heading_deg : 100,

                    # target heading deg

                    target_altitude_ft : 100,

                    # controls command

                    #'fcs/throttle-cmd-norm', 'throttle commanded position, normalised', 0., 1.)

                    throttle_cmd : 0.8,

                    #'fcs/mixture-cmd-norm', 'engine mixture setting, normalised', 0., 1.)

                    mixture_cmd : 0.8,

                    #target time

                    target_time : 400,

                    #target waypoint latitude

                    target_latitude_geod_deg : 49.0447,

                    #target waypoint longitude

                    target_longitude_geod_deg : -120.3206,

        }

def get_reward(state):
    return 1

def is_terminal(state):
    return False

HeadingControlTask = Task(state_var,action_var,init_conditions,get_reward,is_terminal)