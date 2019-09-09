from gym_jsbsim.task import Task
from gym_jsbsim.catalogs.catalog import Catalog as c
import math
import random
import numpy as np

"""

    A task in which the agent must perform steady, level flight maintaining its

    initial heading.
"""

class ApproachControlTask(Task):


    state_var = [c.delta_altitude,
             c.delta_heading,
             c.velocities_v_down_fps,
             c.velocities_vc_fps
             ]

    action_var = [c.fcs_aileron_cmd_norm,
                  c.fcs_elevator_cmd_norm,
                  c.fcs_rudder_cmd_norm,
                  c.fcs_throttle_cmd_norm,
                  ]

    init_conditions = { # 'ic/h-sl-ft', 'initial altitude MSL [ft]'
                        c.ic_h_sl_ft: 2000,
                        #'ic/terrain-elevation-ft', 'initial terrain alt [ft]'
                        c.ic_terrain_elevation_ft: 0,
                        #'ic/long-gc-deg', 'initial geocentric longitude [deg]'
                        c.ic_long_gc_deg: 1.493405,#1.442031
                        #'ic/lat-geod-deg', 'initial geodesic latitude [deg]'
                        c.ic_lat_geod_deg: 43.502138,#43.607181,
                        #'ic/u-fps', 'body frame x-axis velocity; positive forward [ft/s]'
                        c.ic_u_fps: 800,
                        #'ic/v-fps', 'body frame y-axis velocity; positive right [ft/s]'
                        c.ic_v_fps: 0,
                        #'ic/w-fps', 'body frame z-axis velocity; positive down [ft/s]'
                        c.ic_w_fps: 0,
                        #'ic/p-rad_sec', 'roll rate [rad/s]'
                        c.ic_p_rad_sec: 0,
                        #'ic/q-rad_sec', 'pitch rate [rad/s]'
                        c.ic_q_rad_sec: 0,
                        #'ic/r-rad_sec', 'yaw rate [rad/s]'
                        c.ic_r_rad_sec: 0,
                        #'ic/roc-fpm', 'initial rate of climb [ft/min]'
                        c.ic_roc_fpm: 0,
                        #'ic/psi-true-deg', 'initial (true) heading [deg]'
                        c.ic_psi_true_deg: 323,
                        # target heading deg
                        c.target_heading_deg: 323,
                        # target heading deg
                        c.target_altitude_ft: 2000,
                        # controls command
                        #'fcs/throttle-cmd-norm', 'throttle commanded position, normalised', 0., 1.
                        c.fcs_throttle_cmd_norm: 0.8,
                        #'fcs/mixture-cmd-norm', 'engine mixture setting, normalised', 0., 1.
                        c.fcs_mixture_cmd_norm: 1,
                        # gear up
                        c.gear_gear_pos_norm : 0,
                        c.gear_gear_cmd_norm: 0,
                        c.steady_flight:150
    }


    def get_reward(self, state, sim):
        '''
        Reward with delta and altitude heading directly in the input vector state.
        '''
        # inverse of the proportional absolute value of the minimal angle between the initial and current heading ...
        heading_r = math.exp(-math.fabs(sim.get_property_value(c.delta_heading)))

        # inverse of the proportional absolute value between the initial and current altitude ...
        alt_r = math.exp(-math.fabs(sim.get_property_value(c.delta_altitude)))
    
        angle_speed_r = math.exp(-(0.1*math.fabs(sim.get_property_value(c.accelerations_a_pilot_x_ft_sec2)) + 
                                0.1*math.fabs(sim.get_property_value(c.accelerations_a_pilot_y_ft_sec2)) + 
                                0.8*math.fabs(sim.get_property_value(c.accelerations_a_pilot_z_ft_sec2))))

        # Add selective pressure to model that end up the simulation earlier
        reward = 0.4*heading_r + 0.1*alt_r + 0.4*angle_speed_r
        
        return reward


    def is_terminal(self, state, sim):
        sim.set_property_value(c.target_altitude_ft, sim.get_property_value(c.target_altitude_ft)-1)
        return sim.get_property_value(c.target_altitude_ft) < 20 or math.fabs(sim.get_property_value(c.delta_altitude)) > 5000

