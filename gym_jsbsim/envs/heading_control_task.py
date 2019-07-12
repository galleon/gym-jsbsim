from gym_jsbsim.task import Task
from gym_jsbsim.catalogs.catalog import Catalog as c
import math
import random
import numpy as np

"""

    A task in which the agent must perform steady, level flight maintaining its

    initial heading.
"""

class HeadingControlTask(Task):


    state_var = [c.delta_altitude,
             c.delta_heading,
             c.velocities_v_down_fps,
             c.velocities_vc_fps,
             c.velocities_p_rad_sec,
             c.velocities_q_rad_sec,
             c.velocities_r_rad_sec
             ]

    action_var = [c.fcs_aileron_cmd_norm,
                  c.fcs_elevator_cmd_norm,
                  c.fcs_rudder_cmd_norm,
                  c.fcs_throttle_cmd_norm,
                  ]

    init_conditions = { # 'ic/h-sl-ft', 'initial altitude MSL [ft]'
                        c.ic_h_sl_ft: 10000,
                        #'ic/terrain-elevation-ft', 'initial terrain alt [ft]'
                        c.ic_terrain_elevation_ft: 0,
                        #'ic/long-gc-deg', 'initial geocentric longitude [deg]'
                        c.ic_long_gc_deg: 1.442031,
                        #'ic/lat-geod-deg', 'initial geodesic latitude [deg]'
                        c.ic_lat_geod_deg: 43.607181,
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
                        c.ic_psi_true_deg: 100,
                        # target heading deg
                        c.target_heading_deg: 100,
                        # target heading deg
                        c.target_altitude_ft: 10000,
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

        # inverse of the normalised value of q, r, p acceleartion
        angle_speed_r = math.exp(-(0.33*math.fabs(sim.get_property_value(c.velocities_p_rad_sec)*180/math.pi) + 
                                0.33*math.fabs(sim.get_property_value(c.velocities_q_rad_sec)*180/math.pi) + 
                                0.33*math.fabs(sim.get_property_value(c.velocities_r_rad_sec)*180/math.pi)))

        # Add selective pressure to model that end up the simulation earlier
        reward = 0.4*heading_r + 0.4*alt_r + 0.2*angle_speed_r
        '''
        if sim.get_property_value(c.simulation_sim_time_sec) < 300:
            reward = reward / 3.0
        if sim.get_property_value(c.simulation_sim_time_sec) >= 300 and sim.get_property_value(c.simulation_sim_time_sec) < 1000:
            reward = reward / 2.0
        '''
        return reward


    def is_terminal(self, state, sim):
        # Change heading every 150 seconds
        if sim.get_property_value(c.simulation_sim_time_sec) >= sim.get_property_value(c.steady_flight):
            new_alt = sim.get_property_value(c.target_altitude_ft)# + random.uniform(-1000, 1000)
            angle = int(sim.get_property_value(c.steady_flight)/150) * 10
            if int(sim.get_property_value(c.steady_flight)/150) % 2 == 1:
                new_heading = sim.get_property_value(c.target_heading_deg) + angle
            else:
                new_heading = sim.get_property_value(c.target_heading_deg) - angle
                
            if (new_heading <= 0):
                new_heading = 360 - new_heading
            if (new_heading >= 360):
                new_heading = new_heading - 360

            print(f'Time to change: {sim.get_property_value(c.simulation_sim_time_sec)} (Altitude: {sim.get_property_value(c.target_altitude_ft)} -> {new_alt}, Heading: {sim.get_property_value(c.target_heading_deg)} -> {new_heading})')
            sim.set_property_value(c.target_altitude_ft, new_alt)
            sim.set_property_value(c.target_heading_deg, new_heading)

            sim.set_property_value(c.steady_flight,sim.get_property_value(c.steady_flight)+150)
        # End up the simulation after 1200 secondes or if the aircraft is under or above 500 feet of its target altitude or velocity under 400f/s
        return sim.get_property_value(c.simulation_sim_time_sec)>=1500 or math.fabs(sim.get_property_value(c.delta_altitude)) >= 300 or math.fabs(sim.get_property_value(c.delta_heading) >= 90)

