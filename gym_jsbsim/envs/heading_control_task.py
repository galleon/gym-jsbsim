from gym_jsbsim.task import Task
from gym_jsbsim.catalogs.catalog import Catalog as c
import math
import random
import numpy as np

"""
    A task in which the agent must perform steady, level flight maintaining its initial heading.
    Every 150 sec a new target heading is set.
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

    init_conditions = { c.ic_h_sl_ft: 10000,
                        c.ic_terrain_elevation_ft: 0,
                        c.ic_long_gc_deg: 1.442031,
                        c.ic_lat_geod_deg: 43.607181,
                        c.ic_u_fps: 800,
                        c.ic_v_fps: 0,
                        c.ic_w_fps: 0,
                        c.ic_p_rad_sec: 0,
                        c.ic_q_rad_sec: 0,
                        c.ic_r_rad_sec: 0,
                        c.ic_roc_fpm: 0,
                        c.ic_psi_true_deg: 100,
                        c.target_heading_deg: 100,
                        c.target_altitude_ft: 10000,
                        c.fcs_throttle_cmd_norm: 0.8,
                        c.fcs_mixture_cmd_norm: 1,
                        c.gear_gear_pos_norm : 0,
                        c.gear_gear_cmd_norm: 0,
                        c.steady_flight:150
    }


    def get_reward(self, state, sim):
        '''
        Reward accroding to altitude, heading and axes accelerations.
        '''
        
        heading_r = math.exp(-math.fabs(sim.get_property_value(c.delta_heading)))
        alt_r = math.exp(-math.fabs(sim.get_property_value(c.delta_altitude)))
        angle_speed_r = math.exp(-(0.1*math.fabs(sim.get_property_value(c.accelerations_a_pilot_x_ft_sec2)) + 
                                0.1*math.fabs(sim.get_property_value(c.accelerations_a_pilot_y_ft_sec2)) + 
                                0.8*math.fabs(sim.get_property_value(c.accelerations_a_pilot_z_ft_sec2))))
        
        reward = 0.4*heading_r + 0.4*alt_r + 0.2*angle_speed_r
        
        return reward


    def is_terminal(self, state, sim):
        # if accelerations are too high stop the simulation
        acc = 36 # 1.2G
        if (sim.get_property_value(c.simulation_sim_time_sec)>10):
            if math.fabs(sim.get_property_value(c.accelerations_a_pilot_x_ft_sec2)) > acc or math.fabs(sim.get_property_value(c.accelerations_a_pilot_y_ft_sec2)) > acc or math.fabs(sim.get_property_value(c.accelerations_a_pilot_z_ft_sec2)) > acc:
                return True

        # Change heading every 150 seconds
        if sim.get_property_value(c.simulation_sim_time_sec) >= sim.get_property_value(c.steady_flight):
            # if the traget heading was not reach before, we stop the simulation
            if math.fabs(sim.get_property_value(c.delta_heading)) > 10:
                return True

            # We set the new target heading every 150s in an incremental difficulty: 10, -20, 30, -40, 50, -60, 70, -80, 90
            new_alt = sim.get_property_value(c.target_altitude_ft)
            angle = int(sim.get_property_value(c.steady_flight)/150) * 10
            if int(sim.get_property_value(c.steady_flight)/150) % 2 == 1:
                new_heading = sim.get_property_value(c.target_heading_deg) + angle
            else:
                new_heading = sim.get_property_value(c.target_heading_deg) - angle
                
            new_heading = (new_heading +360) % 360

            #print(f'Time to change: {sim.get_property_value(c.simulation_sim_time_sec)} (Altitude: {sim.get_property_value(c.target_altitude_ft)} -> {new_alt}, Heading: {sim.get_property_value(c.target_heading_deg)} -> {new_heading})')
            
            sim.set_property_value(c.target_altitude_ft, new_alt)
            sim.set_property_value(c.target_heading_deg, new_heading)

            sim.set_property_value(c.steady_flight,sim.get_property_value(c.steady_flight)+150)
        
            # End up the simulation after 1500 secondes or if the aircraft is under or above 300 feet of its target altitude or velocity under 400f/s
        return sim.get_property_value(c.simulation_sim_time_sec)>=1500 or math.fabs(sim.get_property_value(c.delta_altitude)) >= 300 or bool(sim.get_property_value(c.detect_extreme_state))

