from gym_jsbsim.properties import*
from gym_jsbsim.task import Task
import random

"""
    A task in which the agent must follow taxiway centerline trajectory.
    The aircraft velocity should not exceed 20 knot/s in straightline and 7 knots/s during turn.
"""


state_var = [v_air,
             shortest_dist,
             d1, d2, d3, d4, d5, d6, d7, d8,
             a1, a2, a3, a4, a5, a6, a7, a8]

action_var = [steer_cmd_norm,
              left_brake_cmd_norm,
              right_brake_cmd_norm,
              throttle_cmd
]

INIT_AC_LON = 1.372124
INIT_AC_LAT = 43.618951
INIT_AC_HEADING = 320
INITIAL_ALTITUDE_FT = 11.52
INITIAL_VELOCITY_U = 33.76/3.0 #20 knots/sec

init_conditions = { # 'ic/h-sl-ft', 'initial altitude MSL [ft]'
                    initial_altitude_ft: INITIAL_ALTITUDE_FT,
                    #'ic/terrain-elevation-ft', 'initial terrain alt [ft]'
                    initial_terrain_altitude_ft: INITIAL_ALTITUDE_FT, # Blagnac Ariport Altittude (148.72m = 487.9265f)
                    ic_h_agl_ft : INITIAL_ALTITUDE_FT,
                    #'ic/long-gc-deg', 'initial geocentric longitude [deg]'
                    initial_longitude_geoc_deg: INIT_AC_LON,
                    #'ic/lat-geod-deg', 'initial geodesic latitude [deg]'
                    initial_latitude_geod_deg: INIT_AC_LAT,
                    #'ic/u-fps', 'body frame x-axis velocity; positive forward [ft/s]'
                    initial_u_fps: INITIAL_VELOCITY_U, # ie: 10 knots
                    #'ic/v-fps', 'body frame y-axis velocity; positive right [ft/s]'
                    initial_v_fps: 0,
                    #'ic/w-fps', 'body frame z-axis velocity; positive down [ft/s]'
                    initial_w_fps: 0,
                    #'ic/p-rad_sec', 'roll rate [rad/s]'
                    initial_p_radps: 0,
                    #'ic/q-rad_sec', 'pitch rate [rad/s]'
                    initial_q_radps: 0,
                    #'ic/r-rad_sec', 'yaw rate [rad/s]'
                    initial_r_radps: 0,
                    #'ic/roc-fpm', 'initial rate of climb [ft/min]'
                    initial_roc_fpm: 0,
                    #'ic/psi-true-deg', 'initial (true) heading [deg]'
                    initial_heading_deg: INIT_AC_HEADING,
                    # target heading deg
                    target_heading_deg: INIT_AC_HEADING,
                    # controls command
                    #'fcs/throttle-cmd-norm', 'throttle commanded position, normalised', 0., 1.
                    throttle_cmd: 0,
                    #'fcs/mixture-cmd-norm', 'engine mixture setting, normalised', 0., 1.
                    mixture_cmd: 1,
                    # all engine running
                    all_engine_running: -1,
                    # gear up
                    gear_all_cmd: 1,
                    nb_step:0
}


def get_reward(state, sim):
    '''
    Reward with delta and altitude heading directly in the input vector state.
    '''
    # inverse of the normalised value of q, r, p acceleartion
    #angle_speed_r = 1.0/math.sqrt((math.fabs(last_state.accelerations_pdot_rad_sec2) + math.fabs(last_state.accelerations_qdot_rad_sec2) + math.fabs(last_state.accelerations_rdot_rad_sec2) + math.fabs(last_state.velocities_v_down_fps)) / 4.0 + 1)

    # Add selective pressure to model that end up the simulation earlier
    if sim.get_property_value(v_air) <= 20 and sim.get_property_value(v_air) >= 7:
        vel_r = 1.
    else:
        vel_r = 0.

    dist_r = 1.0/(sim.get_property_value(shortest_dist)+1)
    
    reward = (vel_r + dist_r) / 2.0
    '''
    if sim.get_property_value(sim_time_s) < 300:
        reward = reward / 3.0
    if sim.get_property_value(sim_time_s) >= 300 and sim.get_property_value(sim_time_s) < 1000:
        reward = reward / 2.0
    '''
    return reward   


def is_terminal(state, sim):

    # End up the simulation after 1200 secondes or if the aircraft is under or above 500 feet of its target altitude or velocity under 400f/s
    #print("state", state)
    #print("sim.get_property_value(v_air)", sim.get_property_value(v_air), sim.get_property_value(u_fps), sim.get_property_value(v_fps))
    return sim.get_property_value(sim_time_s)>=120 or math.fabs(sim.get_property_value(v_air)) >= 40 or math.fabs(sim.get_property_value(v_air)) <= 3


TaxiControlTask = Task(state_var, action_var, init_conditions, get_reward, is_terminal)
