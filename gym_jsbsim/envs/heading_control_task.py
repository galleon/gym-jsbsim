from gym_jsbsim.properties import*
from gym_jsbsim.task import Task
import random

"""
    A task in which the agent must perform steady, level flight maintaining its initial heading.
"""


state_var = [delta_altitude,
             delta_heading,
             v_down_fps,
             v_air
             #p_dot,
             #q_dot,
             #r_dot
]

action_var = [aileron_cmd,
              elevator_cmd,
              rudder_cmd,
              throttle_cmd
]

init_conditions = { # 'ic/h-sl-ft', 'initial altitude MSL [ft]'
                    initial_altitude_ft: 10000,
                    #'ic/terrain-elevation-ft', 'initial terrain alt [ft]'
                    initial_terrain_altitude_ft: 0,
                    #'ic/long-gc-deg', 'initial geocentric longitude [deg]'
                    initial_longitude_geoc_deg: 1.442031,
                    #'ic/lat-geod-deg', 'initial geodesic latitude [deg]'
                    initial_latitude_geod_deg: 43.607181,
                    #'ic/u-fps', 'body frame x-axis velocity; positive forward [ft/s]'
                    initial_u_fps: 800,
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
                    initial_heading_deg: 100,
                    # target heading deg
                    target_heading_deg: 100,
                    # target heading deg
                    target_altitude_ft: 10000,
                    # controls command
                    #'fcs/throttle-cmd-norm', 'throttle commanded position, normalised', 0., 1.
                    throttle_cmd: 0.8,
                    #'fcs/mixture-cmd-norm', 'engine mixture setting, normalised', 0., 1.
                    mixture_cmd: 1,
                    # all engine running
                    all_engine_running: -1,
                    engine_running: 1,
                    # gear up
                    gear_all_cmd: 0,
                    gear:0
}


def get_reward(state, sim):
    '''
    Reward with delta and altitude heading directly in the input vector state.
    '''
    # inverse of the proportional absolute value of the minimal angle between the initial and current heading ...
    heading_r = 1.0/math.sqrt(math.fabs(sim.get_property_value(delta_heading)+1))

    # inverse of the proportional absolute value between the initial and current altitude ...
    alt_r = 1.0/math.sqrt(math.fabs(sim.get_property_value(delta_altitude))+1)

    # inverse of the normalised value of q, r, p acceleartion
    #angle_speed_r = 1.0/math.sqrt((math.fabs(last_state.accelerations_pdot_rad_sec2) + math.fabs(last_state.accelerations_qdot_rad_sec2) + math.fabs(last_state.accelerations_rdot_rad_sec2) + math.fabs(last_state.velocities_v_down_fps)) / 4.0 + 1)

    # Add selective pressure to model that end up the simulation earlier
    reward = (heading_r + alt_r) / 2.0
    if sim.get_property_value(sim_time_s) < 300:
        reward = reward / 3.0
    if sim.get_property_value(sim_time_s) >= 300 and sim.get_property_value(sim_time_s) < 1000:
        reward = reward / 2.0
    return reward   


def is_terminal(state, sim):
    # Change heading every 300 seconds
    if (int(sim.get_property_value(sim_time_s))%300==1 and int(sim.get_property_value(sim_time_s))!=1):

        new_alt = sim.get_property_value(target_altitude_ft)# + random.uniform(-1000, 1000)
        new_heading = sim.get_property_value(target_heading_deg) + random.uniform(-135, 135)
        if (new_heading <= 0):
            new_heading = 360 - new_heading
        if (new_heading >= 360):
            new_heading = new_heading - 360

        print(f'Time to change: {sim.get_property_value(sim_time_s)} (Altitude: {sim.get_property_value(target_altitude_ft)} -> {new_alt}, Heading: {sim.get_property_value(target_heading_deg)} -> {new_heading})')
        sim.set_property_value(target_altitude_ft, new_alt)
        sim.set_property_value(target_heading_deg, new_heading)

    # End up the simulation after 1200 secondes or if the aircraft is under or above 500 feet of its target altitude
    return sim.get_property_value(sim_time_s)>=1200 or math.fabs(sim.get_property_value(delta_altitude)) >= 500


HeadingControlTask = Task(state_var, action_var, init_conditions, get_reward, is_terminal)
