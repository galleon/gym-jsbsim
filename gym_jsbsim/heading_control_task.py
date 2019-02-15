import math
import random
import gym_jsbsim.properties as prp
from gym_jsbsim import utils
from gym_jsbsim.simulation import Simulation
from gym_jsbsim.base_flight_task import BaseFlightTask
from gym_jsbsim.properties import BoundedProperty, Property
from gym_jsbsim.aircraft import Aircraft
from typing import Dict, Tuple, Sequence, NamedTuple
import json
import configparser


### Collect Config Value
config = configparser.ConfigParser()
print(config.read('/home/ubuntu/gym-jsbsim/gym_jsbsim/config-state-action.ini'))
#print(config.sections())

### collect state var from config file
state_list = config.get('SA_DEFAULT', 'states').split('\n')
state_var = ()
for s in state_list:
    #print(s)
    state_var = state_var + (prp.prp_dict[s],)

action_list = config.get('SA_DEFAULT', 'actions').split('\n')
action_var = ()
for a in action_list:
    #print(a)
    action_var = action_var + (prp.prp_dict[a],)

class HeadingControlTask(BaseFlightTask):
    """
    A task in which the agent must perform steady, level flight maintaining its
    initial heading.
    """

    ### Set config var
    THROTTLE_CMD = float(config["HEADING_CONTROL_TASK_CONDITION"]["throttle_cmd"])
    MIXTURE_CMD = float(config["HEADING_CONTROL_TASK_CONDITION"]["mixture_cmd"])
    #INITIAL_HEADING_DEG = float(config["HEADING_CONTROL_TASK_CONDITION"]["initial_heading_deg"])
    #INITIAL_ALTITUDE_FT = float(config["HEADING_CONTROL_TASK_CONDITION"]["initial_altitude_ft"])
    #TARGET_HEADING_DEG = float(config["HEADING_CONTROL_TASK_CONDITION"]["target_heading_deg"])
    DEFAULT_EPISODE_TIME_S = 1000.
    ALTITUDE_SCALING_FT = 150
    MAX_ALTITUDE_DEVIATION_FT = 800  # terminate if altitude error exceeds this

    def __init__(self, step_frequency_hz: float, aircraft: Aircraft,
                 episode_time_s: float = DEFAULT_EPISODE_TIME_S, debug: bool = False) -> None:
        """
        Constructor.

        :param step_frequency_hz: the number of agent interaction steps per second
        :param aircraft: the aircraft used in the simulation
        """
        self.max_time_s = episode_time_s
        episode_steps = math.ceil(self.max_time_s * step_frequency_hz)
        self.steps_left = BoundedProperty('info/steps_left', 'steps remaining in episode', 0,
                                          episode_steps)
        self.nb_episodes = Property('info/nb_episodes', 'number of episodes since the beginning')
        self.aircraft = aircraft



        #self.state_variables = (prp.pitch_rad, prp.roll_rad, prp.sideslip_deg, prp.v_north_fps, prp.v_east_fps, prp.altitude_sl_ft, # minimal state variables for the task
        #                       prp.v_down_fps, prp.p_radps, prp.q_radps, prp.r_radps) # additional state variables used for reward shaping
        self.state_variables = state_var
        #print("state_variables = " , self.state_variables)
        #self.action_variables = (prp.aileron_cmd, prp.elevator_cmd, prp.rudder_cmd)
        self.action_variables = action_var
        #print("action_variables = ", self.action_variables)
        super().__init__(debug)

    def get_initial_conditions(self) -> Dict[Property, float]:
        self.INITIAL_ALTITUDE_FT = random.uniform(1000, 32000)
        self.INITIAL_HEADING_DEG = random.uniform(prp.heading_deg.min, prp.heading_deg.max)
        #print("self.INITIAL_ALTITUDE_FT", self.INITIAL_ALTITUDE_FT)
        #print("self.INITIAL_HEADING_DEG", self.INITIAL_HEADING_DEG)
        initial_conditions = {prp.initial_altitude_ft: self.INITIAL_ALTITUDE_FT,
                              prp.initial_u_fps: self.aircraft.get_cruise_speed_fps(),
                              prp.initial_v_fps: 0,
                              prp.initial_w_fps: 0,
                              prp.initial_p_radps: 0,
                              prp.initial_latitude_geod_deg: 49.243824,
                              prp.initial_longitude_geoc_deg: -121.887340,
                              prp.initial_q_radps: 0,
                              prp.initial_r_radps: 0,
                              prp.initial_roc_fpm: 0,
                              prp.all_engine_running: -1,
                              prp.initial_heading_deg: self.INITIAL_HEADING_DEG,
                              self.nb_episodes: 0
                             }
        return initial_conditions

    def _update_custom_properties(self, sim: Simulation) -> None:
        self._decrement_steps_left(sim)

    def _decrement_steps_left(self, sim: Simulation):
        sim[self.steps_left] -= 1

    def _is_terminal(self, sim: Simulation, state: NamedTuple) -> bool:
        # terminate when time >= max, but use math.isclose() for float equality test
        terminal_step = sim[self.steps_left] <= 0
        #terminal_step = sim[prp.dist_travel_m]  >= 100000
        return terminal_step or self._altitude_out_of_bounds(sim, state)
    
    def _get_reward(self, sim: Simulation, last_state: NamedTuple, action: NamedTuple, new_state: NamedTuple) -> float:
        # inverse of the proportional absolute value of the minimal angle between the initial and current heading ... 
        abs_h = math.fabs(self.INITIAL_HEADING_DEG - last_state.attitude_psi_deg)
        heading_r = 1.0/math.sqrt((0.1*min(360-abs_h, abs_h)+1))
        # inverse of the proportional absolute value between the initial and current altitude ... 
        alt_r = 1.0/math.sqrt((0.1*math.fabs(self.INITIAL_ALTITUDE_FT - last_state.position_h_sl_ft)+1))

        return (heading_r + alt_r)/2.0
    
    def _get_reward_cplx(self, sim: Simulation, last_state: NamedTuple, action: NamedTuple, new_state: NamedTuple) -> float:
        # Get   
        track_deg = prp.Vector2(last_state.velocities_v_east_fps, last_state.velocities_v_north_fps).heading_deg()
        normalised_error_track_deg = math.fabs(utils.reduce_reflex_angle_deg(track_deg - self.INITIAL_HEADING_DEG)) / 180.0
        normalised_altitude_error = min(math.fabs(last_state.position_h_sl_ft - self.INITIAL_ALTITUDE_FT) / self.INITIAL_ALTITUDE_FT, 1.0)
        target_reward = - normalised_error_track_deg - normalised_altitude_error

        # Get negative reward proportional to normalised speed angles and vertical speed
        normalised_angle_speed = min((math.fabs(last_state.velocities_p_rad_sec) + math.fabs(last_state.velocities_q_rad_sec) + math.fabs(last_state.velocities_r_rad_sec)) / (3*2*math.pi), 1.0)
        normalised_vertical_speed = min(math.fabs(last_state.velocities_v_down_fps) / self.INITIAL_ALTITUDE_FT, 1.0)
        stabilisation_reward = - math.exp(- sim[self.nb_episodes] / 100) * (normalised_angle_speed + normalised_vertical_speed)

        return target_reward + stabilisation_reward

    def _altitude_out_of_bounds(self, sim: Simulation, state: NamedTuple) -> bool:
        altitude_error_ft = math.fabs(state.position_h_sl_ft - self.INITIAL_ALTITUDE_FT)
        return abs(altitude_error_ft) > self.MAX_ALTITUDE_DEVIATION_FT

    def _new_episode_init(self, sim: Simulation) -> None:
        super()._new_episode_init(sim)
        sim.set_throttle_mixture_controls(self.THROTTLE_CMD, self.MIXTURE_CMD)
        sim[self.steps_left] = self.steps_left.max
        sim[self.nb_episodes] += 1

    def get_props_to_output(self, sim: Simulation) -> Tuple:
        return (*self.state_variables, prp.lat_geod_deg, prp.lng_geoc_deg, self.steps_left)


class HeadingControlTask_1Bis(BaseFlightTask):
    """
    A task in which the agent must perform steady, level flight maintaining its
    initial heading and reach a target waypoint at target time.
    """

    ### Set config var
    THROTTLE_CMD = float(config["HEADING_CONTROL_TASK_CONDITION"]["throttle_cmd"])
    MIXTURE_CMD = float(config["HEADING_CONTROL_TASK_CONDITION"]["mixture_cmd"])
    INITIAL_HEADING_DEG = float(config["HEADING_CONTROL_TASK_CONDITION"]["initial_heading_deg"])
    INITIAL_ALTITUDE_FT = float(config["HEADING_CONTROL_TASK_CONDITION"]["initial_altitude_ft"])
    TARGET_HEADING_DEG = float(config["HEADING_CONTROL_TASK_CONDITION"]["target_heading_deg"])
    TARGET_TIME = float(config["HEADING_CONTROL_TASK_CONDITION"]["target_time"])
    TARGET_WP_LAT_DEG = float(config["HEADING_CONTROL_TASK_CONDITION"]["target_latitude_geod_deg"])
    TARGET_WP_LON_DEG = float(config["HEADING_CONTROL_TASK_CONDITION"]["target_longitude_geod_deg"])
    DEFAULT_EPISODE_TIME_S = TARGET_TIME+300
    ALTITUDE_SCALING_FT = 150
    MAX_ALTITUDE_DEVIATION_FT = 1000  # terminate if altitude error exceeds this


    def __init__(self, step_frequency_hz: float, aircraft: Aircraft,
                 episode_time_s: float = DEFAULT_EPISODE_TIME_S, debug: bool = False) -> None:
        """
        Constructor.

        :param step_frequency_hz: the number of agent interaction steps per second
        :param aircraft: the aircraft used in the simulation
        """
        self.max_time_s = episode_time_s
        episode_steps = math.ceil(self.max_time_s * step_frequency_hz)
        self.steps_left = BoundedProperty('info/steps_left', 'steps remaining in episode', 0,
                                          episode_steps)
        self.nb_episodes = Property('info/nb_episodes', 'number of episodes since the beginning')
        self.aircraft = aircraft

        # self.state_variables = (prp.pitch_rad, prp.roll_rad, prp.sideslip_deg, prp.v_north_fps, prp.v_east_fps, prp.altitude_sl_ft, # minimal state variables for the task
        #                       prp.v_down_fps, prp.p_radps, prp.q_radps, prp.r_radps) # additional state variables used for reward shaping
        self.state_variables = state_var
        print("state_variables = ", self.state_variables)
        # self.action_variables = (prp.aileron_cmd, prp.elevator_cmd, prp.rudder_cmd)
        self.action_variables = action_var
        print("action_variables = ", self.action_variables)
        super().__init__(debug)

    def get_initial_conditions(self) -> Dict[Property, float]:
        initial_conditions = {prp.initial_altitude_ft: self.INITIAL_ALTITUDE_FT,
                              prp.initial_u_fps: self.aircraft.get_cruise_speed_fps(),
                              prp.initial_v_fps: 0,
                              prp.initial_w_fps: 0,
                              prp.initial_p_radps: 0,
                              prp.initial_latitude_geod_deg: 47.4498333,
                              prp.initial_longitude_geoc_deg: -122.3118333,
                              prp.initial_q_radps: 0,
                              prp.initial_r_radps: 0,
                              prp.initial_roc_fpm: 0,
                              prp.all_engine_running: -1,
                              prp.initial_heading_deg: self.INITIAL_HEADING_DEG,
                              self.nb_episodes: 0
                              }
        return initial_conditions

    def _update_custom_properties(self, sim: Simulation) -> None:
        self._decrement_steps_left(sim)


    def _decrement_steps_left(self, sim: Simulation):
        sim[self.steps_left] -= 1

    def _is_terminal(self, sim: Simulation, state: NamedTuple) -> bool:
        # terminate when time >= max, but use math.isclose() for float equality test
        # check decimal accuracy
        # do we restart simulation if the heading is extremely off track?

        terminal_step = sim[self.steps_left] <= 0
        reached_target = self._is_at_target_wp(sim,state)

        return reached_target or terminal_step or self._altitude_out_of_bounds(sim, state)

    def _is_at_target_wp(self,sim: Simulation, state: NamedTuple)->bool:
        # TBD to check the floating point accuracy
        float_accuracy = 0.0000001
        reached_target = (self.TARGET_WP_LAT_DEG-sim[prp.lat_geod_deg])<float_accuracy and (self.TARGET_WP_LON_DEG-sim[prp.lng_geoc_deg])<float_accuracy
        return reached_target

    def _get_reward(self, sim: Simulation, last_state: NamedTuple, action: NamedTuple,
                           new_state: NamedTuple) -> float:
        heading_r = 1.0 / math.sqrt((0.1 * math.fabs(self.TARGET_HEADING_DEG - last_state.attitude_psi_deg) + 1))
        # alt_r = 2*(self.INITIAL_ALTITUDE_FT/360. - new_state.position_h_sl_ft/360.)
        # print("ALTITUDE REWARD !!! ", self.INITIAL_ALTITUDE_FT, last_state.position_h_sl_ft)
        alt_r = 1.0 / math.sqrt((0.1 * math.fabs(self.INITIAL_ALTITUDE_FT - last_state.position_h_sl_ft) + 1))
        # print(heading_r + alt_r, -(heading_r + alt_r), -(heading_r + alt_r)/2.)
        time_r = 0
        if self._is_at_target_wp(sim,last_state):
            time_r = 1.0 / math.sqrt((0.1 * math.fabs(self.TARGET_TIME - sim.get_sim_time()) + 1))

        return (heading_r + alt_r+ time_r) / 3.0

    def _get_reward_cmplx(self, sim: Simulation, last_state: NamedTuple, action: NamedTuple, new_state: NamedTuple) -> float:
        # Get negative reward proportional to normalised heading and altitude errors
        track_deg = prp.Vector2(last_state.velocities_v_east_fps, last_state.velocities_v_north_fps).heading_deg()
        normalised_error_track_deg = math.fabs(
            utils.reduce_reflex_angle_deg(track_deg - self.INITIAL_HEADING_DEG)) / 180.0
        normalised_altitude_error = min(
            math.fabs(last_state.position_h_sl_ft - self.INITIAL_ALTITUDE_FT) / self.INITIAL_ALTITUDE_FT, 1.0)
        target_reward = - normalised_error_track_deg - normalised_altitude_error

        # Get negative reward proportional to normalised speed angles and vertical speed
        normalised_angle_speed = min((math.fabs(last_state.velocities_p_rad_sec) + math.fabs(
            last_state.velocities_q_rad_sec) + math.fabs(last_state.velocities_r_rad_sec)) / (3 * 2 * math.pi), 1.0)
        normalised_vertical_speed = min(math.fabs(last_state.velocities_v_down_fps) / self.INITIAL_ALTITUDE_FT, 1.0)
        stabilisation_reward = - math.exp(- sim[self.nb_episodes] / 100) * (normalised_angle_speed + normalised_vertical_speed)

        return target_reward + stabilisation_reward

    def _altitude_out_of_bounds(self, sim: Simulation, state: NamedTuple) -> bool:
        altitude_error_ft = math.fabs(state.position_h_sl_ft - self.INITIAL_ALTITUDE_FT)
        return abs(altitude_error_ft) > self.MAX_ALTITUDE_DEVIATION_FT

    def _heading_out_of_bounds(self,sim:Simulation,state:NamedTuple,new_state:NamedTuple) -> bool:
        heading_error_deg = math.fabs(self.TARGET_HEADING_DEG  - new_state.attitude_psi_deg)
        return heading_error_deg>90.0

    def _new_episode_init(self, sim: Simulation) -> None:
        super()._new_episode_init(sim)
        sim.set_throttle_mixture_controls(self.THROTTLE_CMD, self.MIXTURE_CMD)
        sim[self.steps_left] = self.steps_left.max
        sim[self.nb_episodes] += 1

    def get_props_to_output(self, sim: Simulation) -> Tuple:
        return (*self.state_variables, prp.lat_geod_deg, prp.lng_geoc_deg, self.steps_left)


class TurnHeadingChangeLevelControlTask(HeadingControlTask):
    """
    A task in which the agent must make a turn and change its altitude
    """

    TARGET_HEADING_DEG = 360
    TARGET_ALTITUDE_FT = 3000

    def _get_reward(self, sim: Simulation, last_state: NamedTuple, action: NamedTuple, new_state: NamedTuple) -> float:
        # Get negative reward proportional to normalised heading and altitude errors
        track_deg = prp.Vector2(last_state.velocities_v_east_fps, last_state.velocities_v_north_fps).heading_deg()
        normalised_error_track_deg = math.fabs(utils.reduce_reflex_angle_deg(track_deg - self.INITIAL_HEADING_DEG)) / 180.0
        normalised_altitude_error = min(math.fabs(last_state.position_h_sl_ft - self.TARGET_ALTITUDE_FT) / self.INITIAL_ALTITUDE_FT, 1.0)
        target_reward = - normalised_error_track_deg - normalised_altitude_error

        # Get negative reward proportional to normalised speed angles and vertical speed
        normalised_angle_speed = min((math.fabs(last_state.velocities_p_rad_sec) + math.fabs(last_state.velocities_q_rad_sec) + math.fabs(last_state.velocities_r_rad_sec)) / (3*2*math.pi), 1.0)
        normalised_vertical_speed = min(math.fabs(last_state.velocities_v_down_fps) / self.INITIAL_ALTITUDE_FT, 1.0)
        stabilisation_reward = - math.exp(- sim[self.nb_episodes] / 100) * (normalised_angle_speed + normalised_vertical_speed)
        
        return target_reward + stabilisation_reward
