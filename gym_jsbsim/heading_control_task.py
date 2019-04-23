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
from .utils import reduce_reflex_angle_deg




class HeadingControlTask(BaseFlightTask):
    """
    A task in which the agent must perform steady, level flight maintaining its
    initial heading.
    """
    ### Collect Config Value
    config = configparser.ConfigParser()
    print(config.read('/home/ubuntu/gym-jsbsim/gym_jsbsim/config-state-action.ini'))
    #print(config.sections())

    ### collect state var from config file
    state_list = config.get('SA_TAXI', 'states').split('\n')
    print("STATE LIST = ", state_list)
    state_var = ()
    for s in state_list:
        state_var = state_var + (prp.prp_dict[s],)

    action_list = config.get('SA_TAXI', 'actions').split('\n')
    print("ACTION LIST = ", action_list)
    action_var = ()
    for a in action_list:
        action_var = action_var + (prp.prp_dict[a],)

    ### Set config var
    THROTTLE_CMD = float(config["HEADING_CONTROL_TASK_CONDITION"]["throttle_cmd"])
    MIXTURE_CMD = float(config["HEADING_CONTROL_TASK_CONDITION"]["mixture_cmd"])
    INITIAL_LAT = float(config["HEADING_CONTROL_TASK_CONDITION"]["initial_latitude_geod_deg"])
    INITIAL_LONG = float(config["HEADING_CONTROL_TASK_CONDITION"]["initial_longitude_geoc_deg"])
    DEFAULT_EPISODE_TIME_S = 1000.
    ALTITUDE_SCALING_FT = 150
    MAX_ALTITUDE_DEVIATION_FT = 800  # terminate if altitude error exceeds this
    TIME_TO_CHANGE_HEADING_ALT = random.uniform((DEFAULT_EPISODE_TIME_S*5.)*0.33, (DEFAULT_EPISODE_TIME_S*5.)*0.66)
    

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

        self.state_variables = state_var
        self.action_variables = action_var

        super().__init__(debug)

    def get_initial_conditions(self) -> Dict[Property, float]:
        self.INITIAL_ALTITUDE_FT = random.uniform(10000, 20000)
        self.INITIAL_HEADING_DEG = random.uniform(prp.heading_deg.min, prp.heading_deg.max)
        self.TARGET_ALTITUDE_FT = self.INITIAL_ALTITUDE_FT
        self.TARGET_HEADING_DEG = self.INITIAL_HEADING_DEG
        self.INITIAL_VELOCITY_U = self.aircraft.get_cruise_speed_fps()
        self.INITIAL_VELOCITY_V = 0
        
        
        initial_conditions = {prp.initial_altitude_ft: self.INITIAL_ALTITUDE_FT,
                              prp.initial_u_fps: self.INITIAL_VELOCITY_U,
                              prp.initial_v_fps: self.INITIAL_VELOCITY_V,
                              prp.initial_w_fps: 0,
                              prp.initial_p_radps: 0,
                              prp.initial_latitude_geod_deg: self.INITIAL_LAT,
                              prp.initial_longitude_geoc_deg: self.INITIAL_LONG,
                              prp.initial_q_radps: 0,
                              prp.initial_r_radps: 0,
                              prp.initial_roc_fpm: 0,
                              prp.all_engine_running: -1,
                              prp.initial_heading_deg: self.INITIAL_HEADING_DEG,
                              prp.initial_altitude_ft: self.INITIAL_ALTITUDE_FT,
                              prp.delta_heading: reduce_reflex_angle_deg(self.INITIAL_HEADING_DEG - self.TARGET_HEADING_DEG),
                              prp.delta_altitude: self.INITIAL_ALTITUDE_FT - self.TARGET_ALTITUDE_FT,
                              prp.target_altitude_ft: self.TARGET_ALTITUDE_FT,
                              prp.target_heading_deg: self.TARGET_HEADING_DEG,
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
        return terminal_step or math.fabs(sim[prp.delta_altitude]) >= 600 or math.fabs(sim[prp.delta_heading]) >= 80
    
    def _get_reward_with_heading(self, sim: Simulation, last_state: NamedTuple, action: NamedTuple, new_state: NamedTuple) -> float:
        '''
        reward with current heading and initial heading
        '''
        # inverse of the proportional absolute value of the minimal angle between the initial and current heading ... 
        abs_h = math.fabs(self.INITIAL_HEADING_DEG - last_state.attitude_psi_deg)
        heading_r = 1.0/math.sqrt((0.5*min(360-abs_h, abs_h)+1))
        # inverse of the proportional absolute value between the initial and current ground speed ... 
        vel_i = math.sqrt(math.pow(self.INITIAL_VELOCITY_U,2) + math.pow(self.INITIAL_VELOCITY_V,2)) 
        vel_c = math.sqrt(math.pow(last_state.velocities_u_fps,2) + math.pow(last_state.velocities_v_fps,2)) 
        vel_r = 1.0/math.sqrt((0.1*math.fabs(vel_i - vel_c)+1))
        # inverse of the proportional absolute value between the initial and current altitude ... 
        alt_r = 1.0/math.sqrt((0.1*math.fabs(self.INITIAL_ALTITUDE_FT - last_state.position_h_sl_ft)+1))
        #print(" -v- ", self.INITIAL_VELOCITY_U, last_state.velocities_u_fps, vel_r, " -h- ", self.INITIAL_HEADING_DEG, last_state.attitude_psi_deg, heading_r, " -a- ", self.INITIAL_ALTITUDE_FT, last_state.position_h_sl_ft, alt_r, " -r- ", (heading_r + alt_r + vel_r)/3.0)
        return (heading_r + alt_r + vel_r)/3.0
    
    def _get_reward(self, sim: Simulation, last_state: NamedTuple, action: NamedTuple, new_state: NamedTuple) -> float:
        '''
        Reward with delta and altitude heading directly in the input vector state.
        '''
        # inverse of the proportional absolute value of the minimal angle between the initial and current heading ... 
        heading_r = 1.0/math.sqrt((0.1*math.fabs(last_state.position_delta_heading_to_target_deg)+1))
        # inverse of the proportional absolute value between the initial and current ground speed ... 
        vel_i = math.sqrt(math.pow(self.INITIAL_VELOCITY_U,2) + math.pow(self.INITIAL_VELOCITY_V,2)) 
        vel_c = math.sqrt(math.pow(last_state.velocities_u_fps,2) + math.pow(last_state.velocities_v_fps,2)) 
        vel_r = 1.0/math.sqrt((0.1*math.fabs(vel_i - vel_c)+1))
        # inverse of the proportional absolute value between the initial and current altitude ... 
        alt_r = 1.0/math.sqrt((0.1*math.fabs(last_state.position_delta_altitude_to_target_ft)+1))
        #print(" -v- ", self.INITIAL_VELOCITY_U, last_state.velocities_u_fps, vel_r, " -h- ", self.INITIAL_HEADING_DEG, last_state.attitude_psi_deg, heading_r, " -a- ", self.INITIAL_ALTITUDE_FT, last_state.position_h_sl_ft, alt_r, " -r- ", (heading_r + alt_r + vel_r)/3.0)
        return (heading_r + alt_r + vel_r)/3.0
    
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

class ChangeHeadingControlTask(BaseFlightTask):
    """
    A task in which the agent must perform steady, level flight maintaining its
    initial heading and altitude and changint them to another ones in the middle of the simiulation
    """

    ### Collect Config Value
    config = configparser.ConfigParser()
    print(config.read('/home/ubuntu/gym-jsbsim/gym_jsbsim/config-state-action.ini'))
    #print(config.sections())

    ### collect state var from config file
    state_list = config.get('SA_DEFAULT', 'states').split('\n')
    print("STATE LIST = ", state_list)
    state_var = ()
    for s in state_list:
        state_var = state_var + (prp.prp_dict[s],)

    action_list = config.get('SA_DEFAULT', 'actions').split('\n')
    print("ACTION LIST = ", action_list)
    action_var = ()
    for a in action_list:
        action_var = action_var + (prp.prp_dict[a],)

    ### Set config var
    THROTTLE_CMD = float(config["HEADING_CONTROL_TASK_CONDITION"]["throttle_cmd"])
    MIXTURE_CMD = float(config["HEADING_CONTROL_TASK_CONDITION"]["mixture_cmd"])
    INITIAL_LAT = float(config["HEADING_CONTROL_TASK_CONDITION"]["initial_latitude_geod_deg"])
    INITIAL_LONG = float(config["HEADING_CONTROL_TASK_CONDITION"]["initial_longitude_geoc_deg"])
    DEFAULT_EPISODE_TIME_S = 3800.
    ALTITUDE_SCALING_FT = 150
    MAX_ALTITUDE_DEVIATION_FT = 800  # terminate if altitude error exceeds this
    THRESHOLD_CONTROL = 0.5
    PENALTY_CONTROL = -0.2

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

        self.state_variables = state_var
        self.action_variables = action_var

        super().__init__(debug)

    def get_initial_conditions(self) -> Dict[Property, float]:
        self.INITIAL_ALTITUDE_FT = random.uniform(10000, 20000)
        self.INITIAL_HEADING_DEG = 90#random.uniform(prp.heading_deg.min, prp.heading_deg.max)
        self.TARGET_ALTITUDE_FT = self.INITIAL_ALTITUDE_FT
        self.TARGET_HEADING_DEG = self.INITIAL_HEADING_DEG
        self.INITIAL_VELOCITY_U = self.aircraft.get_cruise_speed_fps()
        self.INITIAL_VELOCITY_V = 0
        self.ALREADY_CHANGE = False
        self.LAST_CONTROL_STATE = [0,0,0,0,0]

        self.TIME_TO_CHANGE_HEADING_ALT = random.uniform((self.DEFAULT_EPISODE_TIME_S*5.)*0.33, (self.DEFAULT_EPISODE_TIME_S*5.)*0.66)
        self.NEW_ALTITUDE_FT = self.TARGET_ALTITUDE_FT + random.uniform(-4000, 4000)
        new_heading = self.TARGET_HEADING_DEG + random.uniform(-90, 90)
        if (new_heading <= 0):
            new_heading = 360 - new_heading
        if (new_heading >= 360):
            new_heading = new_heading - 360
        self.NEW_HEADING_DEG = new_heading
        
        initial_conditions = {prp.initial_altitude_ft: self.INITIAL_ALTITUDE_FT,
                              prp.initial_u_fps: self.INITIAL_VELOCITY_U,
                              prp.initial_v_fps: self.INITIAL_VELOCITY_V,
                              prp.initial_w_fps: 0,
                              prp.initial_p_radps: 0,
                              prp.initial_latitude_geod_deg: self.INITIAL_LAT,
                              prp.initial_longitude_geoc_deg: self.INITIAL_LONG,
                              prp.initial_q_radps: 0,
                              prp.initial_r_radps: 0,
                              prp.initial_roc_fpm: 0,
                              prp.all_engine_running: -1,
                              prp.initial_heading_deg: self.INITIAL_HEADING_DEG,
                              prp.initial_altitude_ft: self.INITIAL_ALTITUDE_FT,
                              prp.delta_heading: reduce_reflex_angle_deg(self.INITIAL_HEADING_DEG - self.TARGET_HEADING_DEG),
                              prp.delta_altitude: self.INITIAL_ALTITUDE_FT - self.TARGET_ALTITUDE_FT,
                              prp.target_altitude_ft: self.TARGET_ALTITUDE_FT,
                              prp.target_heading_deg: self.TARGET_HEADING_DEG,
                              self.nb_episodes: 0
                             }
        print(f'Time to change INIT: {self.TIME_TO_CHANGE_HEADING_ALT} (Altitude: {self.TARGET_ALTITUDE_FT} -> {self.NEW_ALTITUDE_FT}, Heading: {self.TARGET_HEADING_DEG} -> {self.NEW_HEADING_DEG})')
        return initial_conditions

    def _update_custom_properties(self, sim: Simulation) -> None:
        self._decrement_steps_left(sim)

    def _decrement_steps_left(self, sim: Simulation):
        sim[self.steps_left] -= 1

    def _is_terminal(self, sim: Simulation, state: NamedTuple) -> bool:
        # Change target ALT and HEADING
        #print(f'nombre episode: {sim[self.nb_episodes]}, nombre step left: {sim[self.steps_left]}')

        '''
        if (sim[self.steps_left] <= self.TIME_TO_CHANGE_HEADING_ALT and not self.ALREADY_CHANGE):
            print(f'Time to change: {self.TIME_TO_CHANGE_HEADING_ALT} (Altitude: {self.TARGET_ALTITUDE_FT} -> {self.NEW_ALTITUDE_FT}, Heading: {self.TARGET_HEADING_DEG} -> {self.NEW_HEADING_DEG})')
            sim[prp.target_altitude_ft] = self.NEW_ALTITUDE_FT
            sim[prp.target_heading_deg] = self.NEW_HEADING_DEG
            self.ALREADY_CHANGE = True
        '''
        
        # Change alt and heading every 2000 steps
        if (sim[self.steps_left]%2000==1):
            
            new_alt = sim[prp.target_altitude_ft] + random.uniform(-4000, 500)
            new_heading = sim[prp.target_heading_deg] + random.uniform(-135, 135)
            if (new_heading <= 0):
                new_heading = 360 - new_heading
            if (new_heading >= 360):
                new_heading = new_heading - 360
            
            print(f'Time to change: {sim[self.steps_left]} (Altitude: {sim[prp.target_altitude_ft]} -> {new_alt}, Heading: {sim[prp.target_heading_deg]} -> {new_heading})')
            sim[prp.target_altitude_ft] = new_alt
            sim[prp.target_heading_deg] = new_heading
        
        
        terminal_step = sim[self.steps_left] <= 0
        sim[self.nb_episodes] += 1
        #terminal_step = sim[prp.dist_travel_m]  >= 100000
        return terminal_step or sim[prp.altitude_sl_ft] <= 2000
    
    
    def _get_reward(self, sim: Simulation, last_state: NamedTuple, action: NamedTuple, new_state: NamedTuple) -> float:
        '''
        Reward with delta and altitude heading directly in the input vector state.
        '''
        # inverse of the proportional absolute value of the minimal angle between the initial and current heading ... 
        heading_r = 1.0/math.sqrt((0.1*math.fabs(last_state.position_delta_heading_to_target_deg)+1))
        # inverse of the proportional absolute value between the initial and current ground speed ... 
        #vel_i = math.sqrt(math.pow(self.INITIAL_VELOCITY_U,2) + math.pow(self.INITIAL_VELOCITY_V,2)) 
        #vel_c = math.sqrt(math.pow(last_state.velocities_u_fps,2) + math.pow(last_state.velocities_v_fps,2)) 
        #vel_r = 1.0/math.sqrt((0.1*math.fabs(vel_i - vel_c)+1))
        # inverse of the proportional absolute value between the initial and current altitude ... 
        alt_r = 1.0/math.sqrt((0.1*math.fabs(last_state.position_delta_altitude_to_target_ft)+1))
        #print(" -v- ", self.INITIAL_VELOCITY_U, last_state.velocities_u_fps, vel_r, " -h- ", self.INITIAL_HEADING_DEG, last_state.attitude_psi_deg, heading_r, " -a- ", self.INITIAL_ALTITUDE_FT, last_state.position_h_sl_ft, alt_r, " -r- ", (heading_r + alt_r + vel_r)/3.0)

        #check to strong manoeuvres
        sum_penalty_control_state = 0

        if (sim[self.nb_episodes]>=1):
            delta_left_aileron = math.fabs(self.LAST_CONTROL_STATE[0] - sim[prp.aileron_left])
            delta_right_aileron = math.fabs(self.LAST_CONTROL_STATE[1] - sim[prp.aileron_right])
            delta_elevator = math.fabs(self.LAST_CONTROL_STATE[2] - sim[prp.elevator])
            delta_rudder = math.fabs(self.LAST_CONTROL_STATE[3] - sim[prp.rudder])
            delta_throttle = math.fabs(self.LAST_CONTROL_STATE[4] - sim[prp.throttle])

            
            if delta_left_aileron >= self.THRESHOLD_CONTROL:
                sum_penalty_control_state += self.PENALTY_CONTROL
            if delta_right_aileron >= self.THRESHOLD_CONTROL:
                sum_penalty_control_state += self.PENALTY_CONTROL
            if delta_elevator >= self.THRESHOLD_CONTROL:
                sum_penalty_control_state += self.PENALTY_CONTROL 
            if delta_rudder >= self.THRESHOLD_CONTROL:
                sum_penalty_control_state += self.PENALTY_CONTROL 
            if delta_throttle >= self.THRESHOLD_CONTROL:
                sum_penalty_control_state += self.PENALTY_CONTROL  
        
        #reward if finish the simulation ponderate with the quality of the fly
        reward_nb_episode = (heading_r + alt_r) / (2.0 * max(sim[self.steps_left],1.0))

        self.LAST_CONTROL_STATE = [sim[prp.aileron_left], sim[prp.aileron_right], sim[prp.elevator], sim[prp.rudder], sim[prp.throttle]]

        return (2*heading_r + 2*alt_r + sum_penalty_control_state + reward_nb_episode) / 6.0
    

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

class TaxiControlTask(BaseFlightTask):
    """
    A task in which the agent must perform taxiing phases
    """
    ### Collect Config Value
    config = configparser.ConfigParser()
    print(config.read('/home/ubuntu/gym-jsbsim/gym_jsbsim/config-state-action.ini'))
    #print(config.sections())

    ### collect state var from config file
    state_list = config.get('SA_TAXI', 'states').split('\n')
    print("STATE LIST = ", state_list)
    state_var = ()
    for s in state_list:
        state_var = state_var + (prp.prp_dict[s],)

    action_list = config.get('SA_TAXI', 'actions').split('\n')
    print("ACTION LIST = ", action_list)
    action_var = ()
    for a in action_list:
        action_var = action_var + (prp.prp_dict[a],)

    ### Set config var
    THROTTLE_CMD = 0 #float(config["HEADING_CONTROL_TASK_CONDITION"]["throttle_cmd"])
    MIXTURE_CMD = 0 #float(config["HEADING_CONTROL_TASK_CONDITION"]["mixture_cmd"])
    INITIAL_LAT = 43.621148#float(config["HEADING_CONTROL_TASK_CONDITION"]["initial_latitude_geod_deg"])
    INITIAL_LONG = 1.374493#float(config["HEADING_CONTROL_TASK_CONDITION"]["initial_longitude_geoc_deg"])
    DEFAULT_EPISODE_TIME_S = 1000.
    THRESHOLD_CONTROL = 0.5
    PENALTY_CONTROL = -0.2
    PATH = [(1.374477624944279, 43.6211633737098), (1.3743272656325203, 43.62131103904902), (1.3741793070531096, 43.62146186573268), (1.3740293116857967, 43.62161080957677), (1.3738886235758112, 43.62175599305125), (1.373742572787681, 43.621901983302934), (1.3736029636929847, 43.62204539072935), (1.373454892130852, 43.62219464199941), (1.3733022001839732, 43.62234796778087), (1.3731583681720616, 43.62249456575382), (1.3730139369695487, 43.622641506722545), (1.3728703035582486, 43.62278727668668), (1.3727249735741136, 43.62293537172754), (1.372574509720457, 43.623086399390836), (1.3724320983060316, 43.62323071098527), (1.3722915878847304, 43.62337443572298), (1.3721504884317743, 43.62351793157518), (1.3720135453506035, 43.623658309602064), (1.3718719693496577, 43.62380259954975), (1.3717244662083776, 43.623950706165566), (1.3715832698467805, 43.62409518992504), (1.3714197203029508, 43.62423527753822), (1.3712165883997154, 43.62434930230902), (1.3710147566912303, 43.62442622772231), (1.3707985339852928, 43.62447743721084), (1.370580561644388, 43.624496063119395), (1.3703517940883985, 43.62448985374439), (1.3701236113800754, 43.62448526078474), (1.3698985242412691, 43.62448147258848), (1.369659811049948, 43.62447623380795), (1.3694124217542247, 43.62446997933365), (1.3691772759406218, 43.62446491747482), (1.3689258073569177, 43.624458350934226), (1.3686597847332456, 43.624449898817986), (1.3684055953391838, 43.62444298233072), (1.3681546459035343, 43.62443545991138), (1.3679060248484796, 43.62442884339179), (1.3676544032601192, 43.62442181833319), (1.3674005562345746, 43.624414755329795), (1.367144678907823, 43.62440719859183), (1.3668860808867576, 43.624399538871614), (1.3666109823633799, 43.62440458553711), (1.3663021847027914, 43.62446696123886), (1.3660120133384996, 43.62460944047199), (1.3657925965127895, 43.62482995839729), (1.3656924085733508, 43.62507891870476), (1.365657141918345, 43.62531754049101), (1.3656147546123856, 43.62553477355542), (1.3655845931312378, 43.62572845397495), (1.3655559095733374, 43.62591079476529), (1.3655301852786488, 43.62608558402758), (1.3655015567484667, 43.62626854907844), (1.365473300007807, 43.62645305456712), (1.365441790577889, 43.62664450492722), (1.3654132978210545, 43.62682745145496), (1.365387309983836, 43.6270035858134), (1.3653616924199086, 43.627179810178724), (1.36533748990122, 43.627349072011974), (1.3653126001552842, 43.62752024503748), (1.3652855268488095, 43.62769787780961), (1.3652614895470017, 43.62786576517723), (1.3652222530755185, 43.6280319796215), (1.3651550776731978, 43.62817667173593), (1.3650552679113999, 43.628317379054), (1.364920873680585, 43.62843978772064), (1.3647657507316657, 43.62853705015475), (1.3645800450472179, 43.628617119720914), (1.3643738581857194, 43.628684647668514), (1.3641679138052598, 43.62875381685097), (1.3639598648403073, 43.628823095245394), (1.3637591448758053, 43.628890897701), (1.3635441765986587, 43.6289613321633), (1.3633136607371092, 43.62903577778764), (1.3630854267420838, 43.62911008098965), (1.362852351957984, 43.62918535662701), (1.3626141871909967, 43.62926145121386), (1.362384266499932, 43.62933624901135), (1.362149174176065, 43.62941259306436), (1.3619156422320344, 43.6294876333761), (1.3616949892884045, 43.62956009489799), (1.3614762622952874, 43.62963239855822), (1.3612478891558213, 43.62969988941954), (1.3610023448682895, 43.62973652242626), (1.3607753012640673, 43.62974121453212), (1.3605671360243812, 43.6297249580276), (1.3603222839479765, 43.62970007547308), (1.3600785491841119, 43.62967644980105), (1.3598132569570147, 43.62964932840282), (1.3595566839419582, 43.629622886280735), (1.3593184619134255, 43.629600187114946), (1.3590920574822234, 43.62957917744402), (1.358850659402978, 43.629555953189765), (1.358607332728727, 43.629531993179725), (1.3583716795139842, 43.62950959810999), (1.3581351279557203, 43.62948820961342), (1.3578861394842228, 43.62946242050686), (1.3576472067239163, 43.62943981424791), (1.3574083702659971, 43.62941619177095), (1.357163470453964, 43.62939268130966), (1.3569146067083728, 43.62936713100512), (1.356675438185074, 43.62934467195867), (1.356421168582002, 43.62933426741473), (1.356158750435266, 43.629365278364325), (1.3558928139432576, 43.6294531869656), (1.3556719813576266, 43.62958483427686), (1.3555010171364326, 43.62975232166556), (1.3553603072752356, 43.629926011211836), (1.3552165201462358, 43.630087563008544), (1.3550867561440094, 43.630240651373846), (1.354957536570994, 43.630392390592924), (1.3548257630344307, 43.63054604232141), (1.3546927700203442, 43.63070178971021), (1.3545661031334002, 43.63085173924364), (1.3544351823723721, 43.6310056434543), (1.3543060106686324, 43.631157309934714), (1.3541757708505644, 43.63130943309103), (1.3540433753689154, 43.63146429545563), (1.3539089620979181, 43.63162064855437), (1.353771463713779, 43.63177880759041), (1.353639882270694, 43.6319328614069), (1.3535131154818618, 43.63208107957274), (1.353383804890791, 43.632217893231925), (1.3532442021967064, 43.632331138774624), (1.353088474516933, 43.63242736944393), (1.3529138718174976, 43.63250600683542), (1.352714708736506, 43.63256238419703), (1.3525042704176546, 43.6325905907528), (1.3522829380482442, 43.63259217023061), (1.3520456891575972, 43.632585654594116), (1.3518018124257278, 43.63258029012025), (1.3515589862077138, 43.63257498299159), (1.351321407705803, 43.63256999796774), (1.3510901348716087, 43.63256555646383), (1.3508466645907802, 43.63255917019191), (1.3505897408580398, 43.63255211640805), (1.3503349688281037, 43.632544498296795), (1.350097128625364, 43.632539487539844), (1.349867786080239, 43.63253496500584), (1.3496268887469154, 43.632529434357004), (1.3493819250582368, 43.63252367546406), (1.349126830312574, 43.632516693585075), (1.3488608732696554, 43.632526375512214), (1.3486115190986496, 43.63257092970923), (1.3483854454019124, 43.63265425017058), (1.3481919413781018, 43.632774285490136), (1.3480291075597626, 43.632898138478076), (1.3478533880021326, 43.63302125023029), (1.3476780678588163, 43.63314505417878), (1.3474945216491616, 43.63327420644576), (1.3473086901928364, 43.63340321553897), (1.3471262542535953, 43.63353101809885), (1.3469501824214, 43.63365571763269), (1.3467812692434045, 43.63377651976286), (1.346602532287416, 43.63390246487089), (1.346416935051654, 43.63403104186345), (1.3462344932288863, 43.63415877151557), (1.346046870262102, 43.63428879784041), (1.3458698909296622, 43.634414682917885), (1.3456981673531163, 43.63453613800926), (1.3455273798859557, 43.63465838983205), (1.3453445029559072, 43.63478643245135), (1.3451630170662123, 43.63493222649979), (1.3450236800492856, 43.63511045408853), (1.3449490021036883, 43.63531487826848), (1.3449471438879734, 43.63552709109813), (1.3450139462210342, 43.63572709765404), (1.3451188993934966, 43.635902542593904), (1.345214346094389, 43.63607510229418), (1.3453139416328725, 43.63625165577126), (1.3454119012032908, 43.63642430923127), (1.3455084561224187, 43.636593083027506), (1.345603883638076, 43.63675691304324), (1.3457005072597017, 43.63692843010812), (1.3457979597631669, 43.63709850185441), (1.3458966355310253, 43.63727452210706), (1.3459945969645748, 43.63744854770751), (1.3460906573612816, 43.63761618711546), (1.3461843634451869, 43.63778002787853), (1.3462793935163078, 43.6379433161125), (1.3463733390840718, 43.638106077366864), (1.346468175925334, 43.63827124472486), (1.346587665845734, 43.63844695159281), (1.3467782278868603, 43.63861104999216), (1.347060454527512, 43.63873691429538), (1.3473877386573785, 43.63879036709815), (1.3477169736096501, 43.638771330910515), (1.3480189095613562, 43.63869752667049), (1.3482889907509663, 43.63862119779481), (1.3485360594830607, 43.638553590976), (1.3487684256530512, 43.638487668517), (1.348994500134302, 43.63842317589471), (1.3492207417266082, 43.63835874228689), (1.3494482991936507, 43.63829447325769), (1.3496762868126218, 43.63822981205829), (1.349907029506929, 43.63816426047859), (1.3501383718564073, 43.63809867466474), (1.3503732846373848, 43.638032457563554), (1.350613564931119, 43.63796576337616), (1.3508488098289049, 43.63789912824033), (1.351090319318733, 43.63783183693421), (1.3513305058444272, 43.637764936732054), (1.3515633552329716, 43.63768514537611), (1.3517805513874641, 43.63756474012534), (1.3519458603071324, 43.63741622393178), (1.352075064347554, 43.63727523946173), (1.352219888187467, 43.63712961901233), (1.3523645758376068, 43.63698158046898), (1.3525114342854956, 43.636832551075315), (1.352661130922283, 43.63668155949625), (1.3528063700721944, 43.63653403837284), (1.352955425756992, 43.63638426303129), (1.3531046189626263, 43.6362333257035), (1.3532615654103948, 43.636079337671795), (1.3534085995524012, 43.63593027629199), (1.353559651966557, 43.63577961819504), (1.3537030843605589, 43.635631302941526), (1.3538467347945498, 43.635484847752714), (1.3539854129287583, 43.63534174639555), (1.354126310057656, 43.63519842573287), (1.3542704878200862, 43.635052547238764), (1.3544090362431973, 43.63491106210533), (1.3545571839449526, 43.63477807806003), (1.3547531157367454, 43.63466033525168), (1.354974267434142, 43.634579943760876), (1.3552043187846285, 43.63453231211871), (1.3554489865424615, 43.634491282913274), (1.3556930871612367, 43.63444957765815), (1.3559341562923295, 43.634407945090935), (1.3561764456548315, 43.634366652306824), (1.3564149481157133, 43.63432545027505), (1.3566630400909854, 43.63428328318427), (1.3569043409173018, 43.63424093379564), (1.357135754853587, 43.63419998251433), (1.3573860500226274, 43.634157712382255), (1.3576361510491939, 43.634115406323495), (1.3578858483221339, 43.634072207944655), (1.3581277932942262, 43.63403113731852), (1.358360587381578, 43.633990481565895), (1.3586078072521217, 43.63394808520934), (1.3588604948505851, 43.63390463751774), (1.359110842660035, 43.63386204309488), (1.359345277146505, 43.633827396028586), (1.35958939321701, 43.633795862234656), (1.3598379832166914, 43.63376342177324), (1.3600902995493012, 43.6337311107106), (1.3603401178184737, 43.633699031484106), (1.360599889055628, 43.633666215151194), (1.3608499801099527, 43.633633878523966), (1.3610875450255562, 43.63360237043291), (1.3613205466944707, 43.63357153193382), (1.3615666282830754, 43.633539415337324), (1.3618187913632864, 43.633508105876416), (1.3620476840482534, 43.63347664649176), (1.3622932519373172, 43.63344555032568), (1.3625395529506692, 43.63341306380702), (1.3627790739973458, 43.63338138610709), (1.3630207065138145, 43.63334925817224), (1.3632524987808583, 43.63331765467963), (1.3634915502707334, 43.6332852437484), (1.363738670801981, 43.63325331353072), (1.3639912000744343, 43.63322090713213), (1.3642419324908637, 43.63319946399701), (1.364518402942896, 43.63321156366213), (1.3647825148910977, 43.63326296683621), (1.3650323118703, 43.633349126587454), (1.3652566570711688, 43.63347147646395), (1.3654394567940196, 43.633631891166004), (1.3655710586764966, 43.63381333971584), (1.3656719847300118, 43.63399953562563), (1.365777272599984, 43.634168313619256), (1.365877038174085, 43.63433669514111), (1.3659786487444534, 43.634504873033976), (1.3660819738090426, 43.634677908320334), (1.366183863236221, 43.634848464997646), (1.36628419212899, 43.635016049916935), (1.3663838766608323, 43.63518155927757), (1.3664848139967882, 43.63535065244456), (1.366583649494179, 43.63551608728691), (1.3666846496823348, 43.63568580851896), (1.3667863472157278, 43.63585636209218), (1.3668875510761285, 43.636024750039304), (1.3669724637404062, 43.63618963061436), (1.3670173832461918, 43.63637666297003), (1.367018208282772, 43.63655397581061), (1.3670225982875548, 43.63673135954121), (1.3670247665050315, 43.636911500544194), (1.3670278665901947, 43.637096918633226), (1.3670292406784794, 43.637284521788594), (1.3670314016753808, 43.63747172293988), (1.3670340724023105, 43.63765757178959), (1.3670363047819256, 43.637841136945916), (1.3670413455675714, 43.63801299411212), (1.36704621995255, 43.63818215636352), (1.3670500220854627, 43.63835797284812), (1.367054731922245, 43.638527416691325), (1.367055660013442, 43.638713566129695), (1.3670600947521772, 43.63889429150681), (1.3670631763124905, 43.639079617169315), (1.3670642198468144, 43.6392743386683), (1.367066439128142, 43.63946421735662), (1.3670682823017184, 43.63965331240947), (1.3670986509821739, 43.63985390868276), (1.3671915222517395, 43.640053283032685), (1.367357582042577, 43.64023806490226), (1.3675857640803688, 43.64038400862933), (1.3678261021189755, 43.6405011587234), (1.3680469393290207, 43.64062342165301), (1.3682626600088248, 43.64073523371052), (1.3684798207755826, 43.64085010250695), (1.3686933031744293, 43.64096189987866), (1.3689033033995257, 43.64107222561325), (1.3691091877335768, 43.64117946671715), (1.36930769004483, 43.64128098508255), (1.3695090441644286, 43.64138558905052), (1.3697157061982264, 43.64149270112516), (1.369930497193966, 43.64160653774314), (1.3701350293587111, 43.641713340729474), (1.3703345227844868, 43.641816250950036), (1.3705386409889748, 43.641922792418995), (1.3707457928185316, 43.6420311259746), (1.370948102076424, 43.6421361550468), (1.3711313985330411, 43.64224304293496), (1.3713006491492068, 43.642388928926884), (1.3714189486225277, 43.642573798468455), (1.371486657070164, 43.64277664090166), (1.3715558437288924, 43.642961441270856), (1.371624785105558, 43.64313612889248), (1.371695051418942, 43.64331708545705), (1.3717631210843002, 43.64349107887097), (1.371830970021004, 43.64366327924267), (1.3719003039860405, 43.64383865588625), (1.3719690091792898, 43.64401309013854), (1.372038317220126, 43.644188596193004), (1.3721078930258281, 43.64436573258158), (1.3721769959228636, 43.64454196932716), (1.3722434413738096, 43.644709010476525), (1.3723107174584803, 43.644877033914106), (1.3723793364906065, 43.64504845754156), (1.3724470159572582, 43.645222265621605), (1.372516869540891, 43.645396596096994), (1.3725849651272832, 43.64557128916987), (1.3726542601708414, 43.64574195319235), (1.372723225676654, 43.64591123605725), (1.372791553341158, 43.64607696334855), (1.372860682707498, 43.64625055582141), (1.3729289607243835, 43.64641940486256), (1.3729957641562016, 43.646582127207814), (1.3730624515554308, 43.646745676442926), (1.3731326735883935, 43.64692465678132), (1.3732053014455794, 43.6471113796709), (1.3732751115252257, 43.64728931853568), (1.3733450380381305, 43.647467795762786), (1.3734164505194646, 43.647649548972126), (1.3734890239705946, 43.64783293759803), (1.3735603504531368, 43.64801348341023), (1.373631689759727, 43.64819368963791), (1.3737018191030066, 43.64837707763461), (1.373772070840609, 43.64855077270086), (1.3738402947562118, 43.64872357131749), (1.373910196036859, 43.64889740965348), (1.3739783630807292, 43.64906813474683), (1.3740516238919356, 43.64923065854911), (1.3741434268002162, 43.6493714132984), (1.374260656020613, 43.64949762785036), (1.3744218934043646, 43.64961290850687), (1.3746044022715291, 43.64971960727257), (1.3747815986888992, 43.64983013080794), (1.3749622824759469, 43.649941443475555), (1.375151627984529, 43.65005977949532), (1.375343509867068, 43.65018020040459), (1.375543729312209, 43.6503074707744), (1.3757425403718377, 43.65043304552662), (1.3759407433757456, 43.650559858300426), (1.3761358197959617, 43.65068192615029), (1.376331226791807, 43.650806112110274), (1.3765214880253027, 43.650925727320065), (1.3767062159672807, 43.65104094657069), (1.3768981284284745, 43.65116068416255), (1.3770967847551991, 43.65128570677849), (1.3772959382740513, 43.65141200714147), (1.377489616896622, 43.651533775399784), (1.3776927111096837, 43.651648618540534), (1.3779243646808352, 43.65173732694294), (1.378207131232167, 43.65179067382432), (1.378505676381838, 43.6517874437521), (1.3787872641276686, 43.65176888948643), (1.3790622152944938, 43.65176073954816), (1.3793289684626044, 43.65175041908069), (1.3795930046061622, 43.65174117567447), (1.3798473059841776, 43.65173054967192), (1.3800895152699555, 43.6517197703379), (1.3803166944949172, 43.651708678798634), (1.38056529738116, 43.65169991974994), (1.380822511983674, 43.65168939400434), (1.3810788330800463, 43.65167999836579), (1.3813279590873393, 43.65166925414277), (1.3815841747015876, 43.651659782329055), (1.3818501698835268, 43.651650658930414), (1.3821112355103942, 43.65164088541478), (1.3823632144783953, 43.65163098573444), (1.3826095963736507, 43.65162022652292), (1.3828705582519871, 43.651595544541614), (1.3831200599555726, 43.65153030742751), (1.383344820377439, 43.65143774786455), (1.383562298801003, 43.65135376338344), (1.3837844567080835, 43.65126766426488), (1.3840158118163208, 43.65118024723607), (1.3842492375250748, 43.65109070869403), (1.3844816976947925, 43.651003171391395), (1.3847020301816346, 43.650917835270825), (1.3849190830930593, 43.65083360080555), (1.3851370493430855, 43.650749420209884), (1.385360341226892, 43.65066324270756), (1.3855812309699882, 43.650578591174636), (1.3857953835060406, 43.650494794021334), (1.3860143644424676, 43.6504109706657), (1.3862277142928643, 43.65032771832634), (1.3864469401887463, 43.65024294949089), (1.3866823893817517, 43.65015312589459), (1.3869121731437772, 43.650065265239604), (1.387144649533937, 43.64997670930058), (1.3873841647926357, 43.64989507238572), (1.3876587124399988, 43.64984102338443), (1.387939565088934, 43.64983575354789), (1.3882140055331533, 43.649874590303945), (1.38845203797371, 43.6499496224602), (1.3886589692909086, 43.6500539931087), (1.3888370439136661, 43.65018389846764), (1.3890212333523524, 43.65032049888026), (1.3892016267415606, 43.65045616899684), (1.3893824852912644, 43.65059212842636), (1.3895672697316312, 43.650731686040466), (1.389745911445979, 43.65086732091984), (1.3899233654034924, 43.65099977093759), (1.390105608103183, 43.651137934276406), (1.390290117703396, 43.65127711349117), (1.3904746095999132, 43.65141633874389), (1.390652309850046, 43.651549417590395), (1.3908243028638452, 43.651677115113394), (1.3909977140673366, 43.651804936145325), (1.391176704684222, 43.65193955223581), (1.3913605426621318, 43.65210088200546), (1.3914914911948257, 43.65229512595027), (1.391553986044139, 43.652504857777195), (1.3916021149874829, 43.65270380141575), (1.3916583227124177, 43.65288779744557), (1.3917116231674556, 43.653073355611156), (1.3917653859408734, 43.65324624470422), (1.39181791507456, 43.653420238452846), (1.3918704402905127, 43.653591819162585), (1.3919236215688107, 43.65377003155394), (1.3919770266594482, 43.65394989901349), (1.3920300078342316, 43.65412860313851), (1.3920840314846359, 43.6543115895191), (1.3921367495858623, 43.65448936843115), (1.392190170249263, 43.65467261255728), (1.3922447172604506, 43.65485267870465), (1.392296795757885, 43.6550275265844), (1.3923508782963776, 43.65520465210851), (1.3924026839741794, 43.655392793186635), (1.3924585667696898, 43.65557615846454), (1.3925276444052948, 43.65575430128916), (1.3926433286823738, 43.65592851212551), (1.3928271469421638, 43.65609000625198), (1.3930656179078518, 43.65621265796513), (1.3933326848090177, 43.65628821312916), (1.3935988976235412, 43.65634072680594), (1.3938447583466218, 43.656399111276976), (1.3940834960669715, 43.65645133657789), (1.3943256847185461, 43.65650675301239), (1.3945719809938486, 43.65656239340664), (1.3948320600744821, 43.65662384352168), (1.3950786505523867, 43.65667902836298), (1.3953183420574993, 43.65673295871859), (1.3955609898130035, 43.65678737215206), (1.3958034973308084, 43.656841772836195), (1.396040349832354, 43.65689449003122), (1.3962823702696534, 43.65694944349203), (1.396525241629632, 43.657003536172226), (1.3967665554004987, 43.657058555128515), (1.3970156262908606, 43.6571152712867), (1.3972550259543304, 43.65717981018656), (1.3974576381732433, 43.65726229183541), (1.3976448600541058, 43.65735977362232), (1.3978349098910083, 43.65745436600466), (1.3980338390809455, 43.65755684555462), (1.398230206808164, 43.65765694875968), (1.3984316393952148, 43.65775975961576), (1.3986349946953605, 43.65786466356427), (1.3988435473481424, 43.65797383881369), (1.3990521966751517, 43.658082078756365), (1.3992531668180388, 43.658184576986976), (1.3994565750746268, 43.658289646573294), (1.3996689009194963, 43.65840010256531), (1.3998852323718376, 43.65851352502557), (1.4001001453301303, 43.658625466296186), (1.4003135006363252, 43.65873715211313), (1.400528380028824, 43.658850414569876), (1.4007399172614667, 43.6589598340121), (1.4009510183074734, 43.65906915417234)]
        

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

        self.state_variables = state_var
        self.action_variables = action_var

        super().__init__(debug)
    
    def get_initial_conditions(self) -> Dict[Property, float]:
        self.INITIAL_HEADING_DEG = 324
        self.INITIAL_ALTITUDE_FT = 11.52
        self.TARGET_HEADING_DEG = self.INITIAL_HEADING_DEG
        self.INITIAL_VELOCITY_U = 33.76/2.0 #20 knots/sec
        self.LAST_CONTROL_STATE = [0,0,0,0,0]
        self.ID_NEXT_PATH = 1

        new_heading = self.TARGET_HEADING_DEG + random.uniform(-90, 90)
        if (new_heading <= 0):
            new_heading = 360 - new_heading
        if (new_heading >= 360):
            new_heading = new_heading - 360
        self.NEW_HEADING_DEG = new_heading
        
        initial_conditions = {prp.initial_altitude_ft: self.INITIAL_ALTITUDE_FT,
                              prp.initial_u_fps: self.INITIAL_VELOCITY_U,
                              prp.initial_v_fps: 0,
                              prp.initial_w_fps: 0,
                              prp.initial_p_radps: 0,
                              prp.initial_latitude_geod_deg: self.PATH[0][1], # start at the first point of the path
                              prp.initial_longitude_geoc_deg: self.PATH[0][0], # start at the first point of the path
                              prp.initial_q_radps: 0,
                              prp.initial_r_radps: 0,
                              prp.initial_roc_fpm: 0,
                              prp.all_engine_running: -1,
                              prp.initial_heading_deg: self.calculate_initial_compass_bearing((self.PATH[0][1],self.PATH[0][0]), self.PATH[self.ID_NEXT_PATH]),
                              prp.gear_all_cmd: 1,
                              prp.delta_heading: reduce_reflex_angle_deg(self.INITIAL_HEADING_DEG - self.TARGET_HEADING_DEG),             
                              prp.target_heading_deg: self.TARGET_HEADING_DEG,
                              self.nb_episodes: 0,
                              prp.ic_h_agl_ft: self.INITIAL_ALTITUDE_FT
                             }
        return initial_conditions
    
    def _update_custom_properties(self, sim: Simulation) -> None:
        self._decrement_steps_left(sim)

    def _decrement_steps_left(self, sim: Simulation):
        sim[self.steps_left] -= 1

    def _is_terminal(self, sim: Simulation, state: NamedTuple) -> bool:
        terminal_step = sim[self.steps_left] <= 0
        sim[self.nb_episodes] += 1

        '''
        # Change  heading every 2000 steps
        over_delta_heading = state.position_delta_heading_to_target_deg > 91
        if (sim[self.steps_left]%200==1):
            
            new_heading = sim[prp.target_heading_deg] + random.uniform(-90, 90)
            if (new_heading <= 0):
                new_heading = 360 - new_heading
            if (new_heading >= 360):
                new_heading = new_heading - 360
            
            print(f'Time to change: {sim[self.steps_left]} (Heading: {sim[prp.target_heading_deg]} -> {new_heading}), GPS Coord: {sim[prp.lng_geoc_deg]},{sim[prp.lat_geod_deg]}')
            sim[prp.target_heading_deg] = new_heading
        '''
        



        return terminal_step or (sim[prp.h1]>91) or (self.ID_NEXT_PATH+4 == len(self.PATH))
    
    
    def _get_reward(self, sim: Simulation, last_state: NamedTuple, action: NamedTuple, new_state: NamedTuple) -> float:
        
        # follow path

        lat = sim[prp.lat_geod_deg]
        lon = sim[prp.lng_geoc_deg] 
        for i in range(self.ID_NEXT_PATH, len(self.PATH)-5):    
            if (math.fabs(sim[prp.heading_deg] - self.calculate_initial_compass_bearing((lat,lon), self.PATH[i]))<90):
                self.ID_NEXT_PATH = i
                break
        
        # aricraft bearing
        aircraft_bearing = sim[prp.heading_deg]

        if self.ID_NEXT_PATH > len(self.PATH):
            sim[prp.h1] = 0
        else:
            sim[prp.h1] = math.fabs(aircraft_bearing - self.calculate_initial_compass_bearing((lat,lon), self.PATH[self.ID_NEXT_PATH]))
        
        if self.ID_NEXT_PATH+1 > len(self.PATH):
            sim[prp.h2] = 0
        else:
            sim[prp.h2] = math.fabs(aircraft_bearing - self.calculate_initial_compass_bearing((lat,lon), self.PATH[self.ID_NEXT_PATH+1]))
        
        if self.ID_NEXT_PATH+2 > len(self.PATH):
            sim[prp.h3] = 0
        else:
            sim[prp.h3] = math.fabs(aircraft_bearing - self.calculate_initial_compass_bearing((lat,lon), self.PATH[self.ID_NEXT_PATH+2]))

        if self.ID_NEXT_PATH+3 > len(self.PATH):
            sim[prp.h4] = 0
        else:
            sim[prp.h4] = math.fabs(aircraft_bearing - self.calculate_initial_compass_bearing((lat,lon), self.PATH[self.ID_NEXT_PATH+3]))

        if self.ID_NEXT_PATH+4 > len(self.PATH):
            sim[prp.h5] = 0
        else:
            sim[prp.h5] = math.fabs(aircraft_bearing - self.calculate_initial_compass_bearing((lat,lon), self.PATH[self.ID_NEXT_PATH+4]))
        

        '''
        Reward with delta and altitude heading directly in the input vector state.
        '''
        # inverse of the proportional absolute value of the minimal angle between the initial and current heading ... 
        heading_r = 1.0/math.sqrt((0.1*math.fabs(last_state.h1)+1))
        # inverse of the proportional absolute value between current and target speed
        #speed_r = 1.0/math.sqrt((0.1*math.fabs(last_state.velocities_vc_fps - 33.76)+1))
        
        if (last_state.velocities_vc_fps < 33.76/4.0) or last_state.velocities_vc_fps > 33.76:
            vel_r = 0
        else:
            vel_r = 1

        # reward nb episode
        reward_nb_episode = (heading_r) / (1.0 * max(sim[self.steps_left],1.0))

        #time = 0
        #if (self._is_terminal(sim, new_state) and self.ID_NEXT_PATH+4 > len(self.PATH)):
        #    time = 1.0/(math.sqrt((sim[self.nb_episodes]/1000.0)))

        #print(f'ID path = {self.ID_NEXT_PATH}, lat,lon = {(lat,lon)}, lat,long path= {self.PATH[self.ID_NEXT_PATH]}, a/c bearing = {aircraft_bearing}, bearing h1,h2,h3,h4,h5 = {self.calculate_initial_compass_bearing((lat,lon), self.PATH[self.ID_NEXT_PATH])},{self.calculate_initial_compass_bearing((lat,lon), self.PATH[self.ID_NEXT_PATH+1])},{self.calculate_initial_compass_bearing((lat,lon), self.PATH[self.ID_NEXT_PATH+2])},{self.calculate_initial_compass_bearing((lat,lon), self.PATH[self.ID_NEXT_PATH+3])}, {self.calculate_initial_compass_bearing((lat,lon), self.PATH[self.ID_NEXT_PATH+4])},{self.calculate_initial_compass_bearing((lat,lon), self.PATH[self.ID_NEXT_PATH+5])} (h1, h2, h3, h4, h5) = ({sim[prp.h1]},{sim[prp.h2]},{sim[prp.h3]},{sim[prp.h4]},{sim[prp.h5]}), reward heading = {heading_r}, reward time = {time}')
        return (heading_r + reward_nb_episode + vel_r) / 3.


    def _new_episode_init(self, sim: Simulation) -> None:
        super()._new_episode_init(sim)
        sim.set_throttle_mixture_controls(self.THROTTLE_CMD, self.MIXTURE_CMD)
        sim[self.steps_left] = self.steps_left.max
        sim[self.nb_episodes] += 1

    def get_props_to_output(self, sim: Simulation) -> Tuple:
        return (*self.state_variables, prp.lat_geod_deg, prp.lng_geoc_deg, self.steps_left)


    def calculate_initial_compass_bearing(self, pointA, pointB):
        """
        Calculates the bearing between two points.
        The formulae used is the following:
            θ = atan2(sin(Δlong).cos(lat2),
                    cos(lat1).sin(lat2) − sin(lat1).cos(lat2).cos(Δlong))
        :Parameters:
        - `pointA: The tuple representing the latitude/longitude for the
            first point. Latitude and longitude must be in decimal degrees
        - `pointB: The tuple representing the latitude/longitude for the
            second point. Latitude and longitude must be in decimal degrees
        :Returns:
        The bearing in degrees
        :Returns Type:
        float
        """
        if (type(pointA) != tuple) or (type(pointB) != tuple):
            raise TypeError("Only tuples are supported as arguments")

        lat1 = math.radians(pointA[0])
        lat2 = math.radians(pointB[1])

        diffLong = math.radians(pointB[0] - pointA[1])

        x = math.sin(diffLong) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
                * math.cos(lat2) * math.cos(diffLong))

        initial_bearing = math.atan2(x, y)

        # Now we have the initial bearing but math.atan2 return values
        # from -180° to + 180° which is not what we want for a compass bearing
        # The solution is to normalize the initial bearing as shown below
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing

class TakeoffControlTask(BaseFlightTask):
    """
    Take-off scenario.
    It is assumed that the aircraft is at the beginning of runway correctly aligned.

    Reward function:
    ---------------
    for A320
    q_radps : pitch rate = ~ 2.5 deg/sec i.e. 0.0436 rad/sec
    V_LOF = v_air(airspeed in knots) @ liftoff
    V_2   = v_air(airspeed in knots) @ 35 ft alt
    V_3   = v_air(airspeed in knots) @ 200 - 300 ft = ~ > V_2 + 10
    max_dist_LOF = 1828m
    """

    ### Collect Config Value
    config = configparser.ConfigParser()
    print(config.read('/home/ubuntu/gym-jsbsim/gym_jsbsim/config-state-action.ini'))
    #print(config.sections())

    ### collect state var from config file
    state_list = config.get('SA_TAKEOFF', 'states').split('\n')
    print("STATE LIST = ", state_list)
    state_var = ()
    for s in state_list:
        state_var = state_var + (prp.prp_dict[s],)

    action_list = config.get('SA_TAKEOFF', 'actions').split('\n')
    print("ACTION LIST = ", action_list)
    action_var = ()
    for a in action_list:
        action_var = action_var + (prp.prp_dict[a],)

    THROTTLE_CMD = 0.2
    MIXTURE_CMD = 0.0
    INITIAL_HEADING_DEG = 143.002 #float(config["TAKEOFF_CONTROL_TASK"]["runway_heading"])# -36.984  : for 14L
    INITIAL_ALTITUDE_FT = 8.48
    TARGET_HEADING_DEG = INITIAL_HEADING_DEG
    DEFAULT_EPISODE_TIME_S = 1000.0


    def __init__(self,step_frequency_hz: float, aircraft: Aircraft,
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

        self.state_variables = state_var
        self.action_variables = action_var


        super().__init__(debug)


    def get_initial_conditions(self) -> Dict[Property, float]:
        self.INITIAL_LAT = 43.6371036997  # float(config["HEADING_CONTROL_TASK_CONDITION"]["initial_latitude_geod_deg"])
        self.INITIAL_LONG = 1.35789480571
        self.takeoff_done = False
        self.INITIAL_ALTITUDE_FT = 8.47
        self.INITIAL_HEADING_DEG = 143.002
        self.TARGET_HEADING_DEG = self.INITIAL_HEADING_DEG
        self.INITIAL_VELOCITY_U = 33.76  # 20 knots/sec
        initial_conditions = {prp.initial_altitude_ft: self.INITIAL_ALTITUDE_FT,
                              prp.initial_u_fps: 0,
                              prp.initial_v_fps: 0,
                              prp.initial_w_fps: 0,
                              prp.initial_p_radps: 0,
                              prp.initial_latitude_geod_deg: self.INITIAL_LAT,
                              prp.initial_longitude_geoc_deg: self.INITIAL_LONG,
                              prp.initial_q_radps: 0,
                              prp.initial_r_radps: 0,
                              prp.initial_roc_fpm: 0,
                              prp.all_engine_running: -1,
                              prp.initial_heading_deg: self.INITIAL_HEADING_DEG,
                              prp.target_heading_deg: self.TARGET_HEADING_DEG,
                              prp.delta_heading:reduce_reflex_angle_deg(self.INITIAL_HEADING_DEG - self.TARGET_HEADING_DEG),
                              prp.gear_all_cmd: 1,
                              prp.target_altitude_ft: 300.0,   # irrelevant for this task, but needs to be initialized
                              self.nb_episodes: 0
                              }
        return initial_conditions

    def _update_custom_properties(self, sim: Simulation) -> None:
        self._decrement_steps_left(sim)

    def _decrement_steps_left(self, sim: Simulation):
        sim[self.steps_left] -= 1

    def _is_terminal(self, sim: Simulation, state: NamedTuple) -> bool:
        global takeoff_done
        terminal_step = sim[self.steps_left] <= 0
        sim[self.nb_episodes] += 1

        over_delta_heading = state.position_delta_heading_to_target_deg > 50

        takeoff_done = state.position_h_agl_ft > 500.0

        return terminal_step or over_delta_heading or takeoff_done

    def _get_reward(self, sim: Simulation, last_state: NamedTuple, action: NamedTuple, new_state: NamedTuple) -> float:
        '''
        Reward with delta and altitude heading directly in the input vector state.
        '''
        global takeoff_done
        task_reward = 0
        # inverse of the proportional absolute value of the minimal angle between the initial and current heading ...
        heading_r = 1.0 / math.sqrt((0.1 * math.fabs(last_state.position_delta_heading_to_target_deg) + 1))


        # reward nb episode

        if (last_state.position_h_agl_ft > 5+self.INITIAL_ALTITUDE_FT) and (last_state.position_h_agl_ft < 10+self.INITIAL_ALTITUDE_FT):
            if last_state.position_distance_from_start_mag_mt < 1900:   # max takeoff distance = 1828m
                task_reward += 1.0
        if (last_state.position_h_agl_ft > 35+self.INITIAL_ALTITUDE_FT) and (last_state.position_h_agl_ft < 45+self.INITIAL_ALTITUDE_FT):
            if last_state.velocities_vc_fps > 123:            # minimum V_2 in all conditions
                task_reward += 1.0
        if (last_state.position_h_agl_ft > 200+self.INITIAL_ALTITUDE_FT) and (last_state.position_h_agl_ft < 300+self.INITIAL_ALTITUDE_FT):
            if last_state.velocities_q_rad_sec > 0.0436:       # good pitch rate
                task_reward += 1.0

        return (heading_r  + task_reward)/4.0

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
