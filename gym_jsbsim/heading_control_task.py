import math
import random
import gym_jsbsim.properties as prp
from gym_jsbsim import utils
from gym_jsbsim.simulation import Simulation
from gym_jsbsim.base_flight_task import BaseFlightTask
from gym_jsbsim.properties import BoundedProperty, Property
from gym_jsbsim.aircraft import Aircraft
from typing import Dict, Tuple, Sequence, NamedTuple


class HeadingControlTask(BaseFlightTask):
    """
    A task in which the agent must perform steady, level flight maintaining its
    initial heading.
    """
    THROTTLE_CMD = 0.8
    MIXTURE_CMD = 0.8
    INITIAL_HEADING_DEG = 270
    INITIAL_ALTITUDE_FT = 5000
    DEFAULT_EPISODE_TIME_S = 60.
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
        self.state_variables = (prp.pitch_rad, prp.roll_rad, prp.sideslip_deg, prp.v_north_fps, prp.v_east_fps, prp.altitude_sl_ft, # minimal state variables for the task
                                prp.v_down_fps, prp.p_radps, prp.q_radps, prp.r_radps) # additional state variables used for reward shaping
        self.action_variables = (prp.aileron_cmd, prp.elevator_cmd, prp.rudder_cmd)
        super().__init__(debug)

    def get_initial_conditions(self) -> Dict[Property, float]:
        initial_conditions = {prp.initial_altitude_ft: self.INITIAL_ALTITUDE_FT,
                              prp.initial_u_fps: self.aircraft.get_cruise_speed_fps(),
                              prp.initial_v_fps: 0,
                              prp.initial_w_fps: 0,
                              prp.initial_p_radps: 0,
                              prp.initial_q_radps: 0,
                              prp.initial_r_radps: 0,
                              prp.initial_roc_fpm: 0,
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
        return terminal_step or self._altitude_out_of_bounds(sim, state)
    
    def _get_reward(self, sim: Simulation, last_state: NamedTuple, action: NamedTuple, new_state: NamedTuple) -> float:
        # Get negative reward proportional to normalised heading and altitude errors
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
