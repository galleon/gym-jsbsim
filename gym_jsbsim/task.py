# -*- coding: utf-8 -*-

import numpy as np
import gym

class Task(object):
    """

        A class to create a task

        with its own observation variables, action variables, termination conditions and agent_reward function.

    """

    def __init__(self, observation_var, action_var, init_conditions, get_reward, is_terminal, output = None):
        """
        Constructor

        :param action_var: list of Property, the task's actions
        :param observation_var: list of Properties, the task's observation representation
        :param init_conditions: dict mapping properties to their initial values
        :param get_reward: function which calculates the agent's reward
        :param is_terminal: function which determine if the episode is terminated
        :param output: list of Property, properties to be output
        """


        self.action_var = action_var
        self.observation_var = observation_var
        self.get_reward = get_reward
        self.is_terminal = is_terminal
        self.init_conditions = init_conditions
        if output == None:
            self.output = observation_var
        else:
            self.ouput = output

        
    def get_observation_var(self):
        return self.observation_var
        
    
    def get_action_var(self):
        return self.action_var
    
    
    def get_initial_conditions(self):
        return self.init_conditions


    def get_props_output(self):
        return self.output


    def get_observation_space(self):
         """ Get the task's observation Space object """
         observation_lows = np.array([prop.min for prop in self.observation_var])

         observation_highs = np.array([prop.max for prop in self.observation_var])

         return gym.spaces.Box(low=observation_lows, high=observation_highs, dtype='float')
     
    
    def get_action_space(self):
         """ Get the task's observation Space object """
         action_lows = np.array([prop.min for prop in self.action_var])

         action_highs = np.array([prop.max for prop in self.action_var])

         return gym.spaces.Box(low=action_lows, high=action_highs, dtype='float')
     


    