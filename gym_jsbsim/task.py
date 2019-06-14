import numpy as np
import gym
from gym.spaces import Box, Discrete

class Task:
    """

        A class to create a task with its own observation variables,

        action variables, termination conditions and agent_reward function.

    """

    def __init__(self, observation_var, action_var, init_conditions, get_reward, is_terminal, output=None):
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

        #set default output to observation_var
        if output is None:
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
        """
        Get the task's observation Space object

        :return : spaces.Tuple composed by spaces of each property.
        """

        space_tuple = ()

        for prop in self.observation_var:
            if prop.spaces is Box:
                space_tuple += (Box(low=np.array([prop.min]), high=np.array([prop.max]), dtype='float'),)
            elif prop.spaces is Discrete:
                space_tuple += (Discrete(prop.max - prop.min + 1),)
        return gym.spaces.Tuple(space_tuple)


    def get_action_space(self):
        """
        Get the task's action Space object

        :return : spaces.Tuple composed by spaces of each property.
        """
        space_tuple = ()

        for prop in self.action_var:
            if prop.spaces is Box:
                space_tuple += (Box(low=np.array([prop.min]), high=np.array([prop.max]), dtype='float'),)
            elif prop.spaces is Discrete:
                space_tuple += (Discrete(prop.max - prop.min + 1),)
        return gym.spaces.Tuple(space_tuple)
