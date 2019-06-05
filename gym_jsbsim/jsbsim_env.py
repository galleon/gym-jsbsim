# -*- coding: utf-8 -*-

from gym_jsbsim.simulation import Simulation
import gym_jsbsim.simulation_parameters as param
import gym
import numpy as np



class JSBSimEnv(gym.Env):

    """

    A class wrapping the JSBSim flight dynamics module (FDM) for simulating

    aircraft as an RL environment conforming to the OpenAI Gym Env

    interface.
    
    An JsbSimEnv is instantiated with a Task that implements a specific

    aircraft control task with its own specific observation/action space and

    variables and agent_reward calculation.



    ATTRIBUTION: this class implements the OpenAI Gym Env API. Method

    docstrings have been adapted or copied from the OpenAI Gym source code.

    """
    metadata = {'render.modes': ['human', 'cvs']}
    
    
    def __init__(self, task, aircraft_name):
        """

        Constructor. Init some internal state, but JSBSimEnv.reset() must be

        called first before interacting with environment.



        :param task_type: the Task subclass for the task agent is to perform

        :param aircraft_name: the name of the JSBSim aircraft to be used.

        """
        
        self.sim: Simulation = None
        self.aircraft_name = aircraft_name
        self.task = task
        
        # set Space objects
        self.observation_space: gym.spaces.Box = self.task.get_observation_space()
        self.action_space: gym.spaces.Box = self.task.get_action_space()
        
        self.state = None
        
        
        
    def step(self, action = None):
        """

        Run one timestep of the environment's dynamics. When end of

        episode is reached, you are responsible for calling `reset()`

        to reset this environment's state.

        Accepts an action and returns a tuple (observation, reward, done, info).



        :param action: np.array, the agent's action, with same length as action variables.

        :return:

            state: agent's observation of the current environment

            reward: amount of reward returned after previous action

            done: whether the episode has ended, in which case further step() calls are undefined

            info: auxiliary information

        """
        if action != None:
            assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))

        self.state = self.make_step(action)
        
        reward, done, info = self.task.get_reward(self.state), self.task.is_terminal(self.state), {}

        return np.array(self.state), reward, done, info
    
    
    
    def make_step(self,action=None):
        """

        Calculates new state.


        :param action: array of floats, the agent's last action

        :return: observation: array, agent's observation of the environment state


        """
        #take actions
        if action != None:
            self.sim.set_property_values(self.task.get_action_var(),action)
        
        # run simulation
        for _ in range(param.agent_interaction_steps):
            self.sim.run()
            
        return self.get_observation()


    
    def reset(self,num_engine = -1):
        """

        Resets the state of the environment and returns an initial observation.

        :param num_engine: number of the engine to be run. All engines if -1.
        
        :return: array, the initial observation of the space.

        """
        
        init_conditions = self.task.get_initial_conditions()

        if self.sim:

            self.sim.initialise(init_conditions,num_engine)

        else:

            self.sim = Simulation(self.aircraft_name, init_conditions,num_engine)
            
        self.state = self.get_observation()
        
        return np.array(self.state)
    
    
    
       
    def render(self,mode='human'):
        """Renders the environment.

        The set of supported modes varies per environment. (And some

        environments do not support rendering at all.) By convention,

        if mode is:

        - human: print on the terminal
        - cvs: output to cvs files

        Note:

            Make sure that your class's metadata 'render.modes' key includes

              the list of supported modes. It's recommended to call super()

              in implementations to use the functionality of this method.



        :param mode: str, the mode to render with
        """
        
        if mode == 'human':
            pass
        elif mode == 'cvs':
            pass
        else:
            pass
        


    def seed(self,seed=None):
        """

        Sets the seed for this env's random number generator(s).

        Note:

            Some environments use multiple pseudorandom number generators.

            We want to capture all such seeds used in order to ensure that

            there aren't accidental correlations between multiple generators.

        Returns:

            list<bigint>: Returns the list of seeds used in this env's random

              number generators. The first value in the list should be the

              "main" seed, or the value which a reproducer should pass to

              'seed'. Often, the main seed equals the provided 'seed', but

              this won't be true if seed=None, for example.

        """
        return



    
    def close(self):
        """ Cleans up this environment's objects



        Environments automatically close() when garbage collected or when the

        program exits.

        """
        if self.sim:
            self.sim.close()
            
            


    
    def get_observation(self):
        """
        get state observation from sim.
         
        :return: NamedTuple, the first state observation of the episode
        
        """
        return self.sim.get_property_values(self.task.get_observation_var())
        
        
    def get_sim_time(self):
        """ Gets the simulation time from sim, a float. """
        
        return self.sim.get_sim_time()
        
    
        
        