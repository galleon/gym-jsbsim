import gym
import numpy as np
from gym_jsbsim.simulation import Simulation


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

    metadata = {'render.modes': ['human', 'csv']}


    def __init__(self, task, aircraft_name):
        """

        Constructor. Init some internal state, but JSBSimEnv.reset() must be

        called first before interacting with environment.



        :param task the Task for the task agent is to perform

        :param aircraft_name: the name of the JSBSim aircraft to be used.

        """

        self.sim = None
        self.aircraft_name = aircraft_name
        self.task = task

        self.observation_space = self.task.get_observation_space() #None
        self.action_space = self.task.get_action_space() #None

        self.state = None


    def step(self, action=None):
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

        if action is not None:
            if not len(action) == len(self.action_space):
                raise ValueError(
                    'mismatch between action and action space size')

        self.state = self.make_step(action)

        reward, done, info = self.task.get_reward(self.state, self.sim), self.task.is_terminal(self.state, self.sim), {}

        return np.array(self.state), reward, done, info


    def make_step(self, action=None):
        """

        Calculates new state.


        :param action: array of floats, the agent's last action

        :return: observation: array, agent's observation of the environment state


        """
        # take actions
        if action is not None:
            self.sim.set_property_values(self.task.get_action_var(), action)

        # run simulation
        self.sim.run()

        return self.get_observation()


    def reset(self):
        """

        Resets the state of the environment and returns an initial observation.

        :return: array, the initial observation of the space.

        """
        if self.sim:
            self.sim.initialise(init_conditions=self.task.init_conditions, jsbsim_freq = self.task.jsbsim_freq, agent_interaction_steps = self.task.agent_interaction_steps)

        else :
            self.sim = Simulation(aircraft_name = self.aircraft_name, init_conditions=self.task.init_conditions, jsbsim_freq = self.task.jsbsim_freq, agent_interaction_steps = self.task.agent_interaction_steps)

        self.state = self.get_observation()

        self.observation_space = self.task.get_observation_space()

        self.action_space = self.task.get_action_space()

        return np.array(self.state)


    def define_state(self,states):
        self.task.observation_var = states

    def define_actions(self,actions):
        self.task.action_var = actions

    def define_reward(self,get_reward):
        self.task.get_reward = get_reward

    def define_init_conditions(self,init_conditions):
        self.task.init_conditions=init_conditions

    def define_jsbsim_freq(self, freq = 60):
        self.task.jsbsim_freq = freq

    def define_agent_interaction_steps(self,steps = 5):
        self.task.agent_interaction_steps = steps


    def render(self, mode='human'):
        """Renders the environment.

        The set of supported modes varies per environment. (And some

        environments do not support rendering at all.) By convention,

        if mode is:

        - human: print on the terminal
        - csv: output to cvs files

        Note:

            Make sure that your class's metadata 'render.modes' key includes

              the list of supported modes. It's recommended to call super()

              in implementations to use the functionality of this method.



        :param mode: str, the mode to render with
        """
        output = self.task.get_props_output()

        if mode == 'human':
            pass
        elif mode == 'csv':
            pass
        else:
            pass


    def seed(self, seed=None):
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
        return self.sim.get_property_values(self.task.get_observation_space())


    def get_sim_time(self):
        """ Gets the simulation time from sim, a float. """

        return self.sim.get_sim_time()

    def get_full_state(self):
        return self.sim.get_full_state()

    def set_full_state(self,state):
        init_conditions = self.sim.get_init_conditions_from_state(state)
        self.define_init_conditions(init_conditions)
        self.reset()
