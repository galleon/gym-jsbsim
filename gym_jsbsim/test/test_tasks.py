import unittest
import numpy as np
from gym_jsbsim.environment import JsbSimEnv
from gym_jsbsim.simulation import Simulation
from gym_jsbsim.tasks import SteadyLevelFlightTask, SimplePitchControlTask
from gym_jsbsim.test import SimStub


class TestSteadyLevelFlightTask(unittest.TestCase):
    def setUp(self):
        self.task = SteadyLevelFlightTask()

    def test_reward_calc(self):
        dummy_sim = SimStub({'accelerations/udot-ft_sec2': 1,
                             'accelerations/vdot-ft_sec2': 1,
                             'accelerations/wdot-ft_sec2': 1,
                             'accelerations/pdot-rad_sec2': -2,
                             'accelerations/qdot-rad_sec2': 2,
                             'accelerations/rdot-rad_sec2': 2,
                             'velocities/v-down-fps': 2,
                             'attitude/roll-rad': -2,
        })
        expected_reward = -sum(abs(val) for val in dummy_sim.values())
        dummy_sim['position/h-sl-ft'] = 3000  # above minimum
        self.assertAlmostEqual(expected_reward, self.task._calculate_reward(dummy_sim))

        # test again with low altitude
        dummy_sim['position/h-sl-ft'] = 0
        expected_reward += self.task.TOO_LOW_REWARD
        self.assertAlmostEqual(expected_reward, self.task._calculate_reward(dummy_sim))

    def test_is_done_false(self):
        dummy_sim = SimStub({'simulation/sim-time-sec': 1,
                             'position/h-sl-ft': 5000})
        self.assertFalse(self.task._is_done(dummy_sim))

    def test_is_done_true_too_low(self):
        dummy_sim = SimStub({'simulation/sim-time-sec': 0,
                             'position/h-sl-ft': 0})
        self.assertTrue(self.task._is_done(dummy_sim))

    def test_is_done_true_time_out(self):
        dummy_sim = SimStub({'simulation/sim-time-sec': 9999,
                             'position/h-sl-ft': 5000})
        self.assertTrue(self.task._is_done(dummy_sim))

    def test_task_first_observation(self):
        props_value = 5
        prop_value_pairs = [(prop['name'], props_value) for prop in self.task.state_variables]
        dummy_sim = SimStub(prop_value_pairs)
        state = self.task.observe_first_state(dummy_sim)

        number_of_state_vars = len(self.task.state_variables)
        expected_state = np.full(shape=(number_of_state_vars,), fill_value=5, dtype=int)

        self.assertIsInstance(state, np.ndarray)
        np.testing.assert_array_equal(expected_state, state)

        # check throttle and mixture set
        self.assertAlmostEqual(self.task.THROTTLE_CMD, dummy_sim['fcs/throttle-cmd-norm'])
        self.assertAlmostEqual(self.task.MIXTURE_CMD, dummy_sim['fcs/mixture-cmd-norm'])

    def test_get_initial_conditions(self):
        ics = self.task.get_initial_conditions()

        self.assertIsInstance(ics, dict)
        for prop_name, value in self.task.base_initial_conditions.items():
            self.assertAlmostEqual(value, ics[prop_name])

        steady_level_task_ic_properties = ['ic/psi-true-deg',
                                           'ic/vt-kts',
                                           'ic/phi-deg',
                                           'ic/theta-deg'
                                           ]
        for prop_name in steady_level_task_ic_properties:
            self.assertIn(prop_name, ics.keys(),
                          msg='expected SteadyLevelFlightTask to set value for'
                              f'property {prop_name} but not found in ICs')

    def test_engines_init_running(self):
        env = JsbSimEnv(task_type=SteadyLevelFlightTask)

        # test assumption that property 'propulsion/engine/set-running'
        #   is zero prior to engine start!
        check_sim = Simulation(init_conditions={})
        engine_off_value = 0.0
        self.assertAlmostEqual(engine_off_value,
                               check_sim['propulsion/engine/set-running'])
        check_sim.close()

        # check engines on once env has been reset
        _ = env.reset()
        # now check
        engine_running_value = 1.0
        self.assertAlmostEqual(engine_running_value,
                               env.sim['propulsion/engine/set-running'])

    def test_shaped_reward(self):
        low_reward_state_sim = SimStub.make_valid_state_stub(self.task)
        high_reward_state_sim = SimStub.make_valid_state_stub(self.task)

        # make one sim near the target values, and one relatively far away
        for prop, ideal_value in SteadyLevelFlightTask.TARGET_VALUES:
            low_reward_state_sim[prop] = ideal_value + 5
            high_reward_state_sim[prop] = ideal_value + 0.05
        # make sure altitude hasn't randomly been set below minimum!
        low_reward_state_sim['position/h-sl-ft'] = SteadyLevelFlightTask.MIN_ALT_FT + 1000
        high_reward_state_sim['position/h-sl-ft'] = SteadyLevelFlightTask.MIN_ALT_FT + 1000

        # suppose we start in the low reward state then transition to the high reward state
        self.task.observe_first_state(low_reward_state_sim)
        dummy_action = self.task.get_action_space().sample()
        _, first_reward, _, _ = self.task.task_step(high_reward_state_sim, dummy_action, 1)
        # shaped reward should be positive
        self.assertGreater(first_reward, 0)

        # now suppose we transition in the next step back to the low reward state
        _, second_reward, _, _ = self.task.task_step(low_reward_state_sim, dummy_action, 1)
        # shaped reward should be negative, and equal to the negative first_reward
        self.assertLess(second_reward, 0)
        self.assertAlmostEqual(-1 * first_reward, second_reward)

        # and if we remain in the same low-reward state we should receive 0 shaped reward
        _, third_reward, _, _ = self.task.task_step(low_reward_state_sim, dummy_action, 1)
        self.assertAlmostEqual(0, third_reward)


class TestSimplePitchControlTask(unittest.TestCase):
    def setUp(self):
        self.task = SimplePitchControlTask()

    def test_input_initial_controls_expected_values(self):
        sim = SimStub()
        self.task._input_initial_controls(sim)

        throttle_cmd = sim['fcs/throttle-cmd-norm']
        self.assertAlmostEqual(SimplePitchControlTask.THROTTLE_CMD_CRUISE, throttle_cmd)
        mixture_cmd = sim['fcs/mixture-cmd-norm']
        self.assertAlmostEqual(SimplePitchControlTask.MIXTURE_CMD, mixture_cmd)
        is_wing_level_autopilot = sim['ap/autopilot-roll-on'] == 1
        self.assertTrue(is_wing_level_autopilot)

    def test_calculate_reward(self):
        acceptably_high_altitude = 5000
        high_elevation_rate = 0.01
        low_elevation_rate = 50

        high_reward_sim = SimStub()
        high_reward_sim['position/h-sl-ft'] = acceptably_high_altitude
        high_reward_sim['velocities/h-dot-fps'] = high_elevation_rate
        low_reward_sim = SimStub()
        low_reward_sim['position/h-sl-ft'] = acceptably_high_altitude
        low_reward_sim['velocities/h-dot-fps'] = low_elevation_rate

        expected_high_reward = - abs(high_elevation_rate)
        expected_low_reward = - abs(low_elevation_rate)
        high_reward = self.task._calculate_reward(high_reward_sim)
        low_reward = self.task._calculate_reward(low_reward_sim)

        self.assertAlmostEqual(expected_high_reward, high_reward)
        self.assertAlmostEqual(expected_low_reward, low_reward)
        self.assertGreater(high_reward, low_reward)



