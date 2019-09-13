import unittest
import sys
import math
sys.path.append('H:\\new_gym-jsbsim\\gym-jsbsim')
import os
os.environ['JSBSIM_ROOT_DIR'] = 'H:/jsbsim-code'
import gym_jsbsim
from gym_jsbsim import Catalog as c

class TestSimulation(unittest.TestCase):

    state_properties = [c.position_h_sl_ft,c.position_lat_geod_deg,c.position_long_gc_deg,
                        c.velocities_u_fps,c.velocities_v_fps,c.velocities_w_fps,
                        c.velocities_p_rad_sec, c.velocities_q_rad_sec,c.velocities_r_rad_sec]

    error_max = 0.001

    tmax = 3600 # seconds

    def setUp(self):
        self.env = gym_jsbsim.make('GymJsbsim-HeadingControlTask-v0')
        self.env.reset()

    def tearDown(self):
        self.env.close()

    def test_get_state_set_state_no_action(self):
        get_state = False
        count_actions = 0
        state = None
        t = 0
        # first flight
        while t <= self.tmax and self.env.sim.get_property_value(c.position_h_sl_ft) >= 100:
            if t >= 5 and get_state is False:
                # get_state after 5 seconds of simulation
                state = self.env.get_full_state()
                get_state = True
            if get_state:
                count_actions +=1
            self.env.step()
            t = self.env.get_sim_time()
        end_state_1 = self.env.get_full_state()

        # second flight
        self.env.set_full_state(state)
        for _ in range(count_actions):
            self.env.step()
        end_state_2 = self.env.get_full_state()

        # compute error between two end states
        for prop in self.state_properties:
            p1 = end_state_1[prop]
            p2 = end_state_2[prop]
            if p1 == p2:
                error = 0
            else:
                error = math.fabs(p2 - p1)/max(math.fabs(p1),math.fabs(p2))
            self.assertLess(error,self.error_max,'The two simulations have diverged')



