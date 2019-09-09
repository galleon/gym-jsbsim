import gym
import gym_jsbsim
from gym_jsbsim.catalogs.catalog import Catalog as prp
import time


k2f = 1.68781

def random_agent():
	env = gym.make("GymJsbsim-TaxiapControlTask-v0")
	env.reset()
	done = False

	traj_string = []
	attitude_string = []

	start_time = time.time()
	while not done:
		
		lon = env.sim.get_property_value(prp.position_long_gc_deg)
		lat = env.sim.get_property_value(prp.position_lat_geod_deg)
		alt = env.sim.get_property_value(prp.position_h_sl_ft)
		psi = env.sim.get_property_value(prp.attitude_psi_rad)
		theta = env.sim.get_property_value(prp.attitude_theta_rad)
		phi = env.sim.get_property_value(prp.attitude_phi_rad)
		sim_time = env.sim.get_property_value(prp.simulation_sim_time_sec)



		#d1 = env.sim.get_property_value(prp.d1)
		a1 = env.sim.get_property_value(prp.a1)
		#sd = env.sim.get_property_value(prp.shortest_dist)

		

		traj_string += [sim_time, lon, lat, alt]
		attitude_string += [sim_time, psi, theta, phi]

		#action = env.action_space.sample()
		
		a1_norm = 0.2*min(max(a1, -1.0),1.0)#2.0 * (a1 - env.observation_space[1].low) / (env.observation_space[1].high - env.observation_space[1].low) - 1.0

		env.sim.set_property_value(prp.ap_vg_hold, 1)

		'''
		if sim_time < 30:
			env.sim.set_property_value(prp.target_vg, 10*k2f)
		elif sim_time < 60:
			env.sim.set_property_value(prp.target_vg, 20*k2f)
		elif sim_time < 90:
			env.sim.set_property_value(prp.target_vg, 0*k2f)
		elif sim_time < 120:
			env.sim.set_property_value(prp.target_vg, 7*k2f)
		elif sim_time < 150:
			env.sim.set_property_value(prp.target_vg, 200*k2f)
		'''
		if env.sim.get_property_value(prp.a1) > 15:
			env.sim.set_property_value(prp.target_vg, 7.0*k2f)
		else: # STRAIGHTLINE
			env.sim.set_property_value(prp.target_vg, 20.0*k2f)
		action = [a1_norm]

		#print(sim_time, "a1", a1, "-----", a1_norm, "vel", env.sim.get_property_value(prp.velocities_vc_fps), "vel_cmd", env.sim.get_property_value(prp.fcs_throttle_cmd_norm), "brake", env.sim.get_property_value(prp.fcs_left_brake_cmd_norm), env.sim.get_property_value(prp.fcs_center_brake_cmd_norm), env.sim.get_property_value(prp.fcs_right_brake_cmd_norm))
		#start_time = time.time()
		state, reward, done, _ = env.step(action)
		#print("--- %s seconds test_agent.py env.step() ---",(time.time() - start_time))

		#print(str(sim_time) + "," +  str(env.sim.get_property_value(prp.velocities_vc_fps)), a1_norm, str(env.sim.get_property_value(prp.target_vg)))
		#print("action", action)
		#print("state", state)
		#print("reward", reward)
	
	print("\n==========\nTRAJECTOIRE\n==========\n")
	for t in range(0, len(traj_string)-5, +4):
		print(str(traj_string[t]) + "," + str(traj_string[t+1]) + "," + str(traj_string[t+2]) + "," + str(traj_string[t+3]) + ",")
	print("\n==========\nATTITUDE\n==========\n")
	for t in range(0, len(attitude_string)-5, +4):
		print(str(attitude_string[t]) + "," + str(attitude_string[t+1]) + "," + str(attitude_string[t+2]) + "," + str(attitude_string[t+3]) + ",")
	print("--- %s seconds One Simulation ---",(time.time() - start_time))
if __name__ == "__main__":
	random_agent()
