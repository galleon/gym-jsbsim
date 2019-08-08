import gym
import gym_jsbsim
from gym_jsbsim.catalogs.catalog import Catalog as prp
import time

	
def random_agent():
	env = gym.make("GymJsbsim-TaxiControlTask-v0")
	env.reset()
	done = False

	traj_string = []
	attitude_string = []

	start_time = time.time()
	while not done:
		#action = env.action_space.sample()
		action = [0,0,0,1]
		#start_time = time.time()
		state, reward, done, _ = env.step(action)
		#print("--- %s seconds test_agent.py env.step() ---",(time.time() - start_time))

		lon = env.sim.get_property_value(prp.position_long_gc_deg)
		lat = env.sim.get_property_value(prp.position_lat_geod_deg)
		alt = env.sim.get_property_value(prp.position_h_sl_ft)
		psi = env.sim.get_property_value(prp.attitude_psi_rad)
		theta = env.sim.get_property_value(prp.attitude_theta_rad)
		phi = env.sim.get_property_value(prp.attitude_phi_rad)
		sim_time = env.sim.get_property_value(prp.simulation_sim_time_sec)

		d1 = env.sim.get_property_value(prp.d1)
		a1 = env.sim.get_property_value(prp.a1)
		sd = env.sim.get_property_value(prp.shortest_dist)

		#print(sim_time, d1, a1, sd)

		traj_string += [sim_time, lon, lat, alt]
		attitude_string += [sim_time, psi, theta, phi]
		
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
