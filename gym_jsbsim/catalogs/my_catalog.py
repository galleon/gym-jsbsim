""" Defines custom properties not implemented in JSBSim """


from enum import Enum
from gym.spaces import Box, Discrete
from gym_jsbsim.catalogs.property import Property
from gym_jsbsim.catalogs.jsbsim_catalog import JsbsimCatalog
from gym_jsbsim.envs.taxi_utils import *
from gym_jsbsim.catalogs import utils
import time
import os.path
from numpy.linalg import norm

my_path = os.path.abspath(os.path.dirname(__file__))
amdb_path = os.path.join(my_path, "../../amdb") 
taxiPath = taxi_path(ambd_folder_path=amdb_path, number_of_points_to_use=4)

#taxi_freq_state = 30



class MyCatalog(Property, Enum):


    def update_delta_altitude(sim):
        value = sim.get_property_value(MyCatalog.target_altitude_ft) - sim.get_property_value(JsbsimCatalog.position_h_sl_ft)
        sim.set_property_value(MyCatalog.delta_altitude, value)

    def update_delta_heading(sim):
        value = utils.reduce_reflex_angle_deg(
            sim.get_property_value(MyCatalog.target_heading_deg) - sim.get_property_value(JsbsimCatalog.attitude_psi_deg))
        sim.set_property_value(MyCatalog.delta_heading, value)

    @staticmethod
    def update_property_incr(sim, discrete_prop, prop, incr_prop):
        value = sim.get_property_value(discrete_prop)
        if value == 0:
            pass
        else:
            if value == 1:
                sim.set_property_value(prop, sim.get_property_value(prop) - sim.get_property_value(incr_prop))
            elif value == 2:
                sim.set_property_value(prop, sim.get_property_value(prop) + sim.get_property_value(incr_prop))
            sim.set_property_value(discrete_prop, 0)

    def update_throttle_cmd_dir(sim):
        MyCatalog.update_property_incr(sim, MyCatalog.throttle_cmd_dir, JsbsimCatalog.fcs_throttle_cmd_norm, MyCatalog.incr_throttle)


    def update_aileron_cmd_dir(sim):
        MyCatalog.update_property_incr(sim, MyCatalog.aileron_cmd_dir, JsbsimCatalog.fcs_aileron_cmd_norm, MyCatalog.incr_aileron)


    def update_elevator_cmd_dir(sim):
        MyCatalog.update_property_incr(sim, MyCatalog.elevator_cmd_dir, JsbsimCatalog.fcs_elevator_cmd_norm, MyCatalog.incr_elevator)


    def update_rudder_cmd_dir(sim):
        MyCatalog.update_property_incr(sim, MyCatalog.rudder_cmd_dir, JsbsimCatalog.fcs_rudder_cmd_norm, MyCatalog.incr_rudder)


    def update_detect_extreme_state(sim):
        extreme_velocity = sim.get_property_value(JsbsimCatalog.velocities_eci_velocity_mag_fps) >= 1e10
        extreme_rotation = norm(sim.get_property_values([JsbsimCatalog.velocities_p_rad_sec,JsbsimCatalog.velocities_q_rad_sec,JsbsimCatalog.velocities_r_rad_sec])) >= 1000
        extreme_altitude = sim.get_property_value(JsbsimCatalog.position_h_sl_ft) >= 1e10
        sim.set_property_value(MyCatalog.detect_extreme_state, extreme_altitude or extreme_rotation or extreme_velocity)


    def update_da(sim):
        #print("sim.get_property_value(nb_step), taxi_freq_state", sim.get_property_value(nb_step), taxi_freq_state)
        #print(taxi_freq_state)
        #if (sim.get_property_value(MyCatalog.nb_step)%sim.get_property_value(MyCatalog.taxi_freq_state)==1):
        #start_time = time.time()
        df = taxiPath.update_path((sim.get_property_value(JsbsimCatalog.position_long_gc_deg), sim.get_property_value(JsbsimCatalog.position_lat_geod_deg)),sim.get_property_value(JsbsimCatalog.attitude_psi_deg))
        #print("--- %s seconds ---",(time.time() - start_time))

        dist = taxiPath.shortest_dist
        #print("shortest_dist2 in meters", dist)
        sim.set_property_value(MyCatalog.shortest_dist, dist)
        #print(sim.get_property_value(MyCatalog.shortest_dist))

        for i in range(1,len(df)+1):
            sim.set_property_value(MyCatalog["d"+str(i)], df[i-1][1])
            sim.set_property_value(MyCatalog["a"+str(i)], df[i-1][2])#]utils.reduce_reflex_angle_deg(df[i-1][2] - sim.get_property_value(JsbsimCatalog.attitude_psi_deg)))
            #sim.set_property_value(MyCatalog["a"+str(i)], df[i][2])

        #d = sim.get_property_value(MyCatalog.d1)
        #print(d)
        #print("a",sim.get_property_value(MyCatalog.a1))
        #print("d",sim.get_property_value(MyCatalog.d1), sim.get_property_value(MyCatalog.d2), sim.get_property_value(MyCatalog.d3), sim.get_property_value(MyCatalog.d4), sim.get_property_value(MyCatalog.d5), sim.get_property_value(MyCatalog.d6), sim.get_property_value(MyCatalog.d7), sim.get_property_value(MyCatalog.d8))
        #print("a",sim.get_property_value(MyCatalog.a1), sim.get_property_value(MyCatalog.a2), sim.get_property_value(MyCatalog.a3), sim.get_property_value(MyCatalog.a4), sim.get_property_value(MyCatalog.a5), sim.get_property_value(MyCatalog.a6), sim.get_property_value(MyCatalog.a7), sim.get_property_value(MyCatalog.a8))
        #print(sim.get_property_value(MyCatalog.shortest_dist))
        #print("")
        

        #sim.set_property_value(MyCatalog.nb_step, int(sim.get_property_value(MyCatalog.nb_step))+1)


    # position and attitude

    delta_altitude = Property('position/delta-altitude-to-target-ft', 'delta altitude to target [ft]', -40000, 40000, access='R', update=update_delta_altitude)
    delta_heading = Property('position/delta-heading-to-target-deg', 'delta heading to target [deg]', -180, 180, access='R', update=update_delta_heading)

    # controls command

    throttle_cmd_dir = Property('fcs/throttle-cmd-dir', 'direction to move the throttle', 0, 2, spaces = Discrete, access = 'W',update = update_throttle_cmd_dir)
    incr_throttle = Property('fcs/incr-throttle','incrementation throttle', 0, 1)
    aileron_cmd_dir = Property('fcs/aileron-cmd-dir', 'direction to move the aileron', 0, 2, spaces = Discrete, access = 'W',update = update_aileron_cmd_dir)
    incr_aileron = Property('fcs/incr-aileron', 'incrementation aileron', 0, 1)
    elevator_cmd_dir = Property('fcs/elevator-cmd-dir', 'direction to move the elevator', 0, 2, spaces = Discrete, access = 'W', update = update_elevator_cmd_dir)
    incr_elevator = Property('fcs/incr-elevator', 'incrementation elevator', 0, 1)
    rudder_cmd_dir = Property('fcs/rudder-cmd-dir', 'direction to move the rudder', 0, 2, spaces = Discrete, access = 'W',update = update_rudder_cmd_dir)
    incr_rudder = Property('fcs/incr-rudder', 'incrementation rudder', 0, 1)

    # detect functions

    detect_extreme_state = Property('detect/extreme-state','detect extreme rotation, velocity and altitude',0,1,spaces = Discrete,access = 'R', update = update_detect_extreme_state)

    # target conditions

    target_altitude_ft = Property('tc/h-sl-ft', 'target altitude MSL [ft]', JsbsimCatalog.position_h_sl_ft.min, JsbsimCatalog.position_h_sl_ft.max)
    target_heading_deg = Property('tc/target-heading-deg', 'target heading [deg]', JsbsimCatalog.attitude_psi_deg.min, JsbsimCatalog.attitude_psi_deg.max)
    target_vg = Property('tc/target-vg', 'target ground velocity [ft/s]')
    target_time = Property('tc/target-time-sec', 'target time [sec]', 0)
    target_latitude_geod_deg = Property('tc/target-latitude-geod-deg', 'target geocentric latitude [deg]', -90, 90)
    target_longitude_geod_deg = Property('tc/target-longitude-geod-deg', 'target geocentric longitude [deg]', -180, 180)

    # following path

    steady_flight = Property('steady_flight', 'steady flight mode', 0, 1000000)
    turn_flight = Property('turn_flight', 'turn flight mode', 0, 1)

    #dist_heading_centerline_matrix = Property('dist_heading_centerline_matrix', 'dist_heading_centerline_matrix', '2D matrix with dist,angle of the next point from the aircraft to 1km (max 10 points)', [0, -45, 0, -45, 0, -45, 0, -45, 0, -45, 0, -45, 0, -45, 0, -45], [1000, 45, 1000, 45, 1000, 45, 1000, 45, 1000, 45, 1000, 45, 1000, 45, 1000, 45])
    d1 = Property('d1', 'd1', 0, 1000, access = 'R', update = update_da)
    d2 = Property('d2', 'd2', 0, 1000, access = 'R')
    d3 = Property('d3', 'd3', 0, 1000, access = 'R')
    d4 = Property('d4', 'd4', 0, 1000, access = 'R')
    d5 = Property('d5', 'd5', 0, 1000, access = 'R')
    d6 = Property('d6', 'd6', 0, 1000, access = 'R')
    d7 = Property('d7', 'd7', 0, 1000, access = 'R')
    d8 = Property('d8', 'd8', 0, 1000, access = 'R')
    a1 = Property('a1', 'a1', -180, 180, access = 'R')
    a2 = Property('a2', 'a2', -180, 180, access = 'R')
    a3 = Property('a3', 'a3', -180, 180, access = 'R')
    a4 = Property('a4', 'a4', -180, 180, access = 'R')
    a5 = Property('a5', 'a5', -180, 180, access = 'R')
    a6 = Property('a6', 'a6', -180, 180, access = 'R')
    a7 = Property('a7', 'a7', -180, 180, access = 'R')
    a8 = Property('a8', 'a8', -180, 180, access = 'R')

    shortest_dist = Property('shortest_dist', 'shortest distance between aircraft and path [m]', 0.0, 1000.0, access = 'R')
    #taxi_freq_state = Property('taxi-freq-state','frequence to update taxi state',0)
    #nb_step = Property('nb_step', 'shortest distance between aircraft and path [m]', access = 'R')
