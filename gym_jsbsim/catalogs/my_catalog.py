""" Defines custom properties not implemented in JSBSim """


from enum import Enum
from gym.spaces import Box, Discrete
from gym_jsbsim.catalogs.property import Property
from gym_jsbsim.catalogs.jsbsim_catalog import JsbsimCatalog
import gym_jsbsim.catalogs.utils as utils


class MyCatalog(Property, Enum):

    # position and attitude

    delta_heading = Property('position/delta-heading-to-target-deg', 'delta heading to target [deg]', -180, 180)
    delta_altitude = Property('position/delta-altitude-to-target-ft', 'delta altitude to target [ft]', -40000, 40000)

    # controls command

    throttle_cmd_dir = Property('fcs/throttle-cmd-dir', 'direction to move the throttle', 0, 2, Discrete)
    aileron_cmd_dir = Property('fcs/aileron-cmd-dir', 'direction to move the aileron', 0, 2, Discrete)
    elevator_cmd_dir = Property('fcs/elevator-cmd-dir', 'direction to move the elevator', 0, 2, Discrete)
    rudder_cmd_dir = Property('fcs/rudder-cmd-dir', 'direction to move the rudder', 0, 2, Discrete)

    # target conditions

    target_altitude_ft = Property('tc/h-sl-ft', 'target altitude MSL [ft]', JsbsimCatalog.position_h_sl_ft.min, JsbsimCatalog.position_h_sl_ft.max)
    target_heading_deg = Property('tc/target-heading-deg', 'target heading [deg]', JsbsimCatalog.attitude_psi_deg.min, JsbsimCatalog.attitude_psi_deg.max)
    target_time = Property('tc/target-time-sec', 'target time [sec]', 0)
    target_latitude_geod_deg = Property('tc/target-latitude-geod-deg', 'target geocentric latitude [deg]', -90, 90)
    target_longitude_geod_deg = Property('tc/target-longitude-geod-deg', 'target geocentric longitude [deg]', -180, 180)

    # following path

    radius_circle = Property('radius-circle', 'radius of the circle aim to compute target heading for ground procedure [m]', 2.0, 100.0)
    shortest_ac_to_path = Property('shortest-ac-to-path', 'shortest distance between aircraft and path [m]', 0.0, 100.0)
    closest_path_point_lat = Property('closest_path_point_lat', 'geocentric latitude [deg]', -90, 90)
    closest_path_point_lon = Property('closest_path_point_lon', 'geodesic longitude [deg]', -180, 180)
    intersectio_point_lat = Property('intersectio_point_lat', 'geocentric latitude [deg]', -90, 90)
    intersectio_point_lon = Property('intersectio_point_lon', 'geodesic longitude [deg]', -180, 180)

    # functions updating custom properties

    @classmethod
    def update_delta_altitude(cls, sim):
        value = sim.get_property_value(JsbsimCatalog.position_h_sl_ft) - sim.get_property_value(cls.target_altitude_ft)
        sim.set_property_value(cls.delta_altitude, value)

    @classmethod
    def update_delta_heading(cls, sim):
        value = utils.reduce_reflex_angle_deg(sim.get_property_value(JsbsimCatalog.attitude_psi_deg) - sim.get_property_value(cls.target_heading_deg))
        sim.set_property_value(cls.delta_heading, value)

    @classmethod
    def update_property_incr(cls, sim, discrete_prop, prop, incr=0.05):
        value = sim.get_property_value(discrete_prop)
        if value == 0:
            pass
        else :
            if value == 1:
                sim.set_property_value(prop, sim.get_property_value(prop) - incr)
            elif value == 2:
                sim.set_property_value(prop, sim.get_property_value(prop) + incr)
            sim.set_property_value(discrete_prop, 0)

    @classmethod
    def update_throttle_cmd_dir(cls, sim):
        cls.update_property_incr(sim, cls.throttle_cmd_dir, JsbsimCatalog.fcs_throttle_cmd_norm)

    @classmethod
    def update_aileron_cmd_dir(cls, sim):
        cls.update_property_incr(sim, cls.aileron_cmd_dir, JsbsimCatalog.fcs_aileron_cmd_norm)

    @classmethod
    def update_elevator_cmd_dir(cls, sim):
        cls.update_property_incr(sim, cls.elevator_cmd_dir, JsbsimCatalog.fcs_elevator_cmd_norm)

    @classmethod
    def update_rudder_cmd_dir(cls, sim):
        cls.update_property_incr(sim, cls.rudder_cmd_dir, JsbsimCatalog.fcs_rudder_cmd_norm)

    @classmethod
    def update_custom_properties(cls,sim,prop):
        update_custom_properties = {cls.delta_altitude : cls.update_delta_altitude,

                                    cls.delta_heading: cls.update_delta_heading,

                                    cls.throttle_cmd_dir: cls.update_throttle_cmd_dir,

                                    cls.aileron_cmd_dir: cls.update_aileron_cmd_dir,

                                    cls.elevator_cmd_dir : cls.update_elevator_cmd_dir,

                                    cls.rudder_cmd_dir : cls.update_rudder_cmd_dir
                                }
        if prop in update_custom_properties:
            update_custom_properties[prop](sim)
