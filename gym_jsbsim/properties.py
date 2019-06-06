# -*- coding: utf-8 -*-

import math
from collections import namedtuple


class Property(namedtuple('Property', ['name','name_jsbsim', 'description', 'min', 'max'],defaults = (None,None,None,float('-inf'),float('+inf')))):

    #define operations for custom properties expressions
    def __sub__(self, other):
        def sub(sim):
            return sim.get_property_value(self) - sim.get_property_value(other)

        return sub

    def __rsub__(self, other):
        def rsub(sim):
            return other(sim) - sim.get_property_value(self)

        return rsub

    def __add__(self, other):
        def add(sim):
            return sim.get_property_value(self) + sim.get_property_value(other)

        return add

    def __radd__(self, other):
        def radd(sim):
            return sim.get_property_value(self) + other(sim)

        return radd

    def __truediv__(self, other):
        def div(sim):
            return sim.get_property_value(self)/sim.get_property_value(other)

        return div

    def __rtruediv__(self, other):
        def rdiv(sim):
            return other(sim)/sim.get_property_value(self)

        return rdiv

    def __mul__(self, other):
        def mul(sim):
            return sim.get_property_value(self)*sim.get_property_value(other)

        return mul

    def __rmul__(self,other):
        def rmul(sim):
            return other(sim)*sim.get_property_value(other)

        return rmul



# position and attitude

altitude_sl_ft = Property('altitude_sl_ft','position/h-sl-ft', 'altitude above mean sea level [ft]', -1400, 85000)

pitch_rad = Property('pitch_rad','attitude/pitch-rad', 'pitch [rad]', -0.5 * math.pi, 0.5 * math.pi)

roll_rad = Property('roll_rad','attitude/roll-rad', 'roll [rad]', -math.pi, math.pi)

heading_deg = Property('heading_deg','attitude/psi-deg', 'heading [deg]', 0, 360)

sideslip_deg = Property('sideslip_deg','aero/beta-deg', 'sideslip [deg]', -180, +180)

lat_geod_deg = Property('lat_geod_deg','position/lat-geod-deg', 'geocentric latitude [deg]', -90, 90)

lng_geoc_deg = Property('lng_geoc_deg','position/long-gc-deg', 'geodesic longitude [deg]', -180, 180)

dist_travel_m = Property('dist_travel_m','position/distance-from-start-mag-mt', 'distance travelled from starting position [m]')

delta_heading = Property('delta_heading','position/delta-heading-to-target-deg', 'delta heading to target [deg]', -180, 180)

delta_altitude = Property('delta_altitude','position/delta-altitude-to-target-ft', 'delta altitude to target [ft]', -40000, 40000)



# velocities

u_fps = Property('u_fps','velocities/u-fps', 'body frame x-axis velocity [ft/s]', -2200, 2200)

v_fps = Property('v_fps','velocities/v-fps', 'body frame y-axis velocity [ft/s]', -2200, 2200)

w_fps = Property('w_fps','velocities/w-fps', 'body frame z-axis velocity [ft/s]', -2200, 2200)

v_north_fps = Property('v_north_fps','velocities/v-north-fps', 'velocity true north [ft/s]')

v_east_fps = Property('v_east_fps','velocities/v-east-fps', 'velocity east [ft/s]')

v_down_fps = Property('v_down_fps','velocities/v-down-fps', 'velocity downwards [ft/s]')

p_radps = Property('p_radps','velocities/p-rad_sec', 'roll rate [rad/s]', -2 * math.pi, 2 * math.pi)

q_radps = Property('q_radps','velocities/q-rad_sec', 'pitch rate [rad/s]', -2 * math.pi, 2 * math.pi)

r_radps = Property('r_radps','velocities/r-rad_sec', 'yaw rate [rad/s]', -2 * math.pi, 2 * math.pi)

altitude_rate_fps = Property('altitude_rate_fps','velocities/h-dot-fps', 'rate of altitude change [ft/s]')

phi_dot = Property('phi_dot','velocities/phidot-rad_sec', 'rad/s', -2 * math.pi, 2 * math.pi)

theta_dot = Property('theta_dot','velocities/thetadot-rad_sec', 'rad/s', -2 * math.pi, 2 * math.pi)

v_air = Property('v_air','velocities/vc-fps', 'airspeed in knots', 0, 4400)



# Acceleration

p_dot = Property('p_dot','accelerations/pdot-rad_sec2', 'rad/s²',  -(8/180) * math.pi, (8/180) * math.pi)

q_dot = Property('q_dot','accelerations/qdot-rad_sec2', 'rad/s²',  -(8/180) * math.pi, (8/180) * math.pi)

r_dot = Property('r_dot','accelerations/rdot-rad_sec2', 'rad/s²',  -(8/180) * math.pi, (8/180) * math.pi)

v_dot = Property('v_dot','accelerations/vdot-ft_sec2', 'ft/s²', -4.0, 4.0)

w_dot = Property('w_dot','accelerations/wdot-ft_sec2', 'ft/s²', -4.0, 4.0)

u_dot = Property('u_dot','accelerations/wdot-ft_sec2', 'ft/s²', -4.0, 4.0)



# accelerations

n_pilot_x = Property('n_pilot_x','accelerations/n-pilot-x-norm', 'pilot body x-axis acceleration, normalised')

n_pilot_y = Property('n_pilot_y','accelerations/n-pilot-y-norm', 'pilot body y-axis acceleration, normalised')

n_pilot_z = Property('n_pilot_z','accelerations/n-pilot-z-norm', 'pilot body z-axis acceleration, normalised')



# controls state

aileron_left = Property('aileron_left','fcs/left-aileron-pos-norm', 'left aileron position, normalised', -1, 1)

aileron_right = Property('aileron_right','fcs/right-aileron-pos-norm', 'right aileron position, normalised', -1, 1)

elevator = Property('elevator','fcs/elevator-pos-norm', 'elevator position, normalised', -1, 1)

rudder = Property('rudder','fcs/rudder-pos-norm', 'rudder position, normalised', -1, 1)

flap = Property('flap','fcs/flap-pos-norm', 'flap position, normalised', 0, 1)

speedbrake = Property('speedbrake','fcs/speedbrake-pos-norm', 'speedbrake position, normalised', 0, 1)

throttle = Property('throttle','fcs/throttle-pos-norm', 'throttle position, normalised', 0, 1)

throttle_1 = Property('throttle_1','fcs/throttle-pos-norm[1]', 'throttle position normalised', 0, 1)

gear = Property('gear','gear/gear-pos-norm', 'landing gear position, normalised', 0, 1)





# engines

engine_running = Property('engine_running','propulsion/engine/set-running', 'engine running (0/1 bool)')

all_engine_running = Property('all_engine_running','propulsion/set-running', 'set engine running (-1 for all engines)')

engine_thrust_lbs = Property('engine_thrust_lbs','propulsion/engine/thrust-lbs', 'engine thrust [lb]')

fuel_contents_lbs_0 = Property('fuel_contents_lbs_0','propulsion/tank[0]/contents-lbs', 'Fuel content in selected tank')

fuel_contents_lbs_1 = Property('fuel_contents_lbs_1','propulsion/tank[1]/contents-lbs', 'Fuel content in selected tank')



# controls command

aileron_cmd = Property('aileron_cmd','fcs/aileron-cmd-norm', 'aileron commanded position, normalised', -1., 1.)

elevator_cmd = Property('elevator_cmd','fcs/elevator-cmd-norm', 'elevator commanded position, normalised', -1., 1.)

rudder_cmd = Property('rudder_cmd','fcs/rudder-cmd-norm', 'rudder commanded position, normalised', -1., 1.)

throttle_cmd = Property('throttle_cmd','fcs/throttle-cmd-norm', 'throttle commanded position, normalised', 0., 1.)

mixture_cmd = Property('mixture_cmd','fcs/mixture-cmd-norm', 'engine mixture setting, normalised', 0., 1.)

throttle_cmd_1 = Property('throttle_1_cmd','fcs/throttle-cmd-norm[1]', 'throttle 1 commanded position, normalised', 0., 1.)

mixture_cmd_1 = Property('mixture_1_cmd','fcs/mixture-cmd-norm[1]', 'engine mixture 1 setting, normalised', 0., 1.)

gear_all_cmd = Property('gear_all_cmd','gear/gear-cmd-norm', 'all landing gear commanded position, normalised', 0., 1.)

left_brake_cmd_norm = Property('left_brake_cmd_norm','fcs/left-brake-cmd-norm', 'Left brake command(normalized)', 0., 1.)

right_brake_cmd_norm = Property('right_brake_cmd_norm','fcs/right-brake-cmd-norm', 'Right brake command(normalized)', 0., 1.)

steer_cmd_norm = Property('steer_cmd_norm','fcs/steer-cmd-norm', 'Steer command(normalized)', -1., 1.)



# trim commands

pitch_trim = Property('pitch_trim','fcs/pitch-trim-cmd-norm', 'pitch trim command, normalised', -1, 1)

roll_trim = Property('roll_trim','fcs/roll-trim-cmd-norm', 'roll trim command, normalised', -1, 1)

yaw_trim = Property('yaw_trim','fcs/yaw-trim-cmd-norm', 'yaw trim command, normalised', -1, 1)



# simulation

sim_dt = Property('sim_dt','simulation/dt', 'JSBSim simulation timestep [s]')

sim_time_s = Property('sim_time_s','simulation/sim-time-sec', 'Simulation time [s]')



# initial conditions

initial_altitude_ft = Property('initial_altitude_ft','ic/h-sl-ft', 'initial altitude MSL [ft]', altitude_sl_ft.min, altitude_sl_ft.max)

initial_terrain_altitude_ft = Property('initial_terrain_altitude_ft','ic/terrain-elevation-ft', 'initial terrain alt [ft]')

initial_longitude_geoc_deg = Property('initial_longitude_geoc_deg','ic/long-gc-deg', 'initial geocentric longitude [deg]')

initial_latitude_geod_deg = Property('initial_latitude_geod_deg','ic/lat-geod-deg', 'initial geodesic latitude [deg]')

initial_u_fps = Property('initial_u_fps','ic/u-fps', 'body frame x-axis velocity; positive forward [ft/s]')

initial_v_fps = Property('initial_v_fps','ic/v-fps', 'body frame y-axis velocity; positive right [ft/s]')

initial_w_fps = Property('initial_w_fps','ic/w-fps', 'body frame z-axis velocity; positive down [ft/s]')

initial_p_radps = Property('initial_p_radps','ic/p-rad_sec', 'roll rate [rad/s]')

initial_q_radps = Property('initial_q_radps','ic/q-rad_sec', 'pitch rate [rad/s]')

initial_r_radps = Property('initial_r_radps','ic/r-rad_sec', 'yaw rate [rad/s]')

initial_roc_fpm = Property('initial_roc_fpm','ic/roc-fpm', 'initial rate of climb [ft/min]')

initial_heading_deg = Property('initial_heading_deg','ic/psi-true-deg', 'initial (true) heading [deg]', heading_deg.min, heading_deg.max)

ic_h_agl_ft = Property('ic_h_agl_ft','ic/h-agl-ft', '', altitude_sl_ft.min, altitude_sl_ft.max)

h_agl_ft = Property('h_agl_ft','position/h-agl-ft', '', altitude_sl_ft.min, altitude_sl_ft.max)



# target conditions

target_altitude_ft = Property('target_altitude_ft','tc/h-sl-ft', 'target altitude MSL [ft]', altitude_sl_ft.min, altitude_sl_ft.max)

target_heading_deg = Property('target_heading_deg','tc/target-heading-deg', 'target heading [deg]', heading_deg.min, heading_deg.max)

target_time = Property('target_time','tc/target-time-sec', 'target time [sec]',0)

target_latitude_geod_deg = Property('target_latitude_geod_deg','tc/target-latitude-geod-deg','target geocentric latitude [deg]',-90,90)

target_longitude_geod_deg = Property('target_longitude_geod_deg','tc/target-longitude-geod-deg','target geocentric longitude [deg]',-180,180)


# following path

radius_circle = Property('radius_circle','radius-circle', 'radius of the circle aim to compute target heading for ground procedure [m]', 2.0, 100.0)

shortest_ac_to_path = Property('shortest_ac_to_path','shortest-ac-to-path', 'shortest distance between aircraft and path [m]', 0.0, 100.0)

closest_path_point_lat = Property('closest_path_point_lat','closest_path_point_lat', 'geocentric latitude [deg]', -90, 90)

closest_path_point_lon = Property('closest_path_point_lon','closest_path_point_lon', 'geodesic longitude [deg]', -180, 180)

intersectio_point_lat = Property('intersectio_point_lat','intersectio_point_lat', 'geocentric latitude [deg]', -90, 90)

intersectio_point_lon = Property('intersectio_point_lon','intersectio_point_lon', 'geodesic longitude [deg]', -180, 180)



# define expressions of properties not implemented in JSBSim

custom_properties = { delta_altitude : altitude_sl_ft - target_altitude_ft,

                    delta_heading : heading_deg - target_heading_deg
}

