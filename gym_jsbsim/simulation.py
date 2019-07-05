from collections import namedtuple
import re
import jsbsim
from gym_jsbsim.catalogs.jsbsim_catalog import JsbsimCatalog
from gym_jsbsim.catalogs.my_catalog import MyCatalog
from gym_jsbsim.catalogs.catalog import Catalog
from gym_jsbsim import ROOT_DIR


class Simulation:
    """

    A class which wraps an instance of JSBSim and manages communication with it.

    """

    def __init__(self, aircraft_name="A320", init_conditions=None, jsbsim_freq = 60, agent_interaction_steps = 5):
        """

        Constructor. Creates an instance of JSBSim and sets initial conditions.


        :param aircraft_name: name of aircraft to be loaded.

            JSBSim looks for file '\model_name\model_name.xml' from root dir.

        :param init_conditions: dict mapping properties to their initial values.

            Defaults to None, causing a default set of initial props to be used.

        """
        self.aircraft_name = aircraft_name

        self.jsbsim_exec = None

        self.agent_interaction_steps = None

        self.initialise(init_conditions,jsbsim_freq,agent_interaction_steps)



    def initialise(self, init_conditions=None, jsbsim_freq = 60, agent_interaction_steps = 5):
        """

         Loads an aircraft and initial conditions.
         Initialises all jsbsim properties.

        :param init_conditions: dict mapping properties to their initial values

        """
        self.close()

        self.jsbsim_exec = jsbsim.FGFDMExec(ROOT_DIR)
        self.jsbsim_exec.set_debug_level(0)  # requests JSBSim not to output any messages whatsoever

        self.jsbsim_exec.load_model(self.aircraft_name)

        # set jsbsim integration time step
        dt = 1 / jsbsim_freq
        self.jsbsim_exec.set_dt(dt)

        self.agent_interaction_steps = agent_interaction_steps

        self.set_initial_conditions(init_conditions)
        success = self.jsbsim_exec.run_ic()
        self.jsbsim_exec.propulsion_init_running(-1)

        if not success:
            raise RuntimeError('JSBSim failed to init simulation conditions.')



    def set_initial_conditions(self, init_conditions=None):
        """

        Loads init_conditions values in JSBSim.

        :param init_conditions: dict mapping properties to their initial values

        """

        if init_conditions is not None:
            for prop, value in init_conditions.items():
                self.set_property_value(prop, value)



    def run(self):
        """

        Runs JSBSim simulation until the agent interacts and update custom properties.


        JSBSim monitors the simulation and detects whether it thinks it should

        end, e.g. because a simulation time was specified. False is returned

        if JSBSim termination criteria are met.



        :return: bool, False if sim has met JSBSim termination criteria else True.

        """
        for _ in range(self.agent_interaction_steps):
            result = self.jsbsim_exec.run()
            if not result:
                raise RuntimeError('JSBSim failed.')
        return result



    def get_sim_time(self):
        """ Gets the simulation time from JSBSim, a float. """

        return self.jsbsim_exec.get_sim_time()



    def close(self):
        """ Closes the simulation and any plots. """

        if self.jsbsim_exec:
            self.jsbsim_exec = None



    def get_property_values(self, props):
        """

        Get the values of the specified properties

        :param props: list of Properties

        : return: NamedTuple with properties name and their values

        """
        Props = namedtuple("NamedTuple", [prop.name for prop in props])
        return Props(*[self.get_property_value(prop) for prop in props])



    def set_property_values(self, props, values):
        """

        Set the values of the specified properties

        :param props: list of Properties

        :param values: list of float

        """
        if not len(props) == len(values):
            raise ValueError('mismatch between properties and values size')
        for prop, value in zip(props, values):
            self.set_property_value(prop, value)



    def get_property_value(self, prop):
        """
        Get the value of the specified property from the JSBSim simulation

        :param prop: Property

        :return : float
        """

        if prop.__class__ is Catalog:
            MyCatalog.update_custom_properties(self,prop)
        try :
            return self.jsbsim_exec[prop.name_jsbsim]
        except KeyError:
            MyCatalog.init_custom_properties(self, prop)
            return self.jsbsim_exec.get_property_value(prop.name_jsbsim)



    def set_property_value(self, prop, value):
        """
        Set the values of the specified property

        :param prop: Property

        :param value: float

        """

        # set value in property bounds
        if value < prop.min:
            value = prop.min
        elif value > prop.max:
            value = prop.max

        equal_engine_properties = [JsbsimCatalog.fcs_throttle_cmd_norm,
                                   JsbsimCatalog.fcs_throttle_pos_norm,
                                   JsbsimCatalog.fcs_mixture_cmd_norm,
                                   JsbsimCatalog.fcs_mixture_pos_norm,
                                   JsbsimCatalog.fcs_advance_pos_norm,
                                   JsbsimCatalog.fcs_advance_cmd_norm,
                                   JsbsimCatalog.fcs_feather_pos_norm,
                                   JsbsimCatalog.fcs_feather_cmd_norm ]

        if prop in equal_engine_properties :
            for i in range(self.jsbsim_exec.propulsion_get_num_engines()):
                self.jsbsim_exec.set_property_value(prop.name_jsbsim + "[" + str(i) + "]", value)

        else:
            self.jsbsim_exec.set_property_value(prop.name_jsbsim, value)

        if prop.__class__ is Catalog:
            MyCatalog.update_custom_properties(self,prop)


    def get_state(self):
        full_jsbsim_state = self.jsbsim_exec.get_property_catalog('')
        full_state = dict()
        for prop_jsbsim,value in full_jsbsim_state.items():
            prop_name = re.sub(r"[\-/\]\[]+",'_',prop_jsbsim)
            try:
                full_state[JsbsimCatalog[prop_name]] = value
            except KeyError:
                pass
        for prop in MyCatalog:
            try:
                full_state[prop] = self.jsbsim_exec[prop.name_jsbsim]
            except KeyError:
                pass
        return full_state

    def set_state(self,state):
        init_conditions = {}

        state_to_ic = {JsbsimCatalog.position_lat_gc_deg: JsbsimCatalog.ic_lat_gc_deg,
                       JsbsimCatalog.position_long_gc_deg:  JsbsimCatalog.ic_long_gc_deg,
                       JsbsimCatalog.position_h_sl_ft: JsbsimCatalog.ic_h_sl_ft,
                       JsbsimCatalog.position_h_agl_ft:  JsbsimCatalog.ic_h_agl_ft,
                       JsbsimCatalog.position_terrain_elevation_asl_ft: JsbsimCatalog.ic_terrain_elevation_ft,
                       JsbsimCatalog.attitude_psi_deg: JsbsimCatalog.ic_psi_true_deg,
                       JsbsimCatalog.attitude_theta_deg: JsbsimCatalog.ic_theta_deg,
                       JsbsimCatalog.attitude_phi_deg: JsbsimCatalog.ic_phi_deg,
                       JsbsimCatalog.velocities_u_fps: JsbsimCatalog.ic_u_fps,
                       JsbsimCatalog.velocities_v_fps: JsbsimCatalog.ic_v_fps,
                       JsbsimCatalog.velocities_w_fps: JsbsimCatalog.ic_w_fps,
                       JsbsimCatalog.velocities_p_rad_sec: JsbsimCatalog.ic_p_rad_sec,
                       JsbsimCatalog.velocities_q_rad_sec: JsbsimCatalog.ic_q_rad_sec,
                       JsbsimCatalog.velocities_r_rad_sec: JsbsimCatalog.ic_r_rad_sec,
                    }

        for prop,value in state.items():
            if prop.__class__ is JsbsimCatalog:
                if not re.match(r'^ic_', prop.name):
                    if prop in state_to_ic:
                        init_conditions[state_to_ic[prop]] = value

                    query_prop =  self.jsbsim_exec.query_property_catalog(prop.name_jsbsim)
                    while query_prop[0].split()[0] != prop.name_jsbsim :
                        query_prop.pop(0)

                    status =query_prop[0].split()[1]
                    if 'RW' in status:
                        init_conditions[prop] = value
            else:
                init_conditions[prop] = value

        return init_conditions
