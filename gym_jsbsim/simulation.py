# -*- coding: utf-8 -*-

import jsbsim
import gym_jsbsim.simulation_parameters as param
from collections import namedtuple
from gym_jsbsim.properties import custom_properties, throttle_cmd, mixture_cmd

class Simulation(object):
    """

    A class which wraps an instance of JSBSim and manages communication with it.

    """
    
    def __init__(self, aircraft_name = "A320", init_conditions = None ):
        """

        Constructor. Creates an instance of JSBSim and sets initial conditions.



        :param aircraft_name: name of aircraft to be loaded.

            JSBSim looks for file \model_name\model_name.xml from root dir.

        :param init_conditions: dict mapping properties to their initial values.

            Defaults to None, causing a default set of initial props to be used.
            
        """
        
        self.jsbsim_exec = jsbsim.FGFDMExec()
        self.jsbsim_exec.set_debug_level(0) #requests JSBSim not to output any messages whatsoever
        self.aircraft_name = aircraft_name
        
        self.jsbsim_exec.set_root_dir(param.ROOT_DIR)
        self.jsbsim_exec.set_aircraft_path(param.AircraftPath)
        self.jsbsim_exec.set_engine_path(param.EnginePath)
        self.jsbsim_exec.set_systems_path(param.SystemPath)

        dt = 1/param.jsbsim_freq
        self.jsbsim_exec.set_dt(dt)

        self.initialise(init_conditions)

        

        
    def initialise(self, init_conditions = None):
        """

         Loads an aircraft and initialises simulation conditions.

        :param init_conditions: dict mapping properties to their initial values

        """

        self.jsbsim_exec.load_model(self.aircraft_name)

        self.set_initial_conditions(init_conditions)

        self.update_custom_properties()
        success = self.jsbsim_exec.run_ic()
        self.jsbsim_exec.propulsion_init_running(-1)
        self.update_custom_properties()

        if not success:
            raise RuntimeError('JSBSim failed to init simulation conditions.')

        
        
    def set_initial_conditions(self, init_conditions = None):
        """
        Loads init_conditions values in JSBSim 
        
        :param init_conditions: dict mapping properties to their initial values
        """
        if init_conditions is not None:
            for prop,value in init_conditions.items():
                self.set_property_value(prop,value)




    def run(self):

        """

        Runs JSBSim simulation until the agent interacts and update custom properties.


        JSBSim monitors the simulation and detects whether it thinks it should

        end, e.g. because a simulation time was specified. False is returned

        if JSBSim termination criteria are met.



        :return: bool, False if sim has met JSBSim termination criteria else True.

        """
        self.update_custom_properties()
        for _ in range(param.agent_interaction_steps):
            result = self.jsbsim_exec.run()
            if not result:
                raise RuntimeError('JSBSim failed.')
        self.update_custom_properties()
        return result


    def get_sim_time(self):
        """ Gets the simulation time from JSBSim, a float. """
        
        return self.jsbsim_exec.get_sim_time()
    
    
    def close(self):

        """ Closes the simulation and any plots. """

        if self.jsbsim_exec:
            self.jsbsim_exec = None

            
            
    def get_property_values(self,props):
        """
        Get the values of the specified properties
        
        :param props: list of Properties
        
        : return: NamedTuple with properties name and their values
        """
        Props = namedtuple("NamedTuple",[prop.name for prop in props])
        return Props(*[self.get_property_value(prop) for prop in props])
    
    
    def set_property_values(self,props,values):
        """
        Set the values of the specified properties
        
        :param props: list of Properties
        
        :param values: list of values
        
        """
        if not (len(props) == len(values)):
            raise ValueError('mismatch between properties and values size')
        for i in range(len(props)):
            self.set_property_value(props[i],values[i])


    def get_property_value(self,prop):
        return self.jsbsim_exec.get_property_value(prop.name_jsbsim)

    def set_property_value(self,prop,value):
        #set value in bounds property
        if value < prop.min :
            value = prop.min
        elif value > prop.max :
            value = prop.max

        # set all throttles
        if prop == throttle_cmd :
            for i in range(self.jsbsim_exec.propulsion_get_num_engines()):
                self.jsbsim_exec.set_property_value("fcs/throttle-cmd-norm["+str(i)+"]", value)
        #set all mixtures
        if prop == mixture_cmd :
            for i in range(self.jsbsim_exec.propulsion_get_num_engines()):
                self.jsbsim_exec.set_property_value("fcs/mixture-cmd-norm[" + str(i) + "]", value)

        else :
            self.jsbsim_exec.set_property_value(prop.name_jsbsim, value)


    def update_custom_property(self,prop):
        custom_properties[prop](self)

    def update_custom_properties(self):
        for prop in custom_properties:
           self.update_custom_property(prop)
