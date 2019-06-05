# -*- coding: utf-8 -*-

import jsbsim
import gym_jsbsim.simulation_parameters as param
from collections import namedtuple
from gym_jsbsim.properties import gear



class Simulation(object):
    """

    A class which wraps an instance of JSBSim and manages communication with it.

    """
    
    def __init__(self, aircraft_name = "A320", init_conditions = None, num_engine =-1):
        """

        Constructor. Creates an instance of JSBSim and sets initial conditions.



        :param aircraft_name: name of aircraft to be loaded.

            JSBSim looks for file \model_name\model_name.xml from root dir.

        :param init_conditions: dict mapping properties to their initial values.

            Defaults to None, causing a default set of initial props to be used.
          
        :param num_engine: number of the engine to be run. All engines if -1.  
            
        """
        
        self.jsbsim_exec = jsbsim.FGFDMExec()
        self.jsbsim_exec.set_debug_level(0) #requests JSBSim not to output any messages whatsoever
        self.aircraft_name = aircraft_name
        
        self.jsbsim_exec.set_root_dir(param.ROOT_DIR)
        self.jsbsim_exec.set_aircraft_path(param.AircraftPath)
        self.jsbsim_exec.set_engine_path(param.EnginePath)
        self.jsbsim_exec.set_systems_path(param.SystemPath)
        
        self.jsbsim_exec.load_model(self.aircraft_name)
        self.jsbsim_exec.load_ic(param.IC_FILE,param.useStoredPath)
        
        dt = 1/param.jsbsim_freq
        self.jsbsim_exec.set_dt(dt)
        
        self.initialise(init_conditions,num_engine)
        

        
    def initialise(self, init_conditions = None, num_engine = -1):
        """

        Loads an aircraft and initialises simulation conditions.


        JSBSim creates an InitialConditions object internally when given an

        XML config file. This method loads a minimal IC

        XML file, and then the dictionary values are fed in.

        :param init_conditions: dict mapping properties to their initial values
        
        :param num_engine: number of the engine to be run. All engines if -1.
        
        """
        
        self.set_initial_conditions(init_conditions)
        
        
        #self.jsbsim_exec.propulsion_init_running(num_engine)
        success = self.jsbsim_exec.run_ic()

        if not success:
            raise RuntimeError('JSBSim failed to init simulation conditions.')
            
        self.jsbsim_exec.propulsion_init_running(-1)
            
        
        
    def set_initial_conditions(self, init_conditions = None):
        """
        Loads init_conditions values in JSBSim 
        
        :param init_conditions: dict mapping properties to their initial values
        """
        if init_conditions != None:
            for prop,value in init_conditions.items():
                self.jsbsim_exec.set_property_value(prop.name_jsbsim,value)
        self.jsbsim_exec.set_property_value(gear.name_jsbsim,1)
        
        
    def run(self) -> bool:

        """

        Runs a single timestep in the JSBSim simulation.



        JSBSim monitors the simulation and detects whether it thinks it should

        end, e.g. because a simulation time was specified. False is returned

        if JSBSim termination criteria are met.



        :return: bool, False if sim has met JSBSim termination criteria else True.

        """
        success = self.jsbsim_exec.run()
        
        if not success:
            raise RuntimeError('JSBSim failed to init simulation conditions.')

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
        return Props(*[self.jsbsim_exec.get_property_value(prop.name_jsbsim) for prop in props])
    
    
    def set_property_values(self,props,values):
        """
        Set the values of the specified properties
        
        :param props: list of Properties
        
        :param values: list of values
        
        """
        if not (len(props) == len(values)):
            raise ValueError('mismatch between properties and values size')
        for i in range(len(props)):
            self.jsbsim_exec.set_property_value(props[i].name_jsbsim,values[i])
        
        