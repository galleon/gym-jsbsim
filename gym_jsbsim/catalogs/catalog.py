from enum import Enum
from gym_jsbsim.catalogs.property import Property
from gym_jsbsim.catalogs.jsbsim_catalog import JsbsimCatalog
from gym_jsbsim.catalogs.my_catalog import MyCatalog

Catalog = Enum(value='Catalog', names=dict(**JsbsimCatalog.__members__, **MyCatalog.__members__), type=Property)
