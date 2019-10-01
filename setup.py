from setuptools import setup, find_packages
from distutils.extension import Extension
from distutils.util import get_platform

with open('README.md') as f:
    long_description = f.read()

libraries = []
if get_platform() == 'mingw':
    libraries = ['ws2_32']

setup(
    name='gym_jsbsim',
    version='1.0.0',
    url='https://github.com/galleon/gym-jsbsim/tree/new/gym_jsbsim',
    author='John Doe',
    author_email='john.doe@gmail.com',
    license='LGPL 2.1',
    description='Gym JSBSim environment',
    long_description=long_description,
    packages=find_packages(),
    ext_modules=[Extension('jsbsim', ['jsbsim/python/jsbsim.pyx', 'jsbsim/src/FGFDMExec.cpp','jsbsim/src/FGJSBBase.cpp','jsbsim/src/initialization/FGInitialCondition.cpp','jsbsim/src/initialization/FGTrim.cpp','jsbsim/src/initialization/FGTrimAxis.cpp','jsbsim/src/models/atmosphere/FGMSIS.cpp','jsbsim/src/models/atmosphere/FGMSISData.cpp','jsbsim/src/models/atmosphere/FGMars.cpp','jsbsim/src/models/atmosphere/FGStandardAtmosphere.cpp','jsbsim/src/models/atmosphere/FGWinds.cpp','jsbsim/src/models/flight_control/FGDeadBand.cpp','jsbsim/src/models/flight_control/FGFCSComponent.cpp','jsbsim/src/models/flight_control/FGFilter.cpp','jsbsim/src/models/flight_control/FGGain.cpp','jsbsim/src/models/flight_control/FGKinemat.cpp','jsbsim/src/models/flight_control/FGSummer.cpp','jsbsim/src/models/flight_control/FGSwitch.cpp','jsbsim/src/models/flight_control/FGFCSFunction.cpp','jsbsim/src/models/flight_control/FGSensor.cpp','jsbsim/src/models/flight_control/FGPID.cpp','jsbsim/src/models/flight_control/FGActuator.cpp','jsbsim/src/models/flight_control/FGAccelerometer.cpp','jsbsim/src/models/flight_control/FGGyro.cpp','jsbsim/src/models/flight_control/FGLinearActuator.cpp','jsbsim/src/models/flight_control/FGMagnetometer.cpp','jsbsim/src/models/flight_control/FGAngles.cpp','jsbsim/src/models/flight_control/FGWaypoint.cpp','jsbsim/src/models/flight_control/FGDistributor.cpp','jsbsim/src/models/propulsion/FGElectric.cpp','jsbsim/src/models/propulsion/FGEngine.cpp','jsbsim/src/models/propulsion/FGForce.cpp','jsbsim/src/models/propulsion/FGNozzle.cpp','jsbsim/src/models/propulsion/FGPiston.cpp','jsbsim/src/models/propulsion/FGPropeller.cpp','jsbsim/src/models/propulsion/FGRocket.cpp','jsbsim/src/models/propulsion/FGTank.cpp','jsbsim/src/models/propulsion/FGThruster.cpp','jsbsim/src/models/propulsion/FGTurbine.cpp','jsbsim/src/models/propulsion/FGTurboProp.cpp','jsbsim/src/models/propulsion/FGTransmission.cpp','jsbsim/src/models/propulsion/FGRotor.cpp','jsbsim/src/models/FGAerodynamics.cpp','jsbsim/src/models/FGAircraft.cpp','jsbsim/src/models/FGAtmosphere.cpp','jsbsim/src/models/FGAuxiliary.cpp','jsbsim/src/models/FGFCS.cpp','jsbsim/src/models/FGSurface.cpp','jsbsim/src/models/FGGroundReactions.cpp','jsbsim/src/models/FGInertial.cpp','jsbsim/src/models/FGLGear.cpp','jsbsim/src/models/FGMassBalance.cpp','jsbsim/src/models/FGModel.cpp','jsbsim/src/models/FGOutput.cpp','jsbsim/src/models/FGPropagate.cpp','jsbsim/src/models/FGPropulsion.cpp','jsbsim/src/models/FGInput.cpp','jsbsim/src/models/FGExternalReactions.cpp','jsbsim/src/models/FGExternalForce.cpp','jsbsim/src/models/FGBuoyantForces.cpp','jsbsim/src/models/FGGasCell.cpp','jsbsim/src/models/FGAccelerations.cpp','jsbsim/src/math/FGColumnVector3.cpp','jsbsim/src/math/FGFunction.cpp','jsbsim/src/math/FGLocation.cpp','jsbsim/src/math/FGMatrix33.cpp','jsbsim/src/math/FGPropertyValue.cpp','jsbsim/src/math/FGQuaternion.cpp','jsbsim/src/math/FGRealValue.cpp','jsbsim/src/math/FGTable.cpp','jsbsim/src/math/FGCondition.cpp','jsbsim/src/math/FGRungeKutta.cpp','jsbsim/src/math/FGModelFunctions.cpp','jsbsim/src/math/FGTemplateFunc.cpp','jsbsim/src/input_output/FGGroundCallback.cpp','jsbsim/src/input_output/FGPropertyManager.cpp','jsbsim/src/input_output/FGScript.cpp','jsbsim/src/input_output/FGXMLElement.cpp','jsbsim/src/input_output/FGXMLParse.cpp','jsbsim/src/input_output/FGfdmSocket.cpp','jsbsim/src/input_output/FGOutputType.cpp','jsbsim/src/input_output/FGOutputFG.cpp','jsbsim/src/input_output/FGOutputSocket.cpp','jsbsim/src/input_output/FGOutputFile.cpp','jsbsim/src/input_output/FGOutputTextFile.cpp','jsbsim/src/input_output/FGPropertyReader.cpp','jsbsim/src/input_output/FGModelLoader.cpp','jsbsim/src/input_output/FGInputType.cpp','jsbsim/src/input_output/FGInputSocket.cpp','jsbsim/src/input_output/FGUDPInputSocket.cpp','jsbsim/src/simgear/props/props.cxx','jsbsim/src/simgear/props/propertyObject.cxx','jsbsim/src/simgear/xml/easyxml.cxx','jsbsim/src/simgear/xml/xmlparse.c','jsbsim/src/simgear/xml/xmltok.c','jsbsim/src/simgear/xml/xmlrole.c','jsbsim/src/simgear/magvar/coremag.cxx','jsbsim/src/simgear/misc/sg_path.cxx','jsbsim/src/simgear/misc/strutils.cxx','jsbsim/src/simgear/io/iostreams/sgstream.cxx',],
                           include_dirs=['jsbsim/src'],
                           libraries=libraries,
                           extra_compile_args=['-DJSBSIM_VERSION=\"1.0.0.dev1\"',
                                               '-DHAVE_EXPAT_CONFIG_H',
                                               '-std=c++11'
                                               ],
                           language='c++')],
    install_requires=['cython>=0.25','gym>=0.12.5'],
    setup_requires=['cython>=0.25','gym>=0.12.5'])
