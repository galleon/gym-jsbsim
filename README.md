# JSBSim Aircraft Simulator for Open AI Gym Environment

## Are we able to learn an aircraft to fly (or roll)

This project aims at creating realistic open AI GYM environements using the open source Flight Dynamic Model JSBSim.


## Instalation



## Environments

So far, is available:

 * [x] Heading Task: *GymJsbsim-HeadingControlTask-v0*: The aircraft should maintain its initial heading and altitude. during the simulation, the aircraft should turn to reach a new heading every 150 secondes with an incremental difficulty.
 * [x] Taxi task: *GymJsbsim-TaxiControlTask-v0*: The aircraft should follow a predifined trajectory on the runaway. 
 * [x] Taxi with AutoPilot task: *GymJsbsim-TaxiapControlTask-v0*: The aircraft should follow a predifined trajectory on the runaway
 * [x] Approach Task: *GymJsbsim-ApproachControlTask-v0*: The aircraft should take off and reach a selected altitude.
 
## Heading Task

```
env = gym.make("GymJsbsim-HeadingControlTask-v0")   
```

In this task, the aircraft should perform a stable steady flight following its initial heading and altitude. Every 150 secondes, a new target heading is set. At each time the target heading is more and more complecated to reach, starting with a delta of 10° to finish with a delta of 90°, alterning left and right turn:

So, the scenario is as follow:

 * initial heading = 100°
 * [0, 150]: target heading = 100° (delta heading = 0°)
 * [150, 300]: target heading = 110° (delta heading = +10°)
 * [300, 450]: target heading = 90° (delta heading = -20°)
 * [450, 600]: target heading = 120° (delta heading = +30°)
 * [600, 750]: target heading = 80° (delta heading = -40°)
 * [750, 900]: target heading = 130° (delta heading = +50°)
 * [900, 1050]: target heading = 70° (delta heading = -60°)
 * [1050, 1200]: target heading = 140° (delta heading = +70°)
 * [1200, 1350]: target heading = 60° (delta heading = -80°)
 * [1350, 1500]: target heading = 150° (delta heading = +90°)

Terminal conditions are the following:
 * If the aircraft is up or under 300 feets from its target altitude, the scenario is over.
 * If the target heading is not reach with an accuracy of 10° during the 150 secondes, the scenario is over.
 * The full scenario will not exeed 1500 secondes.

The input state set is a vector of 7 parameters:

```
state_var = [c.delta_altitude, # the delta altitude between the aircraft and target altitude
             c.delta_heading, # the delta heading between the aircraft and target heading
             c.velocities_v_down_fps, # the vertical velocity of the aircraft
             c.velocities_vc_fps, # the air velocity of the aircraft
             c.velocities_p_rad_sec, # roll axis velocity
             c.velocities_q_rad_sec, # pitch axis velocity
             c.velocities_r_rad_sec] # yaw axis velocity
```

The action set is a vector of 4 parameters:

```
action_var = [c.fcs_aileron_cmd_norm, 
              c.fcs_elevator_cmd_norm,
              c.fcs_rudder_cmd_norm,
              c.fcs_throttle_cmd_norm]
```

The reward is compute as a weighted function between:
 * accuracy of aircraft altitude in regards to the target
 * accuracy of aircraft heading in regards to the target
 * stability of the aircraft in regards to the 3 axes acceleration feeling by the pilot in the cockpit

## Taxi Task

```
env = gym.make("GymJsbsim-TaxiControlTask-v0")   
```

In this environnement, the aircraft should behave on ground and following a specific path, trying to be closest as possible to the runaway centerline.

For this environement, we have extract a path from the AMDB files of blagnac airport:

<p align="center">
  <img width="600" height="400" src=./gym_jsbsim/docs/l2f_taxi_path.png>
</p>

and extracted from this path a list of geodesic coordinates that the aircraft should follow.

As state set for this environment, we compute every timestep, the distance (di) and angle (ai) to the next 4 path points according to location of the aircraft (ie: d1 to d4 and a1 to a4)

<p align="center">
  <img src=./gym_jsbsim/docs/l2f_taxi_state.png>
</p>


The full state set is:

```
state_var = [c.velocities_vc_fps,
             c.shortest_dist,
             c.d1, c.d2, c.d3, c.d4,
             c.a1, c.a2, c.a3, c.a4]
```

The action set is define with 3 parameters that correspond to throttle, brake and steer commands:

```
action_var = [c.fcs_steer_cmd_norm,
              c.fcs_center_brake_cmd_norm,
              c.fcs_throttle_cmd_norm]
```

the reward is simply computed with the distance to the centerline

The scenario is over if the aircraft overtake the centerline from 10 meters or more.

## Taxi Auto Pilot Task

```
env = gym.make("GymJsbsim-TaxiapControlTask-v0")   
```

This is the same Taxi environement with an autopilot to manage the aircraft velocity.   
The velocity during straight line is set to 20 knots and the velocity during turn is set to 7 knots.

The state set is unchange:

```
state_var = [c.velocities_vc_fps,
             c.shortest_dist,
             c.d1, c.d2, c.d3, c.d4,
             c.a1, c.a2, c.a3, c.a4]
```

but the action set is therefor focus on the steering command only:

```
action_var = [c.fcs_steer_cmd_norm]
```

the reward is unchanged and computed with the distance to the centerline


## Test

You could run a random agent with 
```
python test_agent.py
```

## Results


## Links

Open AI GYM: https://github.com/openai/gym  
JSBSim: https://github.com/JSBSim-Team/jsbsim  
Gor-Ren Gym-Jsbsim repo: https://github.com/Gor-Ren/gym-jsbsim






