### JSBSim Aircraft Simulator for Open AI Gym Environment

This repo allows a full acess to JSBSim properties and provide GYM environments to perform reinforcement learning tasks

So far, is available:

 * [x] Heading Task: *GymJsbsim-HeadingControlTask-A320-v0*: The aircraft should maintain its initial heading and altitude. during the simulation, the aircraft should turn to reach a new random heading.
 * [ ] Altitude and Heading Task: *GymJsbsim-AltitudeHeadingControlTask-A320-v0*: The aircraft should maintain its initial heading and altitude. During the simulation, the aircraft should reach a new random heading AND and new random altitude.
 * [ ] Path plan task: *GymJsbsim-PathPlanTask-A320-v0*: The aircraft should change heading and altitude during the simulation to follow a path plan trajectory.
 * [ ] Taxi task: *GymJsbsim-TaxiTask-A320-v0*: The aircraft should follow a predifined trajectory on the runaway
 * [ ] Take Off Task: *GymJsbsim-TakeOffTask-A320-v0*: The aircraft should take off and reach a selected altitude.
 
 
 ## Instalation
 
 The full gym-jsbsim environment is embeded in a Docker container that is locally synchronise with our repository for easy pull and push git events.
 
 The following script will setup and download prerequisities for the docker instalation
```
wget https://raw.githubusercontent.com/galleon/gym-jsbsim/new/install-working-env.sh -O install-working-env.sh
sh install-working-env.sh
```
Once the first script run, you could launch the second script to build and run the docker
```
wget https://raw.githubusercontent.com/galleon/gym-jsbsim/new/install-docker.sh -O install-docker.sh
sh install-docker.sh
```

Your working env is under ```$HOME/Docker-Gym-JSBSim```. All modifications will be directly synchronise with the docker.
You could have acces to your docker (without rebuild it) using:
```
docker run -v $HOME/Docker-Gym-JSBSim:/home/GJsim -it gym-jsbsim
```

## Test

You could run a random agent with 
```
python $HOME/Docker-Gym-JSBSim/clean-gym-jsbsim/test_agent.py
```






