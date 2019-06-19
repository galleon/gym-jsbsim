# Build Docker
cd $HOME/Docker-Gym-JSBSim
docker build --no-cache -t gym-jsbsim .
docker run -v $HOME/Docker-Gym-JSBSim:/home/nico -it gym-jsbsim
