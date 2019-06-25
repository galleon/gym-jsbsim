# Build Docker
cd $HOME/Docker-Gym-JSBSim
docker build -t gym-jsbsim .
docker run -v $HOME/Docker-Gym-JSBSim:/home/GJsim -it gym-jsbsim
