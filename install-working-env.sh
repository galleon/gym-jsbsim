# Set working env
mkdir $HOME/Docker-Gym-JSBSim
cd $HOME/Docker-Gym-JSBSim
# Clone gym-jsbsim
git clone --branch new https://github.com/galleon/gym-jsbsim.git clean-gym-jsbsim
# Download JSBSim
curl -L https://github.com/JSBSim-Team/jsbsim/archive/JSBSim-trusty-v2018a.tar.gz | tar xz
# Move Docker file in working env root
mv clean-gym-jsbsim/Dockerfile .