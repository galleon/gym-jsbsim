FROM alpine
FROM python:3.7

RUN mkdir /home/GJsim
WORKDIR /home/GJsim

RUN apt-get update
RUN pip install --upgrade pip

# APT-GET
RUN apt-get -y install git vim htop bc psmisc graphviz

# PIP pyGRN gym_jsbsim
RUN pip install numpy tqdm jsbsim gym matplotlib geopandas pyshp joblib 

# EXPORT var env
ENV JSBSIM_ROOT_DIR=/home/GJsim/jsbsim-JSBSim-trusty-v2018a
#ENV GYM_JSBSIM=/home/nico/gym-jsbsim
ENV PYTHONPATH=/home/GJsim/clean-gym-jsbsim:$PYTHONPATH

CMD ["/bin/bash"]