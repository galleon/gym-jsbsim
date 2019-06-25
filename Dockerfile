FROM alpine
FROM python:3.7

RUN mkdir /home/GJsim
WORKDIR /home/GJsim

RUN apt-get update
RUN pip install --upgrade pip

# APT-GET
RUN apt-get -y install git vim

# PIP pyGRN gym_jsbsim
RUN pip install keras numpy PyYAML tensorflow tqdm Pillow jsbsim keras-rl gym matplotlib

# Rllib
RUN pip install ray[rllib]
RUN pip install ray[debug]
# fix import in ray
#RUN sed -i '1s/^/import gym_jsbsim\n/' /home/nico/anaconda3/envs/tensorflow_p36/lib/python3.6/site-packages/ray/rllib/agents/agent.py

# EXPORT var env
ENV JSBSIM_ROOT_DIR=/home/GJsim/jsbsim-JSBSim-trusty-v2018a
#ENV GYM_JSBSIM=/home/nico/gym-jsbsim
ENV PYTHONPATH=/home/GJsim/clean-gym-jsbsim:$PYTHONPATH

CMD ["/bin/bash"]