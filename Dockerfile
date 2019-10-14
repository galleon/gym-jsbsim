FROM alpine
FROM python:3.7

RUN mkdir /home/gym-jsbsim
WORKDIR /home/gym-jsbsim/gym_jsbsim
RUN git clone https://github.com/JSBSim-Team/jsbsim.git

WORKDIR /home/gym-jsbsim
RUN pip install .

CMD ["/bin/bash"]