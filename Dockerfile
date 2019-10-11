FROM alpine
FROM python:3.7

RUN mkdir /home/gym-jsbsim
WORKDIR /home/gym-jsbsim/gym_jsbsim
RUN git clone https://github.com/JSBSim-Team/jsbsim.git
RUN sed -i -E  '/in.(Mixture|Throttle)Cmd\[[in]\] = in.(Mixture|Throttle)Pos\[[in]\] = 1;/d' jsbsim/src/models/FGPropulsion.cpp
RUN sed -i 's/<expat_config.h>/"expat_config.h"/' jsbsim/src/simgear/xml/xmlparse.c jsbsim/src/simgear/xml/xmlrole.c jsbsim/s

WORKDIR /home/gym-jsbsim
RUN pip install .

CMD ["/bin/bash"]